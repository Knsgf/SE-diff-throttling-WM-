using System;
using System.Collections.Generic;
using System.IO;

using Sandbox.Game.Entities;
using Sandbox.ModAPI;
using VRage.Game;
using VRage.Game.ModAPI;
using VRage.Utils;

namespace ttdtwm
{
    public struct display_settings
    {
        public bool show_thrust_reduction, show_vertical_speed;
        public uint min_displayed_reduction;
    };

    static class screen_info
    {
        const string settings_file = "TTDTWM.CFG";
        private static readonly char[] whitespace_char = { ' ', '\t' };

        const string CONTROL_LOSS_WARNING = "/tpdt-cw";
        const string THRUST_LOSS          = "/tpdt-tl";
        const string VERTICAL_SPEED       = "/tpdt-vs";

        private class HUD_notification
        {
            public IMyHudNotification contents;
            public string             screen_description;
            public bool               toggled_on, currently_visible, set_to_visible;
        }

        private static Dictionary<string, HUD_notification> _HUD_messages = new Dictionary<string, HUD_notification>();
        //private static display_settings _display_settings;
        private static bool _UI_handlers_registered = false, _settings_loaded = false;
        private static uint _min_displayed_reduction = 10;

        #region Properties

        public static IMyPlayer local_player { get; private set; }
        public static VRage.Game.ModAPI.Interfaces.IMyControllableEntity local_controller { get; private set; }

        #endregion

        #region Debug display

        static private void display_info(MyCubeGrid grid, string message, int display_time_ms, string font, bool controlled_only)
        {
            bool display = true;

            if (controlled_only)
            {
                var controller = MyAPIGateway.Session.ControlledObject as MyShipController;
                display = controller != null && controller.CubeGrid == grid;
            }
            if (display)
                MyAPIGateway.Utilities.ShowNotification(message, display_time_ms, font);
        }

        static public void screen_text(MyCubeGrid grid, string method_name, string message, int display_time_ms, bool controlled_only)
        {
            string grid_name = (grid == null) ? "" : ("\"" + grid.DisplayName + "\"");
            if (method_name == "")
                display_info(grid, string.Format("{0} {1}", controlled_only ? "" : grid_name, message), display_time_ms, MyFontEnum.White, controlled_only);
            else
                display_info(grid, string.Format("{0}(): {1} {2}", method_name, controlled_only ? "" : grid_name, message), display_time_ms, MyFontEnum.White, controlled_only);
        }

        #endregion

        #region HUD messages

        static private void register_HUD_notification(string message_cmd, string screen_description, bool toggled_on, string default_text = "", string default_colour = MyFontEnum.White)
        {
            var new_message = new HUD_notification();
            new_message.contents           = MyAPIGateway.Utilities.CreateNotification(default_text, 0, default_colour);
            new_message.currently_visible  = false;
            new_message.toggled_on         = toggled_on;
            new_message.screen_description = screen_description;
            _HUD_messages[message_cmd]     = new_message;
        }

        public static void set_control_loss_warning_visibility(bool is_visible)
        {
            _HUD_messages[CONTROL_LOSS_WARNING].set_to_visible = is_visible;
        }

        public static void set_displayed_thrust_reduction(uint thrust_reduction, bool is_single_grid_assembly)
        {
            bool is_visible = is_single_grid_assembly && thrust_reduction > _min_displayed_reduction;

            _HUD_messages[THRUST_LOSS].set_to_visible = is_visible;
            if (is_visible)
            {
                _HUD_messages[THRUST_LOSS].contents.Text = "Thrust loss: " + thrust_reduction.ToString() + " %";
                _HUD_messages[THRUST_LOSS].contents.Font = (thrust_reduction > 30) ? MyFontEnum.Red : MyFontEnum.White;
            }
        }

        public static void set_displayed_vertical_speed(float vertical_speed, bool is_primary_grid)
        {
            bool is_visible = is_primary_grid && Math.Abs(vertical_speed) >= 0.05f;

            _HUD_messages[VERTICAL_SPEED].set_to_visible = is_visible;
            if (is_visible)
                _HUD_messages[VERTICAL_SPEED].contents.Text = string.Format("Vertical speed: {0:F1} m/s", vertical_speed);
        }

        #endregion

        #region Chat command handling

        private static void extract_command_and_parameter(string input, out string command, out string parameter)
        {
            string[] split_input = input.Trim().ToLower().Split(whitespace_char, 2);
            command   = split_input[0];
            parameter = (split_input.Length > 1) ? split_input[1] : "";
        }

        private static void command_handler(string message, ref bool dummy)
        {
            string command, parameter;

            extract_command_and_parameter(message, out command, out parameter);
            if (command != THRUST_LOSS || parameter.Length <= 0)
            {
                if (!_HUD_messages.ContainsKey(command))
                    return;
                _HUD_messages[command].toggled_on = !_HUD_messages[command].toggled_on;
                MyAPIGateway.Utilities.ShowMessage("TP&DT", string.Format("{0} is now {1}", _HUD_messages[command].screen_description, _HUD_messages[command].toggled_on ? "visible" : "hidden"));
            }
            else
            {
                uint min_thrust_loss  = 0; 
                bool min_loss_entered = false;

                min_loss_entered = uint.TryParse(parameter, out min_thrust_loss);
                if (!min_loss_entered)
                    return;
                if (min_thrust_loss > 100)
                    min_thrust_loss = 100;
                _min_displayed_reduction = min_thrust_loss;
                MyAPIGateway.Utilities.ShowMessage("TP&DT", string.Format("Minimum displayed thrust loss is now {0} %", min_thrust_loss));
            }

            try
            {
                display_settings stored_settings = new display_settings
                {
                    show_thrust_reduction   = _HUD_messages[   THRUST_LOSS].toggled_on,
                    show_vertical_speed     = _HUD_messages[VERTICAL_SPEED].toggled_on,
                    min_displayed_reduction = _min_displayed_reduction
                };
                TextWriter output = MyAPIGateway.Utilities.WriteFileInLocalStorage(settings_file, typeof(display_settings));
                output.Write(MyAPIGateway.Utilities.SerializeToXML(stored_settings));
                output.Close();
            }
            catch (Exception error)
            {
                MyLog.Default.WriteLine(error);
            }
        }

        #endregion

        public static void try_register_handlers()
        {
            if (!_UI_handlers_registered && MyAPIGateway.Utilities != null)
            {
                MyAPIGateway.Utilities.MessageEntered += command_handler;
                register_HUD_notification(CONTROL_LOSS_WARNING,   "Control loss warning",  true, "WARNING: Control loss imminent", MyFontEnum.Red);
                register_HUD_notification(         THRUST_LOSS, "Thrust loss indication",  true);
                register_HUD_notification(      VERTICAL_SPEED, "Vertical speed readout", false);
                _UI_handlers_registered = true;
            }

            if (!_settings_loaded && _UI_handlers_registered)
            {
                if (MyAPIGateway.Utilities.FileExistsInLocalStorage(settings_file, typeof(display_settings)))
                {
                    display_settings stored_settings = new display_settings
                    {
                        show_thrust_reduction   = true,
                        show_vertical_speed     = false,
                        min_displayed_reduction = 10
                    };

                    try
                    {
                        TextReader input = MyAPIGateway.Utilities.ReadFileInLocalStorage(settings_file, typeof(display_settings));
                        stored_settings = MyAPIGateway.Utilities.SerializeFromXML<display_settings>(input.ReadToEnd());
                        input.Close();
                    }
                    catch (Exception error)
                    {
                        MyLog.Default.WriteLine(error);
                    }

                    _HUD_messages[   THRUST_LOSS].toggled_on = stored_settings.show_thrust_reduction;
                    _HUD_messages[VERTICAL_SPEED].toggled_on = stored_settings.show_vertical_speed;
                    _min_displayed_reduction                 = stored_settings.min_displayed_reduction;
                }
                _settings_loaded = true;
            }
        }

        public static void deregister_handlers()
        {
            if (!_UI_handlers_registered)
                return;
            MyAPIGateway.Utilities.MessageEntered -= command_handler;
            _HUD_messages.Clear();
            _UI_handlers_registered = _settings_loaded = false;
        }

        public static void refresh_local_player_info()
        {
            local_player     = MyAPIGateway.Session.LocalHumanPlayer;
            local_controller = local_player?.Controller.ControlledEntity;
        }

        public static void refresh_local_player_HUD()
        {
            if (!_UI_handlers_registered)
                return;

            bool player_in_cockpit = local_controller is MyShipController;

            foreach (var message_entry in _HUD_messages)
            {
                HUD_notification cur_message = message_entry.Value;
                bool show_message = player_in_cockpit && _HUD_messages[message_entry.Key].toggled_on && cur_message.set_to_visible;

                if (cur_message.currently_visible != show_message)
                {
                    if (show_message)
                        cur_message.contents.Show();
                    else
                        cur_message.contents.Hide();
                    cur_message.currently_visible = show_message;
                }
            }
        }
    }
}
