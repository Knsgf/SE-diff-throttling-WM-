using System;
using System.Collections.Generic;
using System.IO;
using System.Text;

using Sandbox.Game.Entities;
using Sandbox.ModAPI;
using VRage.Game;
using VRage.Game.ModAPI;
using VRage.Utils;

namespace ttdtwm
{
    public struct TTDTWMSettings
    {
        public bool ShowThrustReduction, ShowVerticalSpeed;
        public  int MinDisplayedReduction;
    };

    static class screen_info
    {
        const string settings_file = "TTDTWM.CFG";
        private static readonly char[] whitespace_char = { ' ', '\t' };

        const string THRUST_LOSS          = "/tpdt-tl";
        const string VERTICAL_SPEED       = "/tpdt-vs";

        private class HUD_notification
        {
            public IMyHudNotification contents;
            public string             screen_description;
            public bool               toggled_on, currently_visible, set_to_visible;
        }

        private static Dictionary<string, HUD_notification> _HUD_messages             = new Dictionary<string, HUD_notification>();
        private static Dictionary<string,   Action<string>> _parameter_handlers       = new Dictionary<string,   Action<string>>();
        private static bool _UI_handlers_registered = false, _settings_loaded = false;

        private static int _min_displayed_reduction = 10;

        #region Properties

        public static IMyPlayer                                          local_player          { get; private set; }
        public static VRage.Game.ModAPI.Interfaces.IMyControllableEntity local_controller      { get; private set; }
        public static IMyCubeGrid                                        local_controller_grid { get; private set; }

        public static bool settings_loaded => _settings_loaded;

        #endregion

        #region Debug display

        static private void display_info(IMyCubeGrid grid, string message, int display_time_ms, string font)
        {
            if (grid == null || local_controller_grid == grid)
                MyAPIGateway.Utilities.ShowNotification(message, display_time_ms, font);
        }

        static public void screen_text(IMyCubeGrid grid, string method_name, string message, int display_time_ms)
        {
            string grid_name = (grid == null) ? "" : ("\"" + grid.DisplayName + "\"");
            if (method_name == "")
                display_info(grid, string.Format("{0} {1}", grid_name, message), display_time_ms, MyFontEnum.White);
            else
                display_info(grid, string.Format("{0}(): {1} {2}", method_name, grid_name, message), display_time_ms, MyFontEnum.White);
        }

        static private void remote_info(IMyCubeGrid grid, string message, int display_time_ms)
        {
            if (MyAPIGateway.Multiplayer == null || !MyAPIGateway.Multiplayer.IsServer || !sync_helper.network_handlers_registered)
                return;

            if (grid == null)
            {
                message = display_time_ms.ToString() + " " + message;
                sync_helper.send_message_to_others(sync_helper.message_types.REMOTE_SCREEN_TEXT, null, Encoding.UTF8.GetBytes(message), Encoding.UTF8.GetByteCount(message));
            }
            else 
            {
                IMyPlayer recipient = MyAPIGateway.Multiplayer.Players.GetPlayerControllingEntity(grid);
                if (recipient != null)
                {
                    message = display_time_ms.ToString() + " " + message;
                    sync_helper.send_message_to(recipient.SteamUserId, sync_helper.message_types.REMOTE_SCREEN_TEXT, null, Encoding.UTF8.GetBytes(message), Encoding.UTF8.GetByteCount(message));
                }
            }
        }

        public static void show_remote_text(object entity, byte[] message, int length)
        {
            string[] message_parts = Encoding.UTF8.GetString(message, 0, length).Split(whitespace_char, 2);
            screen_text(null, "", message_parts[1], int.Parse(message_parts[0]));
        }

        public static void remote_screen_text(IMyCubeGrid grid, string method_name, string message, int display_time_ms)
        {
            string grid_name = (grid == null) ? "" : ("\"" + grid.DisplayName + "\"");
            if (method_name == "")
                remote_info(grid, string.Format("{0} {1}", grid_name, message), display_time_ms);
            else
                remote_info(grid, string.Format("{0}(): {1} {2}", method_name, grid_name, message), display_time_ms);
        }

        public static void dual_screen_text(IMyCubeGrid grid, string method_name, string message, int display_time_ms)
        {
            if (MyAPIGateway.Multiplayer != null && MyAPIGateway.Multiplayer.IsServer)
                remote_screen_text(grid, method_name, "SVR " + message, display_time_ms);
            else
                screen_text(grid, method_name, "CLI " + message, display_time_ms);
        }

        #endregion

        #region HUD messages

        static private void register_HUD_notification(string message_cmd, string screen_description, bool toggled_on,
            Action<string> parameter_handler = null, string default_text = "", string default_colour = MyFontEnum.White)
        {
            var new_message = new HUD_notification();
            new_message.contents           = MyAPIGateway.Utilities.CreateNotification(default_text, 0, default_colour);
            new_message.currently_visible  = false;
            new_message.toggled_on         = toggled_on;
            new_message.screen_description = screen_description;
            _HUD_messages[message_cmd]     = new_message;
            if (parameter_handler != null)
                _parameter_handlers[message_cmd] = parameter_handler;
        }

        public static void set_displayed_thrust_reduction(IMyCubeGrid grid, uint thrust_reduction, bool is_single_grid_assembly)
        {
            if (grid != local_controller_grid)
                return;

            bool is_visible = is_single_grid_assembly && thrust_reduction > _min_displayed_reduction && _HUD_messages[THRUST_LOSS].toggled_on;

            _HUD_messages[THRUST_LOSS].set_to_visible = is_visible;
            if (is_visible)
            {
                _HUD_messages[THRUST_LOSS].contents.Text = "Thrust loss: " + thrust_reduction.ToString() + " %";
                _HUD_messages[THRUST_LOSS].contents.Font = (thrust_reduction > 30) ? MyFontEnum.Red : MyFontEnum.White;
            }
        }

        public static void set_displayed_vertical_speed(IMyCubeGrid grid, float vertical_speed, bool is_primary_grid)
        {
            if (grid != local_controller_grid)
                return;

            bool is_visible = is_primary_grid && _HUD_messages[VERTICAL_SPEED].toggled_on && Math.Abs(vertical_speed) >= 0.05f;

            _HUD_messages[VERTICAL_SPEED].set_to_visible = is_visible;
            if (is_visible)
                _HUD_messages[VERTICAL_SPEED].contents.Text = string.Format("Vertical speed: {0:F1} m/s", vertical_speed);
        }

        #endregion

        #region Chat command handling

        private static void set_min_thrust_loss_percentage(string parameter)
        {
            int min_thrust_loss; 
            bool min_loss_entered = int.TryParse(parameter, out min_thrust_loss);

            if (!min_loss_entered || min_thrust_loss < 0 || min_thrust_loss > 100)
            {
                MyAPIGateway.Utilities.ShowMessage("TP&DT", "Please specify a number between 0 and 100");
                return;
            }
            _min_displayed_reduction = min_thrust_loss;
            MyAPIGateway.Utilities.ShowMessage("TP&DT", string.Format("Minimum displayed thrust loss is now {0} %", min_thrust_loss));
            _HUD_messages[THRUST_LOSS].toggled_on = true;
        }

        private static void extract_command_and_parameter(string input, out string command, out string parameter)
        {
            string[] split_input = input.Trim().ToLower().Split(whitespace_char, 2);
            command   = split_input[0];
            parameter = (split_input.Length > 1) ? split_input[1] : "";
        }

        private static void command_handler(string message, ref bool dummy)
        {
            string command, parameter;

            if (message != null)
            { 
                extract_command_and_parameter(message, out command, out parameter);
               if (!_HUD_messages.ContainsKey(command))
                    return;
                if (_parameter_handlers.ContainsKey(command) && parameter.Length > 0)
                    _parameter_handlers[command](parameter);
                else
                {
                    _HUD_messages[command].toggled_on = !_HUD_messages[command].toggled_on;
                    MyAPIGateway.Utilities.ShowMessage("TP&DT", string.Format("{0} is now {1}", _HUD_messages[command].screen_description, _HUD_messages[command].toggled_on ? "visible" : "hidden"));
                }
            }

            try
            {
                TTDTWMSettings stored_settings = new TTDTWMSettings
                {
                    ShowThrustReduction   = _HUD_messages[   THRUST_LOSS].toggled_on,
                    ShowVerticalSpeed     = _HUD_messages[VERTICAL_SPEED].toggled_on,
                    MinDisplayedReduction = _min_displayed_reduction
                };
                TextWriter output = MyAPIGateway.Utilities.WriteFileInLocalStorage(settings_file, typeof(TTDTWMSettings));
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
                register_HUD_notification(   THRUST_LOSS, "Thrust loss indication", false, set_min_thrust_loss_percentage);
                register_HUD_notification(VERTICAL_SPEED, "Vertical speed readout",  true);
                _UI_handlers_registered = true;
            }

            if (!_settings_loaded && _UI_handlers_registered)
            {
                if (MyAPIGateway.Utilities.FileExistsInLocalStorage(settings_file, typeof(TTDTWMSettings)))
                {
                    TTDTWMSettings stored_settings = new TTDTWMSettings
                    {
                        ShowThrustReduction   = true,
                        ShowVerticalSpeed     = false,
                        MinDisplayedReduction = 10
                    };

                    try
                    {
                        TextReader input = MyAPIGateway.Utilities.ReadFileInLocalStorage(settings_file, typeof(TTDTWMSettings));
                        stored_settings  = MyAPIGateway.Utilities.SerializeFromXML<TTDTWMSettings>(input.ReadToEnd());
                        input.Close();
                    }
                    catch (Exception error)
                    {
                        MyLog.Default.WriteLine(error);
                    }

                    _HUD_messages[   THRUST_LOSS].toggled_on = stored_settings.ShowThrustReduction;
                    _HUD_messages[VERTICAL_SPEED].toggled_on = stored_settings.ShowVerticalSpeed;
                    _min_displayed_reduction                 = stored_settings.MinDisplayedReduction;
                }
                command_handler(null, ref _settings_loaded);
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
            local_player          = MyAPIGateway.Session.LocalHumanPlayer;
            local_controller      = local_player?.Controller.ControlledEntity;
            var ship_controller   = local_controller as MyShipController;
            local_controller_grid = ship_controller?.CubeGrid;
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
