﻿using System;
using System.Collections.Generic;
using System.IO;
using System.Text;

using Sandbox.Game.Entities;
using Sandbox.ModAPI;
using VRage.Game;
using VRage.Game.ModAPI;
using VRage.Utils;

namespace orbiter_SE
{
    public struct OSESettings
    {
        public bool AllowScriptsToInspectOrbitOfAnyShip, DisableTorque, ShowThrustReduction, ShowVerticalSpeed;
        public  int MinDisplayedReduction;
    };

    static class screen_info
    {
        const string settings_file = "OSE.CFG";
        private static readonly char[] whitespace_char = { ' ', '\t' };

        const string THRUST_LOSS          = "/tpdt-tl";
        const string VERTICAL_SPEED       = "/tpdt-vs";
        const string ORBITAL_ELEMENTS     = "/ose-oi";
        const string ORBIT_REFERENCE      = "/ose-or";
        const string PLANE_ALIGNMENT      = "/ose-oa";

        private class HUD_notification
        {
            public IMyHudNotification contents;
            public string             screen_description;
            public bool               toggled_on, /*currently_visible,*/ set_to_visible;
        }

        private static Dictionary<string, HUD_notification> _HUD_messages             = new Dictionary<string, HUD_notification>();
        private static Dictionary<string,   Action<string>> _parameter_handlers       = new Dictionary<string,   Action<string>>();
        private static HashSet   <string>                   _requires_natural_gravity = new HashSet   <string>();
        private static bool _UI_handlers_registered = false, _local_settings_loaded = false, _server_settings_loaded = false;

        private static int           _min_displayed_reduction = 10;
        private static bool          _display_local_gravity = false, _display_orbit_energy = false, _display_apside_info = false, _display_inclination = false;
        private static StringBuilder _orbital_elements_text = new StringBuilder();

        static byte[] _message = new byte[9];

        private static readonly Dictionary<string, string> _client_messages = new Dictionary<string, string>(), _server_messages = new Dictionary<string, string>();
        private static readonly Dictionary<string, int> _remote_message_times = new Dictionary<string, int>();

        #region Properties

        public static IMyPlayer                                          local_player          { get; private set; }
        public static VRage.Game.ModAPI.Interfaces.IMyControllableEntity local_controller      { get; private set; }
        public static IMyCubeGrid                                        local_controller_grid { get; private set; }

        public static bool scripts_can_inspect_orbit_of_any_ship { get; private set; } = true;
        public static bool torque_disabled { get; private set; } = false;
        public static bool settings_loaded => _local_settings_loaded && _server_settings_loaded;

        #endregion

        #region Debug display

        private static void log_screen_info_action(string method_name, string message)
        {
            MyLog.Default.WriteLine(string.Format("TTDTWM\tscreen_info.{0}(): {1}", method_name, message));
        }

        static private void display_info(IMyCubeGrid grid, string message, int display_time_ms, string font)
        {
            try
            {
                if (local_player != null && (grid == null || local_controller_grid == grid))
                    MyAPIGateway.Utilities.ShowNotification(message, display_time_ms, font);
            }
            catch (Exception e)
            {
                MyLog.Default.WriteLine(e);
            }
        }

        static public void screen_text(IMyCubeGrid grid, string method_name, string message, int display_time_ms)
        {
            string grid_name = (grid == null) ? "" : ("\"" + grid.DisplayName + "\"");
            if (method_name == "")
                display_info(grid, string.Format("{0} {1}", grid_name, message), display_time_ms, MyFontEnum.White);
            else
                display_info(grid, string.Format("{0}(): {1} {2}", method_name, grid_name, message), display_time_ms, MyFontEnum.White);
        }

        static private void remote_info(IMyCubeGrid grid, string message_id, string message, int display_time_ms)
        {
            if (!sync_helper.network_handlers_registered || !sync_helper.running_on_server)
                return;

            if (grid == null)
            {
                message = display_time_ms.ToString() + " " + message_id + " " + message;
                sync_helper.send_message_to_others(sync_helper.message_types.REMOTE_SCREEN_TEXT, null, Encoding.UTF8.GetBytes(message), Encoding.UTF8.GetByteCount(message));
                if (local_player != null)
                    show_remote_text(sync_helper.message_types.REMOTE_SCREEN_TEXT, null, Encoding.UTF8.GetBytes(message), Encoding.UTF8.GetByteCount(message));
            }
            else 
            {
                IMyPlayer recipient = MyAPIGateway.Multiplayer.Players.GetPlayerControllingEntity(grid);
                if (recipient != null)
                {
                    message = display_time_ms.ToString() + " " + message_id + " " + message;
                    if (local_player == recipient)
                        show_remote_text(sync_helper.message_types.REMOTE_SCREEN_TEXT, null, Encoding.UTF8.GetBytes(message), Encoding.UTF8.GetByteCount(message));
                    else
                        sync_helper.send_message_to(recipient.SteamUserId, sync_helper.message_types.REMOTE_SCREEN_TEXT, null, Encoding.UTF8.GetBytes(message), Encoding.UTF8.GetByteCount(message));
                }
            }
        }

        public static void show_remote_text(sync_helper.message_types message_type, object entity, byte[] message, int length)
        {
            string[] message_parts = Encoding.UTF8.GetString(message, 0, length).Split(whitespace_char, 3);
            string message_id = message_parts[1];
            _remote_message_times[message_id] = int.Parse(message_parts[0]);
            _server_messages     [message_id] = message_parts[2];

            List<string> messages_to_remove = null;
            foreach (string cur_message in _remote_message_times.Keys)
            {
                if (_server_messages.ContainsKey(cur_message) && _client_messages.ContainsKey(cur_message))
                {
                    string output = _server_messages[cur_message];

                    if (_client_messages[cur_message] != "")
                        output += "\n" + _client_messages[cur_message];
                    screen_text(null, "", output, _remote_message_times[cur_message]);
                    if (messages_to_remove == null)
                        messages_to_remove = new List<string>();
                    messages_to_remove.Add(cur_message);
                }
            }
            if (messages_to_remove != null)
            {
                foreach (string cur_message in messages_to_remove)
                {
                    _remote_message_times.Remove(cur_message);
                    _server_messages.Remove(cur_message);
                    _client_messages.Remove(cur_message);
                }
            }
        }

        public static void dual_screen_text(IMyCubeGrid grid, string message_id, string method_name, string message, int display_time_ms)
        {
            string combined_id = message_id, message_prefix = (method_name != "") ? (method_name + "(): ") : "";

            if (grid != null)
            {
                combined_id    += grid.EntityId.ToString();
                message_prefix += "\"" + grid.DisplayName + "\" ";
            }
            if (local_player != null && !_client_messages.ContainsKey(combined_id))
                _client_messages[combined_id] = message_prefix + "<C> " + message;
            if (sync_helper.running_on_server)
                remote_info(grid, combined_id, message_prefix + "<S> " + message, display_time_ms);
        }

        #endregion

        #region HUD messages

        static private void register_HUD_notification(string message_cmd, string screen_description, bool toggled_on, bool requires_natural_gravity,
            Action<string> parameter_handler = null, string default_text = "", string default_colour = MyFontEnum.White)
        {
            var new_message = new HUD_notification();
            new_message.contents           = MyAPIGateway.Utilities.CreateNotification(default_text, 0, default_colour);
            //new_message.currently_visible  = false;
            new_message.toggled_on         = toggled_on;
            new_message.screen_description = screen_description;
            _HUD_messages[message_cmd]     = new_message;
            if (parameter_handler != null)
                _parameter_handlers[message_cmd] = parameter_handler;
            if (requires_natural_gravity)
                _requires_natural_gravity.Add(message_cmd);
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

        public static void set_displayed_orbital_elements(IMyCubeGrid grid, Func<orbit_elements> element_getter)
        {
            if (grid != local_controller_grid)
                return;

            bool is_visible = gravity_and_physics.world_has_gravity && _HUD_messages[ORBITAL_ELEMENTS].toggled_on;

            _HUD_messages[ORBITAL_ELEMENTS].set_to_visible = is_visible;
            if (is_visible)
            {
                orbit_elements elements = element_getter();

                _orbital_elements_text.Clear();
                if (elements == null)
                    _orbital_elements_text.Append("Orbit information unavaiable");
                else
                {
                    _orbital_elements_text.AppendFormat("Ref: {0}; R = {1:F1} km", elements.name, elements.predicted_distance / 1000.0);
                    if (_display_local_gravity)
                    {
                        _orbital_elements_text.AppendFormat("\n(1) g = {0:F2} m/s^2; v1 = {1:F0} m/s; v2 = {2:F0} m/s",
                            elements.local_gravity.Length(), elements.circular_speed, elements.escape_speed);
                    }
                    if (_display_apside_info)
                    {
                        char  approaching_apside_name;
                        float time_to_apside;

                        if (elements.eccentricity > 1.0)
                        {
                            approaching_apside_name = 'P';
                            time_to_apside          = (float) (-elements.time_from_periapsis);
                        }
                        else if (elements.time_from_periapsis > elements.orbit_period / 2.0)
                        {
                            approaching_apside_name = 'P';
                            time_to_apside          = (float) (elements.orbit_period - elements.time_from_periapsis);
                        }
                        else
                        {
                            approaching_apside_name = 'A';
                            time_to_apside          = (float) (elements.orbit_period / 2.0 - elements.time_from_periapsis);
                        }

                        _orbital_elements_text.AppendFormat("\n(2) PR = {0:F1} km; AR = {1:F1} km; Tt{2} = {3:F0} s",
                            elements.periapsis_radius / 1000.0, elements.apoapsis_radius / 1000.0, approaching_apside_name, time_to_apside);
                    }
                    if (_display_orbit_energy)
                    {
                        _orbital_elements_text.AppendFormat("\n(3) SMA = {0:F1} km; e = {1:F3}; T = {2:F0} s",
                            elements.semi_major_axis / 1000.0, elements.eccentricity, elements.orbit_period);
                    }
                    if (_display_inclination)
                    {
                        _orbital_elements_text.AppendFormat("\n(4) i = {0:F0} deg; LAN = {1:F0} deg; TL = {2:F0} deg",
                            elements.inclination * 180.0 / Math.PI, elements.longitude_of_ascending_node * 180.0 / Math.PI,
                            ((elements.longitude_of_ascending_node + elements.argument_of_periapsis + elements.true_anomaly) * 180.0 / Math.PI) % 360);
                    }
                }

                _HUD_messages[ORBITAL_ELEMENTS].contents.Text = _orbital_elements_text.ToString();
                _HUD_messages[ORBITAL_ELEMENTS].contents.Font = elements.foreign_reference ? MyFontEnum.DarkBlue : MyFontEnum.White;
            }
        }

        public static void set_displayed_target_plane(IMyCubeGrid grid, Func<orbit_plane_intersection> plane_getter)
        {
            if (grid != local_controller_grid)
                return;

            bool is_visible = gravity_and_physics.world_has_gravity && _HUD_messages[PLANE_ALIGNMENT].toggled_on;

            _HUD_messages[PLANE_ALIGNMENT].set_to_visible = is_visible;
            if (is_visible)
            {
                orbit_plane_intersection target_plane = plane_getter();

                if (target_plane == null)
                {
                    _HUD_messages[PLANE_ALIGNMENT].contents.Text = "Orbit information unavailable";
                    _HUD_messages[PLANE_ALIGNMENT].contents.Font = MyFontEnum.White;
                }
                else if (target_plane.target_normal.LengthSquared() < 0.25)
                {
                    _HUD_messages[PLANE_ALIGNMENT].contents.Text = "Target plane not set";
                    _HUD_messages[PLANE_ALIGNMENT].contents.Font = MyFontEnum.Red;
                }
                else
                {
                    char   node_letter;
                    double time_to_closest_node;

                    if (target_plane.time_to_ascending_node < target_plane.time_to_descending_node)
                    {
                        node_letter          = 'A';
                        time_to_closest_node = target_plane.time_to_ascending_node;
                    }
                    else
                    {
                        node_letter          = 'D';
                        time_to_closest_node = target_plane.time_to_descending_node;
                    }
                    _HUD_messages[PLANE_ALIGNMENT].contents.Text = string.Format("Rel. inc.: {0:F1} deg; time to {1}N: {2:F0}", 
                        target_plane.relative_inclination * 180.0 / Math.PI, node_letter, time_to_closest_node);
                    _HUD_messages[PLANE_ALIGNMENT].contents.Font = MyFontEnum.White;
                }
            }
        }

        #endregion

        #region Chat command handling

        private static void set_min_thrust_loss_percentage(string parameter)
        {
            int min_thrust_loss; 
            bool min_loss_entered = int.TryParse(parameter, out min_thrust_loss);

            if (!min_loss_entered || min_thrust_loss < 0 || min_thrust_loss > 100)
            {
                MyAPIGateway.Utilities.ShowMessage("OSE", "Please specify a number between 0 and 100");
                return;
            }
            _min_displayed_reduction = min_thrust_loss;
            MyAPIGateway.Utilities.ShowMessage("OSE", string.Format("Minimum displayed thrust loss is now {0} %", min_thrust_loss));
            _HUD_messages[THRUST_LOSS].toggled_on = true;
        }

        private static void set_displayed_orbital_elements(string parameter)
        {
            uint selection;
            bool selection_entered = uint.TryParse(parameter, out selection), parameter_visible = false;
            string changed_parameter = "";

            if (!selection_entered || selection == 0 || selection > 4)
            {
                MyAPIGateway.Utilities.ShowMessage("OSE", "Please specify a number between 1 and 4");
                return;
            }
            switch (selection)
            {
                case 1:
                    parameter_visible = _display_local_gravity = !_display_local_gravity;
                    changed_parameter = "Local gravity, circular and escape speeds";
                    break;

                case 2:
                    parameter_visible = _display_apside_info = !_display_apside_info;
                    changed_parameter = "Apside radii and approach timer";
                    break;

                case 3:
                    parameter_visible = _display_orbit_energy = !_display_orbit_energy;
                    changed_parameter = "Semi-major axis, eccentricity and period";
                    break;

                case 4:
                    parameter_visible = _display_inclination = !_display_inclination;
                    changed_parameter = "Inclination, longitude of ascending node and true longitude";
                    break;
            }
            MyAPIGateway.Utilities.ShowMessage("OSE", changed_parameter + " are now " + (parameter_visible ? "visisble" : "hidden"));
            _HUD_messages[ORBITAL_ELEMENTS].toggled_on = true;
        }

        private static void set_target_plane(string paramater)
        {
            string[] split_input = paramater.Split(whitespace_char, 2);
            double   inclination;
            if (!double.TryParse(split_input[0], out inclination))
            {
                MyAPIGateway.Utilities.ShowMessage("OSE", "Please specify inclination and LAN in degrees, separated by space");
                return;
            }
            if (inclination < 0.0 || inclination > 180.0)
            {
                MyAPIGateway.Utilities.ShowMessage("OSE", "Inclination must be between 0 and 180");
                return;
            }

            double LAN = 0.0;
            if (inclination != 0.0 && (split_input.Length < 2 || !double.TryParse(split_input[1], out LAN)))
            {
                MyAPIGateway.Utilities.ShowMessage("OSE", "Please specify inclination and LAN in degrees, separated by space");
                return;
            }

            if (inclination == 0.0)
            {
                gravity_and_physics.set_target_plane(local_controller_grid, 0.0, 0.0);
                MyAPIGateway.Utilities.ShowMessage("OSE", "Target plane set with inc. = 0 deg, LAN = 0 deg");
            }
            else
            {
                if (LAN < 0.0 || LAN > 360.0)
                {
                    MyAPIGateway.Utilities.ShowMessage("OSE", "LAN must be between 0 and 360");
                    return;
                }

                gravity_and_physics.set_target_plane(local_controller_grid, inclination * Math.PI / 180.0, LAN * Math.PI / 180.0);
                MyAPIGateway.Utilities.ShowMessage("OSE", string.Format("Target plane set with inc. = {0} deg, LAN = {1} deg", inclination, LAN));
            }
            _HUD_messages[PLANE_ALIGNMENT].toggled_on = true;
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
                if (command == ORBIT_REFERENCE)
                {
                    string new_reference = gravity_and_physics.set_current_reference(local_controller_grid, parameter);
                    if (new_reference != null)
                        MyAPIGateway.Utilities.ShowMessage("OSE", "Changed reference body to " + new_reference);
                    return;
                }
                if (!_HUD_messages.ContainsKey(command))
                    return;
                if (_parameter_handlers.ContainsKey(command) && parameter.Length > 0)
                    _parameter_handlers[command](parameter);
                else if (!_requires_natural_gravity.Contains(command) || gravity_and_physics.world_has_gravity)
                {
                    _HUD_messages[command].toggled_on = !_HUD_messages[command].toggled_on;
                    MyAPIGateway.Utilities.ShowMessage("OSE", string.Format("{0} is now {1}", _HUD_messages[command].screen_description, _HUD_messages[command].toggled_on ? "visible" : "hidden"));
                    if (command == PLANE_ALIGNMENT && !_HUD_messages[PLANE_ALIGNMENT].toggled_on)
                        gravity_and_physics.clear_target_plane(local_controller_grid);
                }
                else
                    MyAPIGateway.Utilities.ShowMessage("OSE", string.Format("{0} cannot be toggled in zero-g worlds", _HUD_messages[command].screen_description));
            }

            try
            {
                OSESettings stored_settings = new OSESettings
                {
                    AllowScriptsToInspectOrbitOfAnyShip = scripts_can_inspect_orbit_of_any_ship,
                    DisableTorque                       = torque_disabled,
                    ShowThrustReduction                 = _HUD_messages[   THRUST_LOSS].toggled_on,
                    ShowVerticalSpeed                   = _HUD_messages[VERTICAL_SPEED].toggled_on,
                    MinDisplayedReduction               = _min_displayed_reduction
                };
                TextWriter output = MyAPIGateway.Utilities.WriteFileInLocalStorage(settings_file, typeof(OSESettings));
                output.Write(MyAPIGateway.Utilities.SerializeToXML(stored_settings));
                output.Close();
            }
            catch (Exception error)
            {
                MyLog.Default.WriteLine(error);
            }
        }

        #endregion

        public static void handle_remote_settings(sync_helper.message_types message_type, object entity, byte[] message, int length)
        {
            if (message[0] == 0 && length == 9 && sync_helper.running_on_server)
            {
                message[0] = 1;
                if (scripts_can_inspect_orbit_of_any_ship)
                    message[0] |= 0x2;
                if (torque_disabled)
                    message[0] |= 0x4;
                ulong recipient = sync_helper.decode_unsigned(8, message, 1);
                sync_helper.send_message_to(recipient, sync_helper.message_types.GLOBAL_MODES, null, message, 1);
            }
            else if (message[0] > 0 && length == 1 && !sync_helper.running_on_server)
            {
                scripts_can_inspect_orbit_of_any_ship = (message[0] & 0x2) != 0;
                torque_disabled                       = (message[0] & 0x4) != 0;
                _server_settings_loaded               = true;
            }
        }

        public static void try_register_handlers()
        {
            if (!_UI_handlers_registered && MyAPIGateway.Utilities != null)
            {
                MyAPIGateway.Utilities.MessageEntered += command_handler;
                register_HUD_notification(     THRUST_LOSS, "Thrust loss indication", false, false, set_min_thrust_loss_percentage);
                register_HUD_notification(  VERTICAL_SPEED, "Vertical speed readout",  true, false);
                register_HUD_notification(ORBITAL_ELEMENTS,      "Orbit information", false, false, set_displayed_orbital_elements);
                register_HUD_notification( PLANE_ALIGNMENT,    "Plane alignment aid", false,  true, set_target_plane);
                _UI_handlers_registered = true;
            }

            if (!_local_settings_loaded && _UI_handlers_registered && sync_helper.network_handlers_registered && (local_player != null || sync_helper.running_on_server))
            {
                if (MyAPIGateway.Utilities.FileExistsInLocalStorage(settings_file, typeof(OSESettings)))
                {
                    OSESettings stored_settings = new OSESettings
                    {
                        AllowScriptsToInspectOrbitOfAnyShip = true,
                        DisableTorque                       = false,
                        ShowThrustReduction                 = true,
                        ShowVerticalSpeed                   = false,
                        MinDisplayedReduction               = 10
                    };

                    try
                    {
                        TextReader input = MyAPIGateway.Utilities.ReadFileInLocalStorage(settings_file, typeof(OSESettings));
                        stored_settings = MyAPIGateway.Utilities.SerializeFromXML<OSESettings>(input.ReadToEnd());
                        input.Close();
                    }
                    catch (Exception error)
                    {
                        MyLog.Default.WriteLine(error);
                    }

                    _HUD_messages[   THRUST_LOSS].toggled_on = stored_settings.ShowThrustReduction;
                    _HUD_messages[VERTICAL_SPEED].toggled_on = stored_settings.ShowVerticalSpeed;
                    _min_displayed_reduction                 = stored_settings.MinDisplayedReduction;
                    if (sync_helper.running_on_server)
                    {
                        scripts_can_inspect_orbit_of_any_ship = stored_settings.AllowScriptsToInspectOrbitOfAnyShip;
                        torque_disabled                       = stored_settings.DisableTorque;
                    }
                    else
                    {
                        _message[0] = 0;
                        sync_helper.encode_unsigned(local_player.SteamUserId, 8, _message, 1);
                        sync_helper.send_message_to_server(sync_helper.message_types.GLOBAL_MODES, null, _message, 9);
                    }
                }
                command_handler(null, ref _local_settings_loaded);
                _local_settings_loaded = true;
                if (sync_helper.running_on_server)
                    _server_settings_loaded = true;
            }
        }

        public static void deregister_handlers()
        {
            if (!_UI_handlers_registered)
                return;
            MyAPIGateway.Utilities.MessageEntered -= command_handler;
            _HUD_messages.Clear();
            _UI_handlers_registered = _local_settings_loaded = _server_settings_loaded = false;
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

                IMyHudNotification contents = cur_message.contents;
                /*
                if (cur_message.currently_visible != show_message)
                {
                    if (show_message)
                        contents.Show();
                    else
                        contents.Hide();
                    cur_message.currently_visible = show_message;
                }
                */
                contents.Hide();
                if (show_message)
                    contents.Show();
            }
        }
    }
}
