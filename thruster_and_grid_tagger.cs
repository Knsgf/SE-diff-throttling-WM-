using System;
using System.Collections.Generic;
using System.Text;

using Sandbox.Game.Entities;
using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI;
using VRage.Game.ModAPI;
using VRage.ModAPI;
using VRage.Utils;
using VRageMath;

namespace ttdtwm
{
    static class thruster_and_grid_tagger
    {
        const int STEERING = 0x1, THRUST_TRIM = 0x2, THRUST_LIMIT = 0x4, LINEAR_OFF = 0x8;
        const int COT_MODE = 0x1, INDIVIDUAL_CALIBRATION = 0x2, TOUCHDOWN_MODE = 0x4, ROTATIONAL_DAMNPING_OFF = 0x8;
        const int IDO_FORE_AFT = 0x10, IDO_PORT_STARBOARD = 0x20, IDO_DORSAL_VENTRAL = 0x40;

        const float MANUAL_THROTTLE_SCALE = 9990.0f;

        const int THRUSTER_MODE_FIELD_START   = 0;
        const int THRUSTER_MODE_FIELD_END     = THRUSTER_MODE_FIELD_START   + 1;
        const int MANUAL_THROTTLE_FIELD_START = THRUSTER_MODE_FIELD_END     + 1;
        const int MANUAL_THROTTLE_FIELD_END   = MANUAL_THROTTLE_FIELD_START + 3;
        const int THRUSTER_FIELDS_LENGTH      = MANUAL_THROTTLE_FIELD_END   + 1;

        const float GRID_CONTROL_SENSITIVITY_SCALE = 1000.0f, MIN_CONTROL_SENSITIVITY = 0.1f;
        
        const int GRID_MODE_FIELD_START                = 0;
        const int GRID_MODE_FIELD_END                  = GRID_MODE_FIELD_START                + 2;
        const int GRID_MANOEUVRE_FIELD_START           = GRID_MODE_FIELD_END                  + 1;
        const int GRID_MANOEUVRE_FIELD_END             = GRID_MANOEUVRE_FIELD_START           + 0;
        const int GRID_CONTROL_SENSITIVITY_FIELD_START = GRID_MANOEUVRE_FIELD_END             + 1;
        const int GRID_CONTROL_SENSITIVITY_FIELD_END   = GRID_CONTROL_SENSITIVITY_FIELD_START + 5;
        const int GRID_FIELDS_LENGTH                   = GRID_CONTROL_SENSITIVITY_FIELD_END   + 1;

        private static readonly Guid _uuid = new Guid("A78B4F70-BC21-4BAF-A403-855F82C2856F");

        private static HashSet<IMyTerminalBlock> _pending_thruster_requests = null;
        private static HashSet<IMyCubeGrid     > _pending_grid_requests     = null;
        private static bool _have_pending_requests = false;

        private static readonly Dictionary<IMyCubeGrid,    grid_logic> _grid_handlers = new Dictionary<IMyCubeGrid,    grid_logic>();
        private static readonly Dictionary<IMyCubeGrid, StringBuilder> _grid_settings = new Dictionary<IMyCubeGrid, StringBuilder>();

        private static readonly Dictionary<IMyTerminalBlock, engine_control_unit> _thruster_hosts    = new Dictionary<IMyTerminalBlock, engine_control_unit>();
        private static readonly Dictionary<IMyTerminalBlock,       StringBuilder> _thruster_settings = new Dictionary<IMyTerminalBlock,       StringBuilder>();

        private static StringBuilder _current_thruster_settings = null, _current_grid_settings = null;
        
        private static IMyTerminalBlock _current_thruster = null, _last_controller = null;
        private static bool             _current_active_control_on, _current_anti_slip_on, _current_disable_linear, _current_thrust_limiter_on;
        private static int              _thruster_switches, _manual_throttle;

        private static IMyCubeGrid _current_grid = null;
        private static int         _grid_switches, _grid_control_sensitivity;
        private static bool        _CoT_mode_on, _individual_calibration_on, _touchdown_mode_on, _rotational_damping_on;

        private static readonly byte[] _message = new byte[8];

        public static int displayed_thrust_limit { get; set; }

        #region Auxiliaries

        private static void log_tagger_action(string method_name, string message)
        {
            MyLog.Default.WriteLine(string.Format("TTDTWM\tthruster_and_grid_tagger.{0}(): {1}", method_name, message));
        }

        private static int parse_number(string input, int beginning, int end)
        {
            int result = 0;
            char digit;

            for (int index = beginning; index <= end; ++index)
            {
                digit = input[index];
                if (digit < '0' || digit > '9')
                    return result;
                result = result * 10 + (digit - '0');
            }
            return result;
        }

        private static int parse_number(StringBuilder input, int beginning, int end)
        {
            int result = 0;
            char digit;

            for (int index = beginning; index <= end; ++index)
            {
                digit = input[index];
                if (digit < '0' || digit > '9')
                    return result;
                result = result * 10 + (digit - '0');
            }
            return result;
        }

        private static void store_number(StringBuilder input, int number, int beginning, int end)
        {
            char digit;
            int divider = 1;

            for (int index = beginning; index < end; ++index)
                divider *= 10;
            number %= 10 * divider;
            
            for (int index = beginning; index <= end; ++index)
            {
                digit        = (char) (number / divider + '0');
                input[index] = digit;
                number      %= divider;
                divider     /= 10;
            }
        }

        private static void toggle_switch(ref int switches, int switch_mask, bool new_state_on)
        {
            if (new_state_on)
                switches |= switch_mask;
            else
                switches &= ~switch_mask;
        }

        private static void set_switch(IMyEntity entity, StringBuilder stored_settings, ref int switches, sync_helper.message_types sync_message, 
            int switch_mask, bool new_state_on, int beginning, int end)
        {
            toggle_switch(ref switches, switch_mask, new_state_on);
            store_number(stored_settings, switches, beginning, end);
            entity.Storage[_uuid] = stored_settings.ToString();
            
            _message[0] = (byte) switches;
            sync_helper.send_message_to_others(sync_message, entity, _message, 1);
        }

        #endregion

        #region Thrusters' settings handlers

        private static bool update_thruster_flags(IMyTerminalBlock thruster, bool use_remote_switches = false, bool use_remote_manual_throttle = false)
        {
            if (!(thruster is IMyThrust) || !_thruster_hosts.ContainsKey(thruster))
            {
                _current_thruster = null;
                _current_thruster_settings = null;
                _current_active_control_on = _current_anti_slip_on = _current_disable_linear = _current_thrust_limiter_on = false;
                _manual_throttle = 0;
                return false;
            }
            if (thruster == _current_thruster && !use_remote_switches && !use_remote_manual_throttle)
                return true;

            _current_thruster          = thruster;
            _current_thruster_settings = _thruster_settings[thruster];
            thruster.RefreshCustomInfo();
            if (!use_remote_switches)
                _thruster_switches = parse_number(_current_thruster_settings, THRUSTER_MODE_FIELD_START, THRUSTER_MODE_FIELD_END);
            else
            {
                store_number(_current_thruster_settings, _thruster_switches, THRUSTER_MODE_FIELD_START, THRUSTER_MODE_FIELD_END);
                thruster.Storage[_uuid] = _current_thruster_settings.ToString();
            }
            if (!use_remote_manual_throttle)
                _manual_throttle = parse_number(_current_thruster_settings, MANUAL_THROTTLE_FIELD_START, MANUAL_THROTTLE_FIELD_END);
            else
            {
                store_number(_current_thruster_settings, _manual_throttle, MANUAL_THROTTLE_FIELD_START, MANUAL_THROTTLE_FIELD_END);
                thruster.Storage[_uuid] = _current_thruster_settings.ToString();
            }
            _current_active_control_on = (_thruster_switches &     STEERING) != 0;
            _current_anti_slip_on      = (_thruster_switches &  THRUST_TRIM) != 0;
            _current_thrust_limiter_on = (_thruster_switches & THRUST_LIMIT) != 0;
            _current_disable_linear    = (_thruster_switches &   LINEAR_OFF) != 0;

            if (use_remote_switches)
            {
                engine_control_unit host = _thruster_hosts[thruster];
                long thruster_entity_id  = thruster.EntityId;
                host.set_thruster_steering    (thruster_entity_id, _current_active_control_on);
                host.set_thruster_trim        (thruster_entity_id, _current_anti_slip_on     );
                host.set_thruster_limiter     (thruster_entity_id, _current_thrust_limiter_on);
                host.set_thruster_linear_input(thruster_entity_id, _current_disable_linear   );
            }
            if (use_remote_manual_throttle)
                _thruster_hosts[thruster].set_manual_throttle(thruster.EntityId, _manual_throttle / MANUAL_THROTTLE_SCALE);

            return true;
        }

        public static void remote_thrust_settings(sync_helper.message_types message_type, object entity, byte[] argument, int length)
        {
            var thruster = entity as IMyTerminalBlock;
            if (thruster == null)
                return;

            if (message_type == sync_helper.message_types.MANUAL_THROTTLE && length == 2)
            {
                _manual_throttle = (int) sync_helper.decode_signed(2, argument, 0);
                //log_tagger_action("remote_thrust_settings", $"received \"{thruster.CustomName}\" T={_manual_throttle / MANUAL_THROTTLE_SCALE * 100.0f}");
                update_thruster_flags(thruster, use_remote_manual_throttle: true);
            }
            else if (message_type == sync_helper.message_types.THRUSTER_MODES)
            {
                if (length == 1)
                {
                    _thruster_switches = argument[0];
                    //log_tagger_action("remote_thrust_settings", $"received \"{thruster.CustomName}\" S={_thruster_switches:X}h");
                    update_thruster_flags(thruster, use_remote_switches: true);
                }
                else if (length == 8 && sync_helper.running_on_server)
                {
                    update_thruster_flags(thruster);
                    _message[0] = (byte) _thruster_switches;
                    sync_helper.encode_signed(_manual_throttle, 2, _message, 1);
                    ulong recipient = sync_helper.decode_unsigned(8, argument, 0);
                    //log_tagger_action("remote_thrust_settings", $"sending \"{thruster.CustomName}\" S={_thruster_switches:X}h+T={_manual_throttle / MANUAL_THROTTLE_SCALE * 100.0f} to {recipient}");
                    sync_helper.send_message_to(recipient, sync_helper.message_types.THRUSTER_MODES, thruster, _message, 3);
                }
                else if (length == 3 && !sync_helper.running_on_server)
                {
                    _pending_thruster_requests.Remove(thruster);
                    _have_pending_requests = _pending_thruster_requests != null && _pending_thruster_requests.Count > 0
                                          || _pending_grid_requests     != null && _pending_grid_requests.Count     > 0;

                    _thruster_switches = argument[0];
                    _manual_throttle   = (int) sync_helper.decode_signed(2, argument, 1);
                    //log_tagger_action("remote_thrust_settings", $"received \"{thruster.CustomName}\" S={_thruster_switches:X}h+T={_manual_throttle / MANUAL_THROTTLE_SCALE * 100.0f}");
                    update_thruster_flags(thruster, use_remote_switches: true, use_remote_manual_throttle: true);
                }
            }
        }

        public static void attach_ECU(IMyTerminalBlock thruster, engine_control_unit ECU)
        {
            sync_helper.register_entity(sync_helper.message_types.THRUSTER_MODES , thruster, thruster.EntityId);
            sync_helper.register_entity(sync_helper.message_types.MANUAL_THROTTLE, thruster, thruster.EntityId);
            _thruster_hosts   [thruster] = ECU;
            _thruster_settings[thruster] = new StringBuilder(new string('0', THRUSTER_FIELDS_LENGTH));
                
            if (thruster.Storage == null)
                thruster.Storage = new MyModStorageComponent();

            if (!sync_helper.running_on_server)
            {
                //log_tagger_action("attach_ECU", $"queueing \"{thruster.CustomName}\" state for player {screen_info.local_player.SteamUserId}");
                if (_pending_thruster_requests == null)
                    _pending_thruster_requests = new HashSet<IMyTerminalBlock>();
                _pending_thruster_requests.Add(thruster);
                _have_pending_requests = true;
                return;
            }

            string thruster_data;
            if (!thruster.Storage.TryGetValue(_uuid, out thruster_data) || thruster_data.Length < THRUSTER_FIELDS_LENGTH)
                _thruster_switches = _manual_throttle = 0;
            else
            {
                _thruster_switches = parse_number(thruster_data,   THRUSTER_MODE_FIELD_START,   THRUSTER_MODE_FIELD_END);
                _manual_throttle   = parse_number(thruster_data, MANUAL_THROTTLE_FIELD_START, MANUAL_THROTTLE_FIELD_END);
            }
            update_thruster_flags(thruster, use_remote_switches: true, use_remote_manual_throttle: true);
        }

        public static void detach_ECU(IMyTerminalBlock thruster)
        {
            sync_helper.deregister_entity(sync_helper.message_types.THRUSTER_MODES , thruster.EntityId);
            sync_helper.deregister_entity(sync_helper.message_types.MANUAL_THROTTLE, thruster.EntityId);
            _thruster_hosts.Remove(thruster);
            _thruster_settings.Remove(thruster);
        }

        public static float get_thrust_limit(IMyTerminalBlock thruster)
        {
            engine_control_unit host;

            if (!_thruster_hosts.TryGetValue(thruster, out host))
                return -2;
            return host.extract_thrust_limit(thruster.EntityId);
        }

        public static void show_thrust_limit(IMyTerminalBlock thruster, StringBuilder info_text)
        {
            int displayed_thrust_limit = (int) get_thrust_limit(thruster);
            if (displayed_thrust_limit >= -1)
            {
                if (info_text.Length > 0)
                    info_text.AppendLine();
                info_text.Append("Calibrated Thrust Level: ");
                if (displayed_thrust_limit < 0)
                    info_text.Append("N/A");
                else
                    info_text.Append(displayed_thrust_limit).Append(" %");
            }
        }

        public static bool is_active_control_available(IMyTerminalBlock thruster)
        {
            return update_thruster_flags(thruster);
        }

        public static bool is_under_active_control(IMyTerminalBlock thruster)
        {
            return is_active_control_available(thruster) && _current_active_control_on;
        }

        public static void set_active_control(IMyTerminalBlock thruster, bool new_state_on)
        {
            if (!is_active_control_available(thruster))
                return;
            
            _thruster_hosts[thruster].set_thruster_steering(thruster.EntityId, new_state_on);
            set_switch(thruster, _current_thruster_settings, ref _thruster_switches, sync_helper.message_types.THRUSTER_MODES, STEERING, new_state_on, 
                THRUSTER_MODE_FIELD_START, THRUSTER_MODE_FIELD_END);
            _current_active_control_on = new_state_on;
            if (new_state_on && is_thrust_limiter_on(thruster))
                set_anti_slip(thruster, true);
        }

        public static bool is_anti_slip_available(IMyTerminalBlock thruster)
        {
            return is_active_control_available(thruster) && _current_active_control_on && !_current_thrust_limiter_on;
        }

        public static bool is_anti_slip(IMyTerminalBlock thruster)
        {
            return is_active_control_available(thruster) && _current_anti_slip_on;
        }

        public static void set_anti_slip(IMyTerminalBlock thruster, bool new_state_on)
        {
            if (!is_active_control_available(thruster))
                return;

            if (is_under_active_control(thruster) && (new_state_on || !is_thrust_limiter_on(thruster)))
            {
                _thruster_hosts[thruster].set_thruster_trim(thruster.EntityId, new_state_on);
                set_switch(thruster, _current_thruster_settings, ref _thruster_switches, sync_helper.message_types.THRUSTER_MODES, THRUST_TRIM, new_state_on,
                    THRUSTER_MODE_FIELD_START, THRUSTER_MODE_FIELD_END);
                _current_anti_slip_on = new_state_on;
            }
        }

        public static bool is_rotational_only(IMyTerminalBlock thruster)
        {
            return is_active_control_available(thruster) && _current_disable_linear;
        }

        public static void toggle_linear_input(IMyTerminalBlock thruster, bool new_state_on)
        {
            if (!is_active_control_available(thruster))
                return;

            _thruster_hosts[thruster].set_thruster_linear_input(thruster.EntityId, new_state_on);
            set_switch(thruster, _current_thruster_settings, ref _thruster_switches, sync_helper.message_types.THRUSTER_MODES, LINEAR_OFF, new_state_on, 
                THRUSTER_MODE_FIELD_START, THRUSTER_MODE_FIELD_END);
            _current_disable_linear = new_state_on;
        }

        public static bool is_thrust_limiter_available(IMyTerminalBlock thruster)
        {
            return is_active_control_available(thruster) && !_current_disable_linear && (!_current_active_control_on || _current_anti_slip_on);
        }

        public static bool is_thrust_limiter_on(IMyTerminalBlock thruster)
        {
            return is_active_control_available(thruster) && _current_thrust_limiter_on;
        }

        public static void set_thrust_limiter(IMyTerminalBlock thruster, bool new_state_on)
        {
            if (!is_active_control_available(thruster))
                return;

            new_state_on &= is_thrust_limiter_available(thruster);
            _thruster_hosts[thruster].set_thruster_limiter(thruster.EntityId, new_state_on);
            set_switch(thruster, _current_thruster_settings, ref _thruster_switches, sync_helper.message_types.THRUSTER_MODES, THRUST_LIMIT, new_state_on, 
                THRUSTER_MODE_FIELD_START, THRUSTER_MODE_FIELD_END);
            _current_thrust_limiter_on = new_state_on;
        }

        public static float get_manual_throttle(IMyTerminalBlock thruster)
        {
            if (!is_active_control_available(thruster))
                return 0.0f;
            return _manual_throttle * 100.0f / MANUAL_THROTTLE_SCALE;
        }

        public static void set_manual_throttle(IMyTerminalBlock thruster, float new_setting)
        {
            engine_control_unit host;
            if (!_thruster_hosts.TryGetValue(thruster, out host))
                return;

            update_thruster_flags(thruster);
            if (new_setting < 0.0f)
                _manual_throttle = 0;
            else
            {
                new_setting /= 100.0f;
                if (new_setting > 1.0f)
                    new_setting = 1.0f;
                _manual_throttle = (int) (new_setting * MANUAL_THROTTLE_SCALE);
            }
            host.set_manual_throttle(thruster.EntityId, new_setting);
            store_number(_current_thruster_settings, _manual_throttle, MANUAL_THROTTLE_FIELD_START, MANUAL_THROTTLE_FIELD_END);
            thruster.Storage[_uuid] = _current_thruster_settings.ToString();
            sync_helper.encode_signed(_manual_throttle, 2, _message, 0);
            sync_helper.send_message_to_others(sync_helper.message_types.MANUAL_THROTTLE, thruster, _message, 2);
        }

        public static void throttle_status(IMyTerminalBlock thruster, StringBuilder status)
        {
            float throttle = get_manual_throttle(thruster);
            status.Clear();
            status.Append((throttle < 1.0f) ? "Disabled" : string.Format("{0:F0} %", throttle));
        }

        #endregion

        #region Grid-wide settings handlers

        public static void remote_grid_settings(sync_helper.message_types message_type, object entity, byte[] argument, int length)
        {
            var grid = entity as IMyCubeGrid;
            if (grid == null)
                return;

            if (message_type == sync_helper.message_types.GRID_CONTROL_SENSITIVITY && length == 2)
            {
                _grid_control_sensitivity = (int) sync_helper.decode_signed(2, argument, 0);
                //log_tagger_action("remote_grid_settings", $"received \"{grid.DisplayName}\" P={_grid_control_sensitivity / GRID_CONTROL_SENSITIVITY_SCALE}");
                update_grid_flags(grid, use_remote_sensitivity: true);
            }
            else if (message_type == sync_helper.message_types.GRID_MODES)
            {
                if (length == 1)
                {
                    _grid_switches = argument[0];
                    //log_tagger_action("remote_grid_settings", $"received \"{grid.DisplayName}\" S={_grid_switches:X}h");
                    update_grid_flags(grid, use_remote_switches: true);
                }
                else if (length == 8 && sync_helper.running_on_server)
                {
                    update_grid_flags(grid);
                    _message[0] = (byte) _grid_switches;
                    sync_helper.encode_signed(_grid_control_sensitivity, 2, _message, 1);
                    ulong recipient = sync_helper.decode_unsigned(8, argument, 0);
                    //log_tagger_action("remote_grid_settings", $"sending \"{grid.DisplayName}\" S={_grid_switches:X}h+M={_current_manoeuvre}+P={_grid_control_sensitivity / GRID_CONTROL_SENSITIVITY_SCALE} to {recipient}");
                    sync_helper.send_message_to(recipient, sync_helper.message_types.GRID_MODES, grid, _message, 3);
                }
                else if (length == 3 && !sync_helper.running_on_server)
                {
                    _pending_grid_requests.Remove(grid);
                    _have_pending_requests = _pending_thruster_requests != null && _pending_thruster_requests.Count > 0
                                          || _pending_grid_requests     != null && _pending_grid_requests.Count     > 0;

                    _grid_switches            = argument[0];
                    _grid_control_sensitivity = (int) sync_helper.decode_signed(2, argument, 1);
                    //log_tagger_action("remote_grid_settings", $"received \"{grid.DisplayName}\" S={_grid_switches:X}h+M={_current_manoeuvre}+P={_grid_control_sensitivity / GRID_CONTROL_SENSITIVITY_SCALE}");
                    update_grid_flags(grid, use_remote_switches: true, use_remote_sensitivity: true);
                }
            }
        }

        public static void load_grid_settings(IMyCubeGrid grid, grid_logic grid_handler)
        {
            sync_helper.register_entity(sync_helper.message_types.GRID_MODES              , grid, grid.EntityId);
            sync_helper.register_entity(sync_helper.message_types.GRID_CONTROL_SENSITIVITY, grid, grid.EntityId);
            _grid_handlers[grid] = grid_handler;
            _grid_settings[grid] = new StringBuilder(new string('0', GRID_FIELDS_LENGTH));

            if (grid.Storage == null)
                grid.Storage = new MyModStorageComponent();

            if (!sync_helper.running_on_server)
            {
                //log_tagger_action("load_grid_settings", $"queueing \"{grid.DisplayName}\" state for player {screen_info.local_player.SteamUserId}");
                if (_pending_grid_requests == null)
                    _pending_grid_requests = new HashSet<IMyCubeGrid>();
                _pending_grid_requests.Add(grid);
                _have_pending_requests = true;
                return;
            }

            string grid_data = grid.Storage.ContainsKey(_uuid) ? grid.Storage[_uuid] : "";
            _grid_switches = (grid_data.Length > GRID_MODE_FIELD_END) ? parse_number(grid_data, GRID_MODE_FIELD_START, GRID_MODE_FIELD_END) : 0;
            _grid_control_sensitivity = (grid_data.Length > GRID_CONTROL_SENSITIVITY_FIELD_END)
                ? parse_number(grid_data, GRID_CONTROL_SENSITIVITY_FIELD_START, GRID_CONTROL_SENSITIVITY_FIELD_END)
                : ((int) (5.0f * GRID_CONTROL_SENSITIVITY_SCALE));
            update_grid_flags(grid, use_remote_switches: true, use_remote_sensitivity: true);
            //log_tagger_action("load_grid_settings", $"\"{grid.DisplayName}\" S={_grid_switches:X}h M={_current_manoeuvre} P={_grid_control_sensitivity}");
        }

        public static void dispose_grid(IMyCubeGrid grid)
        {
            sync_helper.deregister_entity(sync_helper.message_types.GRID_MODES              , grid.EntityId);
            sync_helper.deregister_entity(sync_helper.message_types.GRID_CONTROL_SENSITIVITY, grid.EntityId);
        }

        private static bool update_grid_flags(IMyCubeGrid grid, bool use_remote_switches = false, bool use_remote_sensitivity = false)
        {
            if (!_grid_handlers.ContainsKey(grid))
                return false;
            
            if (grid == _current_grid && !use_remote_switches && !use_remote_sensitivity)
                return true;

            _current_grid          = grid;
            _current_grid_settings = _grid_settings[grid];
            if (!use_remote_switches)
                _grid_switches = parse_number(_current_grid_settings, GRID_MODE_FIELD_START, GRID_MODE_FIELD_END);
            else
            {
                store_number(_current_grid_settings, _grid_switches, GRID_MODE_FIELD_START, GRID_MODE_FIELD_END);
                grid.Storage[_uuid] = _current_grid_settings.ToString();
            }
            if (!use_remote_sensitivity)
                _grid_control_sensitivity = parse_number(_current_grid_settings, GRID_CONTROL_SENSITIVITY_FIELD_START, GRID_CONTROL_SENSITIVITY_FIELD_END);
            else
            {
                store_number(_current_grid_settings, _grid_control_sensitivity, GRID_CONTROL_SENSITIVITY_FIELD_START, GRID_CONTROL_SENSITIVITY_FIELD_END);
                grid.Storage[_uuid] = _current_grid_settings.ToString();
            }
            _CoT_mode_on               = (_grid_switches &                COT_MODE) != 0;
            _touchdown_mode_on         = (_grid_switches &          TOUCHDOWN_MODE) != 0;
            _rotational_damping_on     = (_grid_switches & ROTATIONAL_DAMNPING_OFF) == 0;
            _individual_calibration_on = (_grid_switches &  INDIVIDUAL_CALIBRATION) != 0;

            if (use_remote_switches)
            {
                grid_logic grid_handler = _grid_handlers[grid];
                grid_handler.set_CoT_mode                  (_CoT_mode_on              );
                grid_handler.set_rotational_damping        (_rotational_damping_on    );
                grid_handler.set_touchdown_mode            (_touchdown_mode_on        );
                grid_handler.set_individual_calibration_use(_individual_calibration_on);
                grid_handler.set_damper_enabled_axes(
                    (_grid_switches & IDO_FORE_AFT      ) == 0,
                    (_grid_switches & IDO_PORT_STARBOARD) == 0,
                    (_grid_switches & IDO_DORSAL_VENTRAL) == 0);
            }
            if (use_remote_sensitivity)
                _grid_handlers[grid].set_thrust_control_sensitivity(_grid_control_sensitivity / GRID_CONTROL_SENSITIVITY_SCALE);

            return true;
        }

        public static bool is_grid_control_available(IMyTerminalBlock controller)
        {
            var ship_controller = (MyShipController) controller;
            if (!ship_controller.ControlThrusters)
                return false;

            IMyCubeGrid grid = controller.CubeGrid;
            if (((MyCubeGrid) grid).HasMainCockpit() && !ship_controller.IsMainCockpit)
                return false;
            if (controller != _last_controller && (!sync_helper.running_on_server || screen_info.local_player != null))
            {
                controller.RefreshCustomInfo();
                _last_controller = controller;
            }
            if (!update_grid_flags(grid))
                return false;
            return _grid_handlers[grid].is_thrust_control_available;
        }

        public static bool is_grid_CoT_mode_on(IMyTerminalBlock controller)
        {
            return update_grid_flags(controller.CubeGrid) && _CoT_mode_on;
        }

        public static void set_grid_CoT_mode(IMyTerminalBlock controller, bool new_state_on)
        {
            IMyCubeGrid grid = controller.CubeGrid;
            if (!update_grid_flags(grid))
                return;
            set_switch(grid, _current_grid_settings, ref _grid_switches, sync_helper.message_types.GRID_MODES, COT_MODE, new_state_on, 
                GRID_MODE_FIELD_START, GRID_MODE_FIELD_END);
            _CoT_mode_on = new_state_on;
            _grid_handlers[grid].set_CoT_mode(new_state_on);
        }

        public static bool use_individual_calibration(IMyTerminalBlock controller)
        {
            return update_grid_flags(controller.CubeGrid) && _individual_calibration_on;
        }

        public static void choose_calibration_method(IMyTerminalBlock controller, bool use_individual_calibration)
        {
            IMyCubeGrid grid = controller.CubeGrid;
            if (!update_grid_flags(grid))
                return;
            set_switch(grid, _current_grid_settings, ref _grid_switches, sync_helper.message_types.GRID_MODES, INDIVIDUAL_CALIBRATION, use_individual_calibration, 
                GRID_MODE_FIELD_START, GRID_MODE_FIELD_END);
            _individual_calibration_on = use_individual_calibration;
            _grid_handlers[grid].set_individual_calibration_use(use_individual_calibration);
        }

        public static bool is_grid_rotational_damping_on(IMyTerminalBlock controller)
        {
            return update_grid_flags(controller.CubeGrid) && _rotational_damping_on;
        }

        public static void set_grid_rotational_damping(IMyTerminalBlock controller, bool new_state_on)
        {
            IMyCubeGrid grid = controller.CubeGrid;
            if (!update_grid_flags(grid))
                return;
            set_switch(grid, _current_grid_settings, ref _grid_switches, sync_helper.message_types.GRID_MODES, ROTATIONAL_DAMNPING_OFF, !new_state_on, 
                GRID_MODE_FIELD_START, GRID_MODE_FIELD_END);
            _rotational_damping_on = new_state_on;
            _grid_handlers[grid].set_rotational_damping(new_state_on);
        }

        public static bool is_grid_touchdown_mode_on(IMyTerminalBlock controller)
        {
            return update_grid_flags(controller.CubeGrid) && _touchdown_mode_on;
        }

        public static bool is_grid_touchdown_mode_available(IMyTerminalBlock controller)
        {
            return is_grid_control_available(controller) && _grid_handlers[controller.CubeGrid].is_landing_mode_available;
        }

        public static void set_grid_touchdown_mode(IMyTerminalBlock controller, bool new_state_on)
        {
            IMyCubeGrid grid = controller.CubeGrid;
            if (!update_grid_flags(grid))
                return;
            set_switch(grid, _current_grid_settings, ref _grid_switches, sync_helper.message_types.GRID_MODES, TOUCHDOWN_MODE, new_state_on, 
                GRID_MODE_FIELD_START, GRID_MODE_FIELD_END);
            _touchdown_mode_on = new_state_on;
            _grid_handlers[grid].set_touchdown_mode(new_state_on);
        }

        public static Func<IMyTerminalBlock, bool> create_damper_override_reader(int axis)
        {
            return delegate (IMyTerminalBlock controller)
            {
                return is_grid_control_available(controller) && _grid_handlers[controller.CubeGrid].is_ID_axis_overriden(controller, axis);
            };
        }

        public static Action<IMyTerminalBlock, bool> create_damper_override_setter(int axis)
        {
            return delegate (IMyTerminalBlock controller, bool new_state)
            {
                IMyCubeGrid grid = controller.CubeGrid;
                _grid_handlers[grid].set_ID_override(controller, axis, new_state);
                if (!update_grid_flags(grid))
                    return;
                Vector3 dampers_axis_enabled = _grid_handlers[grid].dampers_axes_enabled;
                toggle_switch(ref _grid_switches, IDO_FORE_AFT      , dampers_axis_enabled.Z < 0.5f);
                toggle_switch(ref _grid_switches, IDO_PORT_STARBOARD, dampers_axis_enabled.X < 0.5f);
                toggle_switch(ref _grid_switches, IDO_DORSAL_VENTRAL, dampers_axis_enabled.Y < 0.5f);
                store_number(_current_grid_settings, _grid_switches, GRID_MODE_FIELD_START, GRID_MODE_FIELD_END);
                _current_grid.Storage[_uuid] = _current_grid_settings.ToString();

                _message[0] = (byte) _grid_switches;
                sync_helper.send_message_to_others(sync_helper.message_types.GRID_MODES, grid, _message, 1);
            };
        }

        public static float get_control_sensitivity(IMyTerminalBlock controller)
        {
            IMyCubeGrid grid = controller.CubeGrid;
            if (!update_grid_flags(grid))
                return 0.0f;
            return _grid_control_sensitivity / GRID_CONTROL_SENSITIVITY_SCALE;
        }

        public static void set_control_sensitivity(IMyTerminalBlock controller, float new_sensitivity)
        {
            IMyCubeGrid grid = controller.CubeGrid;
            if (!update_grid_flags(grid))
                return;
            _grid_control_sensitivity = (int) (new_sensitivity * GRID_CONTROL_SENSITIVITY_SCALE);
            if (_grid_control_sensitivity < MIN_CONTROL_SENSITIVITY * GRID_CONTROL_SENSITIVITY_SCALE)
                _grid_control_sensitivity = (int) (MIN_CONTROL_SENSITIVITY * GRID_CONTROL_SENSITIVITY_SCALE);
            else if (_grid_control_sensitivity > short.MaxValue)
                _grid_control_sensitivity = short.MaxValue;
            _grid_handlers[grid].set_thrust_control_sensitivity(_grid_control_sensitivity / GRID_CONTROL_SENSITIVITY_SCALE);
            store_number(_current_grid_settings, _grid_control_sensitivity, GRID_CONTROL_SENSITIVITY_FIELD_START, GRID_CONTROL_SENSITIVITY_FIELD_END);
            grid.Storage[_uuid] = _current_grid_settings.ToString();
            sync_helper.encode_signed(_grid_control_sensitivity, 2, _message, 0);
            sync_helper.send_message_to_others(sync_helper.message_types.GRID_CONTROL_SENSITIVITY, grid, _message, 2);
        }

        public static void control_sensitivity_text(IMyTerminalBlock controller, StringBuilder status)
        {
            status.Clear();
            IMyCubeGrid grid = controller.CubeGrid;
            if (!update_grid_flags(grid))
                return;
            status.AppendFormat("{0:F2}", _grid_control_sensitivity / GRID_CONTROL_SENSITIVITY_SCALE);
        }

        #endregion

        public static void handle_4Hz()
        {
            _last_controller = null;
        }

        public static void handle_2s_period()
        {
            if (!_have_pending_requests || sync_helper.running_on_server)
                return;

            sync_helper.encode_unsigned(screen_info.local_player.SteamUserId, 8, _message, 0);
            if (_pending_thruster_requests != null)
            {
                foreach (IMyTerminalBlock thruster in _pending_thruster_requests)
                {
                    //log_tagger_action("handle_2s_period", $"requesting \"{thruster.CustomName}\" state for player {screen_info.local_player.SteamUserId}");
                    sync_helper.send_message_to_server(sync_helper.message_types.THRUSTER_MODES, thruster, _message, 8);
                }
            }
            if (_pending_grid_requests != null)
            {
                foreach (IMyCubeGrid grid in _pending_grid_requests)
                {
                    //log_tagger_action("handle_2s_period", $"requesting \"{grid.DisplayName}\" state for player {screen_info.local_player.SteamUserId}");
                    sync_helper.send_message_to_server(sync_helper.message_types.GRID_MODES, grid, _message, 8);
                }
            }
        }
    }
}
