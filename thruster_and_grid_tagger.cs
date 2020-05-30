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

namespace orbiter_SE
{
    static class thruster_and_grid_tagger
    {
        const int STEERING = 0x1, THRUST_TRIM = 0x2, THRUST_LIMIT = 0x4, LINEAR_OFF = 0x8;
        const int COT_MODE = 0x1, INDIVIDUAL_CALIBRATION = 0x2, TOUCHDOWN_MODE = 0x4, ROTATIONAL_DAMNPING_OFF = 0x8;
        const int IDO_FORE_AFT = 0x10, IDO_PORT_STARBOARD = 0x20, IDO_DORSAL_VENTRAL = 0x40, ID_CIRCULARISE = 0x80;

        const float manual_throttle_scale = 9990.0f;

        const int THRUSTER_MODE_FIELD_START   = 0;
        const int THRUSTER_MODE_FIELD_END     = THRUSTER_MODE_FIELD_START   + 1;
        const int MANUAL_THROTTLE_FIELD_START = THRUSTER_MODE_FIELD_END     + 1;
        const int MANUAL_THROTTLE_FIELD_END   = MANUAL_THROTTLE_FIELD_START + 3;
        const int THRUSTER_FIELDS_LENGTH      = MANUAL_THROTTLE_FIELD_END   + 1;

        const int GRID_MODE_FIELD_START      = 0;
        const int GRID_MODE_FIELD_END        = GRID_MODE_FIELD_START      + 2;
        const int GRID_MANOEUVRE_FIELD_START = GRID_MODE_FIELD_END        + 1;
        const int GRID_MANOEUVRE_FIELD_END   = GRID_MANOEUVRE_FIELD_START + 0;
        const int GRID_FIELDS_LENGTH         = GRID_MANOEUVRE_FIELD_END   + 1;

        private static readonly Guid _uuid = new Guid("A78B4F70-BC21-4BAF-A403-855F82C2856F");

        private static readonly Dictionary<IMyCubeGrid, grid_logic> _grid_handlers = new Dictionary<IMyCubeGrid, grid_logic>();
        private static readonly Dictionary<IMyCubeGrid, StringBuilder> _grid_settings = new Dictionary<IMyCubeGrid, StringBuilder>();

        private static readonly Dictionary<IMyTerminalBlock, engine_control_unit> _thruster_hosts = new Dictionary<IMyTerminalBlock, engine_control_unit>();
        private static readonly Dictionary<IMyTerminalBlock, StringBuilder> _thruster_settings = new Dictionary<IMyTerminalBlock, StringBuilder>();

        private static StringBuilder _current_thruster_settings = null, _current_grid_settings = null;
        
        private static string           _thruster_data, _throttle_setting;
        private static IMyTerminalBlock _current_thruster = null;
        private static bool             _current_active_control_on, _current_anti_slip_on, _current_disable_linear, _current_thrust_limiter_on, _settings_changed = false;
        private static int              _thruster_switches, _manual_throttle;

        private static IMyCubeGrid _current_grid = null;
        private static int         _grid_switches;
        private static bool        _CoT_mode_on, _individual_calibration_on, _touchdown_mode_on, _rotational_damping_on, _circularisation_on;
        private static bool        _ID_fore_aft_overridden, _ID_port_starboard_overridden, _ID_dorsal_ventral_overridden;
        private static engine_control_unit.ID_manoeuvres _current_manoeuvre;

        private static readonly byte[] _message = new byte[1];

        public static int displayed_thrust_limit { get; set; }

        #region Auxiliaries

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
                digit = (char) (number / divider + '0');
                input[index] = digit;
                number %= divider;
                divider /= 10;
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
        }

        #endregion

        #region Thrusters' settings handlers

        private static void update_thruster_flags(IMyTerminalBlock thruster)
        {
            if (!(thruster is IMyThrust))
            {
                _current_thruster = null;
                _throttle_setting = null;
                _current_thruster_settings    = null;
                _current_active_control_on = _current_anti_slip_on = _current_disable_linear = _current_thrust_limiter_on = false;
                _manual_throttle = 0;
                return;
            }
            if (thruster == _current_thruster)
                return;

            _current_thruster          = thruster;
            _current_thruster_settings = _thruster_settings[thruster];
            if (thruster.Storage.ContainsKey(_uuid))
            {
                _thruster_switches         = parse_number(_current_thruster_settings, THRUSTER_MODE_FIELD_START, THRUSTER_MODE_FIELD_END);
                _manual_throttle           = parse_number(_current_thruster_settings, MANUAL_THROTTLE_FIELD_START, MANUAL_THROTTLE_FIELD_END);
                _current_active_control_on = (_thruster_switches &     STEERING) != 0;
                _current_anti_slip_on      = (_thruster_switches &  THRUST_TRIM) != 0;
                _current_thrust_limiter_on = (_thruster_switches & THRUST_LIMIT) != 0;
                _current_disable_linear    = (_thruster_switches &   LINEAR_OFF) != 0;
                thruster.RefreshCustomInfo();
            }
            /*
            _thruster_data    = thruster.CustomData;
            bool contains_RCS = _thruster_data.ContainsRCSTag();
            _current_active_control_on = _thruster_data.ContainsTHRTag()  ||  contains_RCS;
            _current_anti_slip_on      = _thruster_data.ContainsSLPTag();
            _current_disable_linear    = _thruster_data.ContainsNLTag();
            _current_thrust_limiter_on = _thruster_data.ContainsSTATTag() && !contains_RCS;

            _throttle_setting  = "TTDTWM_MT_" + thruster.EntityId.ToString();
            bool setting_saved = MyAPIGateway.Utilities.GetVariable(_throttle_setting, out _manual_throttle);
            if (!setting_saved)
                _manual_throttle = 0;

            _thruster_switches = 0;
            toggle_switch(ref _thruster_switches, STEERING, _current_active_control_on);
            toggle_switch(ref _thruster_switches, THRUST_TRIM, _current_anti_slip_on);
            toggle_switch(ref _thruster_switches, THRUST_LIMIT, _current_thrust_limiter_on);
            toggle_switch(ref _thruster_switches, LINEAR_OFF, _current_disable_linear);
            store_number(_current_thruster_settings, _thruster_switches, THRUSTER_MODE_FIELD_START, THRUSTER_MODE_FIELD_END);
            _manual_throttle = (int) (_manual_throttle * manual_throttle_scale / 100.0f);
            store_number(_current_thruster_settings, _manual_throttle, MANUAL_THROTTLE_FIELD_START, MANUAL_THROTTLE_FIELD_END);
            thruster.Storage[_uuid] = _current_thruster_settings.ToString();
            thruster.RefreshCustomInfo();

            attach_ECU(thruster, _thruster_hosts[thruster]);
            update_thruster_flags(thruster);
            */
        }

        public static void attach_ECU(IMyTerminalBlock thruster, engine_control_unit ECU)
        {
            _thruster_hosts   [thruster] = ECU;
            _thruster_settings[thruster] = new StringBuilder(new string('0', THRUSTER_FIELDS_LENGTH));

            string thruster_data;
            if (thruster.Storage == null)
                thruster.Storage = new MyModStorageComponent();
            if (!thruster.Storage.ContainsKey(_uuid))
            {
                _thruster_data    = thruster.CustomData;
                bool contains_RCS = _thruster_data.ContainsRCSTag();
                _current_active_control_on = _thruster_data.ContainsTHRTag()  ||  contains_RCS;
                _current_anti_slip_on      = _thruster_data.ContainsSLPTag();
                _current_disable_linear    = _thruster_data.ContainsNLTag();
                _current_thrust_limiter_on = _thruster_data.ContainsSTATTag() && !contains_RCS;

                _throttle_setting  = "TTDTWM_MT_" + thruster.EntityId.ToString();
                bool setting_saved = MyAPIGateway.Utilities.GetVariable(_throttle_setting, out _manual_throttle);
                if (!setting_saved)
                    _manual_throttle = 0;

                _current_thruster_settings = _thruster_settings[thruster];
                _thruster_switches = 0;
                toggle_switch(ref _thruster_switches, STEERING, _current_active_control_on);
                toggle_switch(ref _thruster_switches, THRUST_TRIM, _current_anti_slip_on);
                toggle_switch(ref _thruster_switches, THRUST_LIMIT, _current_thrust_limiter_on);
                toggle_switch(ref _thruster_switches, LINEAR_OFF, _current_disable_linear);
                store_number(_current_thruster_settings, _thruster_switches, THRUSTER_MODE_FIELD_START, THRUSTER_MODE_FIELD_END);
                _manual_throttle = (int) (_manual_throttle * manual_throttle_scale / 100.0f);
                store_number(_current_thruster_settings, _manual_throttle, MANUAL_THROTTLE_FIELD_START, MANUAL_THROTTLE_FIELD_END);
                MyLog.Default.WriteLine($"attach_ECU(): \"{thruster.CustomName}\" {_thruster_switches:X}h");
                thruster.Storage[_uuid] = _current_thruster_settings.ToString();
            }

            thruster_data = thruster.Storage[_uuid];
            if (thruster_data.Length >= THRUSTER_FIELDS_LENGTH)
            {
                int switches        = parse_number(thruster_data,   THRUSTER_MODE_FIELD_START,   THRUSTER_MODE_FIELD_END);
                int manual_throttle = parse_number(thruster_data, MANUAL_THROTTLE_FIELD_START, MANUAL_THROTTLE_FIELD_END);
                store_number(_thruster_settings[thruster],        switches,   THRUSTER_MODE_FIELD_START,   THRUSTER_MODE_FIELD_END);
                store_number(_thruster_settings[thruster], manual_throttle, MANUAL_THROTTLE_FIELD_START, MANUAL_THROTTLE_FIELD_END);
                ECU.set_thruster_steering    (thruster.EntityId, (switches &     STEERING) != 0);
                ECU.set_thruster_trim        (thruster.EntityId, (switches &  THRUST_TRIM) != 0);
                ECU.set_thruster_limiter     (thruster.EntityId, (switches & THRUST_LIMIT) != 0);
                ECU.set_thruster_linear_input(thruster.EntityId, (switches &   LINEAR_OFF) != 0);
                ECU.set_manual_throttle      (thruster.EntityId, manual_throttle / manual_throttle_scale);
            }
        }

        public static void detach_ECU(IMyTerminalBlock thruster)
        {
            _thruster_hosts.Remove(thruster);
            _thruster_settings.Remove(thruster);
        }

        public static float get_thrust_limit(IMyTerminalBlock thruster)
        {
            engine_control_unit host;

            if (!_thruster_hosts.TryGetValue(thruster, out host))
                return 0;
            return host.extract_thrust_limit(thruster.EntityId);
        }

        public static void show_thrust_limit(IMyTerminalBlock thruster, StringBuilder info_text)
        {
            if (_current_thruster != thruster)
                return;

            int displayed_thrust_limit = (int) (get_thrust_limit(thruster) + 0.5f);
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
            update_thruster_flags(thruster);
            return thruster is IMyThrust;
        }

        public static bool is_under_active_control(IMyTerminalBlock thruster)
        {
            update_thruster_flags(thruster);
            return _current_active_control_on;
        }

        public static void set_active_control(IMyTerminalBlock thruster, bool new_state_on)
        {
            update_thruster_flags(thruster);
            if (!is_active_control_available(thruster))
                return;
            
            if (new_state_on)
            {
                thruster.CustomData        = _current_anti_slip_on ? thruster.CustomData.RemoveRCSTag().AddTHRTag() : thruster.CustomData.RemoveTHRTag().AddRCSTag();
                _current_active_control_on = true;
            }
            else
            {
                thruster.CustomData        = thruster.CustomData.RemoveTHRTag().RemoveRCSTag();
                _current_active_control_on = false;
            }

            _thruster_hosts[thruster].set_thruster_steering(thruster.EntityId, new_state_on);
            set_switch(thruster, _current_thruster_settings, ref _thruster_switches, sync_helper.message_types.THRUSTER_MODES, STEERING, new_state_on, 
                THRUSTER_MODE_FIELD_START, THRUSTER_MODE_FIELD_END);
            _current_active_control_on = new_state_on;
        }

        public static bool is_anti_slip_available(IMyTerminalBlock thruster)
        {
            update_thruster_flags(thruster);
            return _current_active_control_on && !_current_thrust_limiter_on;
        }

        public static bool is_anti_slip(IMyTerminalBlock thruster)
        {
            update_thruster_flags(thruster);
            return _current_anti_slip_on;
        }

        public static void set_anti_slip(IMyTerminalBlock thruster, bool new_state_on)
        {
            update_thruster_flags(thruster);
            if (!is_active_control_available(thruster))
                return;

            if (is_under_active_control(thruster) && !is_thrust_limiter_on(thruster))
            {
                if (!new_state_on)
                {
                    thruster.CustomData   = thruster.CustomData.RemoveTHRTag().AddRCSTag();
                    _current_anti_slip_on = false;
                }
                else
                {
                    thruster.CustomData   = thruster.CustomData.RemoveRCSTag().AddTHRTag();
                    _current_anti_slip_on = true;
                }

                _thruster_hosts[thruster].set_thruster_trim(thruster.EntityId, new_state_on);
                set_switch(thruster, _current_thruster_settings, ref _thruster_switches, sync_helper.message_types.THRUSTER_MODES, THRUST_TRIM, new_state_on,
                    THRUSTER_MODE_FIELD_START, THRUSTER_MODE_FIELD_END);
                _current_anti_slip_on = new_state_on;
            }
        }

        public static bool is_rotational_only(IMyTerminalBlock thruster)
        {
            update_thruster_flags(thruster);
            return _current_disable_linear;
        }

        public static void toggle_linear_input(IMyTerminalBlock thruster, bool new_state_on)
        {
            update_thruster_flags(thruster);
            if (!is_active_control_available(thruster))
                return;

            thruster.CustomData = new_state_on ? thruster.CustomData.AddNLTag() : thruster.CustomData.RemoveNLTag();
            _current_disable_linear = new_state_on;

            _thruster_hosts[thruster].set_thruster_linear_input(thruster.EntityId, new_state_on);
            set_switch(thruster, _current_thruster_settings, ref _thruster_switches, sync_helper.message_types.THRUSTER_MODES, LINEAR_OFF, new_state_on, 
                THRUSTER_MODE_FIELD_START, THRUSTER_MODE_FIELD_END);
            _current_disable_linear = new_state_on;
        }

        public static bool is_thrust_limiter_available(IMyTerminalBlock thruster)
        {
            update_thruster_flags(thruster);
            return is_active_control_available(thruster) && !_current_disable_linear && (!_current_active_control_on || _current_anti_slip_on);
        }

        public static bool is_thrust_limiter_on(IMyTerminalBlock thruster)
        {
            update_thruster_flags(thruster);
            return _current_thrust_limiter_on;
        }

        public static void set_thrust_limiter(IMyTerminalBlock thruster, bool new_state_on)
        {
            update_thruster_flags(thruster);
            if (!is_active_control_available(thruster))
                return;

            if (!new_state_on)
            {
                thruster.CustomData        = thruster.CustomData.RemoveSTATTag();
                _current_thrust_limiter_on = false;
            }
            else if (is_thrust_limiter_available(thruster))
            {
                thruster.CustomData        = thruster.CustomData.AddSTATTag();
                _current_thrust_limiter_on = true;
            }

            new_state_on &= is_thrust_limiter_available(thruster);
            _thruster_hosts[thruster].set_thruster_limiter(thruster.EntityId, new_state_on);
            set_switch(thruster, _current_thruster_settings, ref _thruster_switches, sync_helper.message_types.THRUSTER_MODES, THRUST_LIMIT, new_state_on, 
                THRUSTER_MODE_FIELD_START, THRUSTER_MODE_FIELD_END);
            _current_thrust_limiter_on = new_state_on;
        }

        public static float get_manual_throttle(IMyTerminalBlock thruster)
        {
            update_thruster_flags(thruster);
            return _manual_throttle;
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
                if (new_setting > 100.0f)
                    new_setting = 100.0f;
                _manual_throttle = (int) (new_setting * (manual_throttle_scale / 100.0f));
            }
            host.set_manual_throttle(thruster.EntityId, new_setting);
            store_number(_current_thruster_settings, _manual_throttle, MANUAL_THROTTLE_FIELD_START, MANUAL_THROTTLE_FIELD_END);
            thruster.Storage[_uuid] = _current_thruster_settings.ToString();
        }

        public static void throttle_status(IMyTerminalBlock thruster, StringBuilder status)
        {
            float throttle = get_manual_throttle(thruster);
            status.Clear();
            status.Append((throttle < 1.0f) ? "Disabled" : string.Format("{0:F0} %", throttle));
        }

        #endregion

        #region Grid-wide settings handlers

        public static void load_grid_settings(IMyCubeGrid grid, grid_logic grid_handler)
        {
            _grid_handlers[grid] = grid_handler;
            _grid_settings[grid] = new StringBuilder(new string('0', GRID_FIELDS_LENGTH));

            string grid_data;
            if (grid.Storage == null)
                grid.Storage = new MyModStorageComponent();
            if (!grid.Storage.ContainsKey(_uuid))
            {
                _grid_handlers[grid].update_ECU_cockpit_controls();
                engine_control_unit ECU = _grid_handlers[grid].ECU;
                _CoT_mode_on = ECU.CoT_mode_on;
                _touchdown_mode_on = ECU.touchdown_mode_on;
                _rotational_damping_on = ECU.rotational_damping_on;
                _individual_calibration_on = ECU.use_individual_calibration;
                _circularisation_on = ECU.circularise_on;

                _current_grid_settings = _grid_settings[grid];
                _grid_switches = 0;
                toggle_switch(ref _grid_switches, COT_MODE, _CoT_mode_on);
                toggle_switch(ref _grid_switches, TOUCHDOWN_MODE, _touchdown_mode_on);
                toggle_switch(ref _grid_switches, ROTATIONAL_DAMNPING_OFF, !_rotational_damping_on);
                toggle_switch(ref _grid_switches, INDIVIDUAL_CALIBRATION, _individual_calibration_on);
                toggle_switch(ref _grid_switches, ID_CIRCULARISE, _circularisation_on);

                Vector3 dampers_axis_enabled = ECU.dampers_axes_enabled;
                toggle_switch(ref _grid_switches, IDO_FORE_AFT, dampers_axis_enabled.Z < 0.5f);
                toggle_switch(ref _grid_switches, IDO_PORT_STARBOARD, dampers_axis_enabled.X < 0.5f);
                toggle_switch(ref _grid_switches, IDO_DORSAL_VENTRAL, dampers_axis_enabled.Y < 0.5f);

                MyLog.Default.WriteLine($"load_grid_settings(): \"{grid.DisplayName}\" {_grid_switches:X}h");
                store_number(_current_grid_settings, _grid_switches, GRID_MODE_FIELD_START, GRID_MODE_FIELD_END);
                grid.Storage[_uuid] = _current_grid_settings.ToString();
            }

            grid_data = grid.Storage[_uuid];
            if (grid_data.Length > GRID_MODE_FIELD_END)
            {
                int switches = parse_number(grid_data, GRID_MODE_FIELD_START, GRID_MODE_FIELD_END);
                store_number(_grid_settings[grid], switches, GRID_MODE_FIELD_START, GRID_MODE_FIELD_END);
                grid_handler.set_CoT_mode                  ((switches &                COT_MODE) != 0);
                grid_handler.set_rotational_damping        ((switches & ROTATIONAL_DAMNPING_OFF) == 0);
                grid_handler.set_touchdown_mode            ((switches &          TOUCHDOWN_MODE) != 0);
                grid_handler.set_individual_calibration_use((switches &  INDIVIDUAL_CALIBRATION) != 0);
                grid_handler.set_circularisation           ((switches &          ID_CIRCULARISE) != 0);
                grid_handler.set_damper_enabled_axes(
                    (switches & IDO_FORE_AFT      ) == 0, 
                    (switches & IDO_PORT_STARBOARD) == 0,
                    (switches & IDO_DORSAL_VENTRAL) == 0);
            }
            if (grid_data.Length >= GRID_FIELDS_LENGTH)
            {
                int manoeuvre = parse_number(grid_data, GRID_MANOEUVRE_FIELD_START, GRID_MANOEUVRE_FIELD_END);
                store_number(_grid_settings[grid], manoeuvre, GRID_MANOEUVRE_FIELD_START, GRID_MANOEUVRE_FIELD_END);
                grid_handler.start_manoeuvre((engine_control_unit.ID_manoeuvres) manoeuvre);
            }
        }

        private static void update_grid_flags(IMyCubeGrid grid)
        {
            if (grid == _current_grid)
                return;

            _current_grid          = grid;
            _current_grid_settings = _grid_settings[grid];
            if (grid.Storage.ContainsKey(_uuid))
            {
                _grid_switches     =                                     parse_number(_current_grid_settings, GRID_MODE_FIELD_START, GRID_MODE_FIELD_END);
                _current_manoeuvre = (engine_control_unit.ID_manoeuvres) parse_number(_current_grid_settings, GRID_MANOEUVRE_FIELD_START, GRID_MANOEUVRE_FIELD_END);
                _CoT_mode_on               = (_grid_switches &                COT_MODE) != 0;
                _touchdown_mode_on         = (_grid_switches &          TOUCHDOWN_MODE) != 0;
                _rotational_damping_on     = (_grid_switches & ROTATIONAL_DAMNPING_OFF) == 0;
                _individual_calibration_on = (_grid_switches &  INDIVIDUAL_CALIBRATION) != 0;
                _circularisation_on        = (_grid_switches &          ID_CIRCULARISE) != 0;
            }
            /*
            _grid_handlers[grid].update_ECU_cockpit_controls();
            engine_control_unit ECU = _grid_handlers[grid].ECU;
            _CoT_mode_on = ECU.CoT_mode_on;
            _touchdown_mode_on = ECU.touchdown_mode_on;
            _rotational_damping_on = ECU.rotational_damping_on;
            _individual_calibration_on = ECU.use_individual_calibration;
            _circularisation_on = ECU.circularise_on;

            _grid_switches = 0;
            toggle_switch(ref _grid_switches, COT_MODE, _CoT_mode_on);
            toggle_switch(ref _grid_switches, TOUCHDOWN_MODE, _touchdown_mode_on);
            toggle_switch(ref _grid_switches, ROTATIONAL_DAMNPING_OFF, !_rotational_damping_on);
            toggle_switch(ref _grid_switches, INDIVIDUAL_CALIBRATION, _individual_calibration_on);
            toggle_switch(ref _grid_switches, ID_CIRCULARISE, _circularisation_on);

            Vector3 dampers_axis_enabled = ECU.dampers_axes_enabled;
            toggle_switch(ref _grid_switches, IDO_FORE_AFT, dampers_axis_enabled.Z < 0.5f);
            toggle_switch(ref _grid_switches, IDO_PORT_STARBOARD, dampers_axis_enabled.X < 0.5f);
            toggle_switch(ref _grid_switches, IDO_DORSAL_VENTRAL, dampers_axis_enabled.Y < 0.5f);

            MyLog.Default.WriteLine($"update_grid_flags(): \"{grid.DisplayName}\" {_grid_switches:X}h");
            store_number(_current_grid_settings, _grid_switches, GRID_MODE_FIELD_START, GRID_MODE_FIELD_END);
            _current_grid.Storage[_uuid] = _current_grid_settings.ToString();

            load_grid_settings(grid, _grid_handlers[grid]);
            update_grid_flags(grid);
            */
        }

        public static bool is_grid_control_available(IMyTerminalBlock controller)
        {
            var ship_controller = (MyShipController) controller;
            if (!ship_controller.ControlThrusters)
                return false;

            IMyCubeGrid grid = controller.CubeGrid;
            if (((MyCubeGrid) grid).HasMainCockpit() && !ship_controller.IsMainCockpit)
                return false;
            if (grid != _current_grid)
            {
                update_grid_flags(grid);
                controller.RefreshCustomInfo();
            }
            return _grid_handlers[grid].is_thrust_control_available;
        }

        public static bool is_grid_CoT_mode_on(IMyTerminalBlock controller)
        {
            update_grid_flags(controller.CubeGrid);
            return _CoT_mode_on;
        }

        public static void set_grid_CoT_mode(IMyTerminalBlock controller, bool new_state_on)
        {
            IMyCubeGrid grid = controller.CubeGrid;
            update_grid_flags(grid);
            set_switch(grid, _current_grid_settings, ref _grid_switches, sync_helper.message_types.GRID_MODES, COT_MODE, new_state_on, 
                GRID_MODE_FIELD_START, GRID_MODE_FIELD_END);
            _CoT_mode_on = new_state_on;
            _grid_handlers[grid].set_CoT_mode(new_state_on);
        }

        public static bool use_individual_calibration(IMyTerminalBlock controller)
        {
            update_grid_flags(controller.CubeGrid);
            return _individual_calibration_on;
        }

        public static void choose_calibration_method(IMyTerminalBlock controller, bool use_individual_calibration)
        {
            IMyCubeGrid grid = controller.CubeGrid;
            update_grid_flags(grid);
            set_switch(grid, _current_grid_settings, ref _grid_switches, sync_helper.message_types.GRID_MODES, INDIVIDUAL_CALIBRATION, use_individual_calibration, 
                GRID_MODE_FIELD_START, GRID_MODE_FIELD_END);
            _individual_calibration_on = use_individual_calibration;
            _grid_handlers[grid].set_individual_calibration_use(use_individual_calibration);
        }

        public static bool is_grid_rotational_damping_on(IMyTerminalBlock controller)
        {
            update_grid_flags(controller.CubeGrid);
            return _rotational_damping_on;
        }

        public static void set_grid_rotational_damping(IMyTerminalBlock controller, bool new_state_on)
        {
            IMyCubeGrid grid = controller.CubeGrid;
            update_grid_flags(grid);
            set_switch(grid, _current_grid_settings, ref _grid_switches, sync_helper.message_types.GRID_MODES, ROTATIONAL_DAMNPING_OFF, !new_state_on, 
                GRID_MODE_FIELD_START, GRID_MODE_FIELD_END);
            _rotational_damping_on = new_state_on;
            _grid_handlers[grid].set_rotational_damping(new_state_on);
        }

        public static bool is_grid_touchdown_mode_on(IMyTerminalBlock controller)
        {
            update_grid_flags(controller.CubeGrid);
            return _touchdown_mode_on;
        }

        public static bool is_grid_touchdown_mode_available(IMyTerminalBlock controller)
        {
            return is_grid_control_available(controller) && _grid_handlers[controller.CubeGrid].is_landing_mode_available;
        }

        public static void set_grid_touchdown_mode(IMyTerminalBlock controller, bool new_state_on)
        {
            IMyCubeGrid grid = controller.CubeGrid;
            update_grid_flags(grid);
            set_switch(grid, _current_grid_settings, ref _grid_switches, sync_helper.message_types.GRID_MODES, TOUCHDOWN_MODE, new_state_on, 
                GRID_MODE_FIELD_START, GRID_MODE_FIELD_END);
            _touchdown_mode_on = new_state_on;
            _grid_handlers[grid].set_touchdown_mode(new_state_on);
        }

        public static Func<IMyTerminalBlock, bool> create_damper_override_reader(int axis)
        {
            return delegate (IMyTerminalBlock controller)
            {
                return _grid_handlers[controller.CubeGrid].is_ID_axis_overriden(controller, axis);
            };
        }

        public static Action<IMyTerminalBlock, bool> create_damper_override_setter(int axis)
        {
            return delegate (IMyTerminalBlock controller, bool new_state)
            {
                IMyCubeGrid grid = controller.CubeGrid;
                _grid_handlers[grid].set_ID_override(controller, axis, new_state);
                update_grid_flags(grid);
                Vector3 dampers_axis_enabled = _grid_handlers[grid].dampers_axes_enabled;
                toggle_switch(ref _grid_switches, IDO_FORE_AFT      , dampers_axis_enabled.Z < 0.5f);
                toggle_switch(ref _grid_switches, IDO_PORT_STARBOARD, dampers_axis_enabled.X < 0.5f);
                toggle_switch(ref _grid_switches, IDO_DORSAL_VENTRAL, dampers_axis_enabled.Y < 0.5f);
                store_number(_current_grid_settings, _grid_switches, GRID_MODE_FIELD_START, GRID_MODE_FIELD_END);
                _current_grid.Storage[_uuid] = _current_grid_settings.ToString();
            };
        }

        public static bool is_grid_circularise_mode_available(IMyTerminalBlock controller)
        {
            return is_grid_control_available(controller) && _grid_handlers[controller.CubeGrid].is_circularisation_avaiable;
        }

        public static Action<IMyTerminalBlock> create_ID_mode_selector(bool select_circularise)
        {
            return delegate (IMyTerminalBlock controller)
            {
                IMyCubeGrid grid = controller.CubeGrid;
                update_grid_flags(grid);
                set_switch(grid, _current_grid_settings, ref _grid_switches, sync_helper.message_types.GRID_MODES, ID_CIRCULARISE, select_circularise, 
                    GRID_MODE_FIELD_START, GRID_MODE_FIELD_END);
                _circularisation_on = select_circularise;
                _grid_handlers[controller.CubeGrid].set_circularisation(select_circularise);
            };
        }

        public static Action<IMyTerminalBlock, StringBuilder> create_ID_mode_indicator(bool circularisation_indicator)
        {
            return delegate (IMyTerminalBlock controller, StringBuilder status)
            {
                status.Clear();
                update_grid_flags(controller.CubeGrid);
                if (_circularisation_on == circularisation_indicator)
                    status.Append("Select");
            };
        }

        public static Action<IMyTerminalBlock> create_manoeuvre_starter(engine_control_unit.ID_manoeuvres manoeuvre)
        {
            return delegate (IMyTerminalBlock controller)
            {
                IMyCubeGrid grid = controller.CubeGrid;
                update_grid_flags(grid);
                store_number(_current_thruster_settings, (int) manoeuvre, GRID_MANOEUVRE_FIELD_START, GRID_MANOEUVRE_FIELD_END);
                _current_grid.Storage[_uuid] = _current_grid_settings.ToString();
                _grid_handlers[grid].start_manoeuvre(manoeuvre);
            };
        }

        public static Action<IMyTerminalBlock, StringBuilder> create_manoeuvre_indicator(engine_control_unit.ID_manoeuvres manoeuvre)
        {
            return delegate (IMyTerminalBlock controller, StringBuilder status)
            {
                status.Clear();
                update_grid_flags(controller.CubeGrid);
                if (_current_manoeuvre == manoeuvre)
                    status.Append("Active");
            };
        }

        #endregion
    }
}
