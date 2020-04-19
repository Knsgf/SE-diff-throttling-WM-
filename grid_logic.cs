using System;
using System.Collections.Generic;

using Sandbox.ModAPI;
using VRage.Game.ModAPI;
using VRage.Game.ModAPI.Interfaces;
using VRage.Utils;
using VRageMath;

namespace ttdtwm
{
    sealed class grid_logic: IDisposable
    {
        #region fields

        const float MESSAGE_MULTIPLIER = 1000.0f, MESSAGE_SHIFT = 128.0f;
        const int   CONTROL_WARNING_OFF = 200, CONTROL_WARNING_ON = 201;
        const int   CONTROLS_TIMEOUT = 2;

        private static byte[] __message = new byte[sync_helper.MAX_MESSAGE_LENGTH];

        private IMyCubeGrid                    _grid;
        private HashSet<IMyControllableEntity> _ship_controllers = new HashSet<IMyControllableEntity>();
        private HashSet<IMyRemoteControl>      _RC_blocks        = new HashSet<IMyRemoteControl>();
        private List<grid_logic>               _secondary_grids  = null;
        private engine_control_unit            _ECU              = null;
        private gravity_simulation             _grid_physics     = null;
        private IMyPlayer                      _prev_player      = null;

        private int     _num_thrusters = 0, _prev_thrust_reduction = 0;
        private bool    _control_limit_reached = false, _disposed = false;
        private bool    _was_in_landing_mode = false, _ID_on = true, _is_secondary = false;
        private Vector3 _prev_trim, _prev_last_trim, _prev_linear_integral;

        #endregion

        #region Properties

        public bool is_CoT_mode_available
        {
            get
            {
                return _num_thrusters > 0 && !_grid.IsStatic && _grid.Physics != null && _grid.Physics.Enabled;
            }
        }

        public bool CoT_mode_on
        {
            get
            {
                return is_CoT_mode_available && _ECU.CoT_mode_on;
            }
            set
            {
                IMyTerminalBlock controller_terminal;

                _ECU.CoT_mode_on = value && is_CoT_mode_available;
                foreach (var cur_controller in _ship_controllers)
                {
                    controller_terminal = (IMyTerminalBlock) cur_controller;
                    controller_terminal.CustomData = value ? controller_terminal.CustomData.AddCOTTag() : controller_terminal.CustomData.RemoveCOTTag();
                }
            }
        }

        public bool rotational_damping_on
        {
            get
            {
                return !is_CoT_mode_available || _ECU.rotational_damping_on;
            }
            set
            {
                set_rotational_damping(value);
            }
        }

        public bool is_landing_mode_available
        {
            get
            {
                return is_CoT_mode_available && _grid.Physics.Gravity.LengthSquared() > 0.01f;
            }
        }

        public bool landing_mode_on
        {
            get
            {
                return is_landing_mode_available && _ECU.landing_mode_on;
            }
            set
            {
                set_landing_mode(value);
            }
        }

        public bool use_individual_calibration
        {
            get
            {
                return is_CoT_mode_available && _ECU.use_individual_calibration;
            }
            set
            {
                IMyTerminalBlock controller_terminal;

                _ECU.use_individual_calibration = value && is_CoT_mode_available;
                foreach (var cur_controller in _ship_controllers)
                {
                    controller_terminal = (IMyTerminalBlock)cur_controller;
                    controller_terminal.CustomData = value ? controller_terminal.CustomData.AddICTag() : controller_terminal.CustomData.RemoveICTag();
                }
            }
        }

        public bool is_circularisation_avaiable
        {
            get
            {
                return gravity_and_physics.world_has_gravity && _ECU.current_speed >= engine_control_unit.MIN_CIRCULARISATION_SPEED;
            }
        }

        public bool circularise
        {
            get
            {
                return is_circularisation_avaiable && _ECU.circularise_on;
            }
            set
            {
                IMyTerminalBlock controller_terminal;

                _ECU.circularise_on = value && is_circularisation_avaiable;
                foreach (var cur_controller in _ship_controllers)
                {
                    controller_terminal = (IMyTerminalBlock)cur_controller;
                    controller_terminal.CustomData = value ? controller_terminal.CustomData.AddCIRCULARISETag() : controller_terminal.CustomData.RemoveCIRCULARISETag();
                }
            }
        }

        public bool is_secondary
        {
            get
            {
                return _is_secondary;
            }
            set
            {
                _ECU.secondary_ECU = _is_secondary = value;
            }
        }

        public Func<orbit_elements> orbit_elements_reader
        {
            get
            {
                return _grid_physics.current_elements_reader;
            }
        }

        public engine_control_unit.ID_maneuvres current_maneuvre
        {
            get
            {
                return is_circularisation_avaiable ? _ECU.current_maneuvre : engine_control_unit.ID_maneuvres.maneuvre_off;
            }
        }

        #endregion

        #region ID overrides and maneuvres

        public bool is_ID_axis_overriden(IMyTerminalBlock controller, int axis)
        {
            Vector3 current_override = _ECU.get_damper_override_for_cockpit(controller);
            switch (axis)
            {
                case 0:
                    return current_override.X >= 0.5f;

                case 1:
                    return current_override.Y >= 0.5f;

                case 2:
                    return current_override.Z >= 0.5f;

                default:
                    throw new ArgumentException("Invalid axis");
            }
        }

        public void set_ID_override(IMyTerminalBlock controller, int axis = -1, bool new_state_enabled = false)
        {
            Vector3 new_override = controller.CustomData.IDOverrides();
            switch (axis)
            {
                case 0:
                    new_override.X = new_state_enabled ? 1.0f : 0.0f;
                    break;

                case 1:
                    new_override.Y = new_state_enabled ? 1.0f : 0.0f;
                    break;

                case 2:
                    new_override.Z = new_state_enabled ? 1.0f : 0.0f;
                    break;
            }
            _ECU.translate_damper_override(new_override, controller);

            IMyTerminalBlock controller_terminal;
            foreach (var cur_controller in _ship_controllers)
            {
                controller_terminal = (IMyTerminalBlock) cur_controller;
                controller_terminal.CustomData = controller_terminal.CustomData.SetIDOvveride(_ECU.get_damper_override_for_cockpit(controller_terminal));
            }
        }

        #endregion

        #region auxiliaries

        private void log_grid_action(string method_name, string message)
        {
            MyLog.Default.WriteLine(string.Format("TTDTWM\tgrid_logic.{0}(): \"{1}\" {2}", method_name, _grid.DisplayName, message));
        }

        private void check_disposed()
        {
            if (_disposed)
                throw new Exception(string.Format("grid_logic for \"{0}\" has been disposed", _grid.DisplayName));
        }

        private IMyPlayer get_controlling_player()
        {
            if (MyAPIGateway.Multiplayer != null)
                return MyAPIGateway.Multiplayer.Players.GetPlayerControllingEntity(_grid);
            if (!_ECU.is_under_control_of(screen_info.local_controller))
                return null;
            return screen_info.local_player;
        }

        private void update_ECU_cockpit_controls()
        {
            bool CoT_mode_on = false, landing_mode_on = false, rotational_damping_on = true, use_individual_calibration = false, circularise_on = false;

            foreach (var cur_controller in _ship_controllers)
            {
                string controller_data = ((IMyTerminalBlock) cur_controller).CustomData;
                CoT_mode_on                = controller_data.ContainsCOTTag();
                rotational_damping_on      = controller_data.ContainsDAMPINGTag();
                use_individual_calibration = controller_data.ContainsICTag();
                circularise_on             = controller_data.ContainsCIRCULARISETag();
                landing_mode_on            = controller_data.ContainsLANDINGTag() && is_landing_mode_available;
                _ID_on                     = cur_controller.EnabledDamping;
                _ECU.translate_damper_override(controller_data.IDOverrides(), (IMyTerminalBlock) cur_controller);
                break;
            }
            _ECU.CoT_mode_on                = CoT_mode_on;
            _ECU.use_individual_calibration = use_individual_calibration;
            _ECU.landing_mode_on            = landing_mode_on;
            _ECU.rotational_damping_on      = rotational_damping_on;
            _ECU.linear_dampers_on          = _ID_on;
            _ECU.circularise_on             = circularise_on;
            _ECU.secondary_ECU              = _is_secondary;
        }

        private void initialise_ECU_and_physics()
        {
            var grid_movement = new gravity_and_physics(_grid);
            _ECU              = new engine_control_unit(_grid, grid_movement);
            _grid_physics     = grid_movement;

            update_ECU_cockpit_controls();
        }

        private void set_rotational_damping(bool new_state, bool set_secondary = false)
        {
            IMyTerminalBlock controller_terminal;

            if (!set_secondary && _is_secondary)
                return;
            _ECU.rotational_damping_on = new_state;
            foreach (var cur_controller in _ship_controllers)
            {
                controller_terminal = (IMyTerminalBlock) cur_controller;
                controller_terminal.CustomData = new_state ? controller_terminal.CustomData.AddDAMPINGTag() : controller_terminal.CustomData.RemoveDAMPINGTag();
            }

            if (!set_secondary && !_is_secondary && _secondary_grids != null)
            {
                foreach (var cur_secondary in _secondary_grids)
                    cur_secondary.set_rotational_damping(new_state, true);
            }
        }

        private void set_landing_mode(bool new_state, bool set_secondary = false)
        {
            IMyTerminalBlock controller_terminal;

            if (!set_secondary && _is_secondary)
                return;
            _ECU.landing_mode_on = new_state;
            foreach (var cur_controller in _ship_controllers)
            {
                controller_terminal = (IMyTerminalBlock) cur_controller;
                controller_terminal.CustomData = new_state ? controller_terminal.CustomData.AddLANDINGTag() : controller_terminal.CustomData.RemoveLANDINGTag();
            }

            if (!set_secondary && !_is_secondary && _secondary_grids != null)
            {
                foreach (var cur_secondary in _secondary_grids)
                    cur_secondary.set_landing_mode(new_state, true);
            }
        }

        #endregion

        #region event handlers

        private void on_block_added(IMySlimBlock block)
        {
            check_disposed();
            IMyCubeBlock entity = block.FatBlock;
            if (entity != null)
            {
                var controller      = entity as IMyControllableEntity;
                var ship_controller = entity as IMyShipController;
                if (controller != null && ship_controller != null)
                {
                    var controller_terminal = (IMyTerminalBlock) controller;

                    controller_terminal.AppendingCustomInfo += gravity_and_physics.list_current_elements;
                    _ship_controllers.Add(controller);
                    session_handler.sample_controller(ship_controller);
                    if (_ECU != null)
                    {
                        if (_ECU.CoT_mode_on)
                            controller_terminal.CustomData = controller_terminal.CustomData.AddCOTTag();
                        if (_ECU.landing_mode_on)
                            controller_terminal.CustomData = controller_terminal.CustomData.AddLANDINGTag();
                        if (!_ECU.rotational_damping_on)
                            controller_terminal.CustomData = controller_terminal.CustomData.RemoveDAMPINGTag();
                        if (_ECU.use_individual_calibration)
                            controller_terminal.CustomData = controller_terminal.CustomData.AddICTag();
                        if (_ECU.circularise_on)
                            controller_terminal.CustomData = controller_terminal.CustomData.AddCIRCULARISETag();
                        controller_terminal.CustomData = controller_terminal.CustomData.SetIDOvveride(_ECU.get_damper_override_for_cockpit(controller_terminal));
                    }

                    var RC_block = entity as IMyRemoteControl;
                    if (RC_block != null)
                        _RC_blocks.Add(RC_block);
                    return;
                }

                var thruster = entity as IMyThrust;
                if (thruster != null)
                {
                    if (_ECU == null)
                        initialise_ECU_and_physics();
                    _ECU.assign_thruster(thruster);
                    session_handler.sample_thruster(thruster);
                    thruster.AppendingCustomInfo += thruster_tagger.show_thrust_limit;
                    ++_num_thrusters;
                    return;
                }

                var gyro = entity as IMyGyro;
                if (gyro != null)
                {
                    if (_ECU == null)
                        initialise_ECU_and_physics();
                    _ECU.assign_gyroscope(gyro);
                }
            }
        }

        private void on_block_removed(IMySlimBlock block)
        {
            check_disposed();
            IMyCubeBlock entity = block.FatBlock;
            if (entity != null)
            {
                var controller = entity as IMyControllableEntity;
                if (controller != null)
                {
                    ((IMyTerminalBlock) controller).AppendingCustomInfo -= gravity_and_physics.list_current_elements;
                    _ship_controllers.Remove(controller);

                    var RC_block = entity as IMyRemoteControl;
                    if (RC_block != null)
                        _RC_blocks.Remove(RC_block);
                    return;
                }

                var thruster = entity as IMyThrust;
                if (thruster != null)
                {
                    thruster.AppendingCustomInfo -= thruster_tagger.show_thrust_limit;
                    _ECU.dispose_thruster(thruster);
                    --_num_thrusters;
                    return;
                }

                var gyro = entity as IMyGyro;
                if (gyro != null)
                    _ECU.dispose_gyroscope(gyro);
            }
        }

        internal static void thrust_reduction_handler(object entity, byte[] argument, int length)
        {
            var instance = entity as grid_logic;
            if (length != 1 || instance == null || instance._disposed)
                return;

            screen_info.set_displayed_thrust_reduction(instance._grid, argument[0], !instance._is_secondary && instance._secondary_grids == null);
        }

        private static float structurise_float_from_short(byte[] buffer, int buffer_offset)
        {
            return ((short) (buffer[buffer_offset] | (buffer[buffer_offset + 1] << 8))) / MESSAGE_MULTIPLIER;
        }

        private static Vector3 structurise_vector(byte[] buffer, int buffer_offset)
        {
            Vector3 result;

            result.X = structurise_float_from_short(buffer, buffer_offset    );
            result.Y = structurise_float_from_short(buffer, buffer_offset + 2);
            result.Z = structurise_float_from_short(buffer, buffer_offset + 4);
            return result;
        }

        internal static void I_terms_handler(object entity, byte[] argument, int length)
        {
            if (length != 18 || MyAPIGateway.Multiplayer == null || MyAPIGateway.Multiplayer.IsServer)
                return;

            var instance = entity as grid_logic;
            if (instance == null || instance._disposed)
                return;

            engine_control_unit current_ECU = instance._ECU;
            current_ECU.current_trim    = structurise_vector(argument, 0);
            current_ECU.last_trim       = structurise_vector(argument, 6);
            current_ECU.linear_integral = structurise_vector(argument, 12);
        }

        internal static void sync_maneuvre(object entity, byte[] argument, int length)
        {
            if (length != 1)
                return;
            var instance = entity as grid_logic;
            if (instance == null || instance._disposed)
                return;

            instance.start_maneuvre((engine_control_unit.ID_maneuvres) argument[0], false);
        }

        #endregion

        #region event triggers

        public void start_maneuvre(engine_control_unit.ID_maneuvres selection, bool sync_maneuvre)
        {
            if (is_circularisation_avaiable)
            {
                _ECU.begin_maneuvre(selection);
                if (sync_maneuvre)
                {
                    __message[0] = (byte) selection;
                    sync_helper.send_message_to_others(sync_helper.message_types.MANEUVRE, this, __message, 1);
                }
            }
        }

        private void send_thrust_reduction_message(IMyPlayer controlling_player)
        {
            if (controlling_player != null && (controlling_player != _prev_player || _prev_thrust_reduction != _ECU.thrust_reduction))
            {
                _prev_thrust_reduction = _ECU.thrust_reduction;
                __message[0]           = (byte) _prev_thrust_reduction;
                if (__message[0] > 100)
                    __message[0] = 100;
                sync_helper.send_message_to(controlling_player.SteamUserId, sync_helper.message_types.THRUST_LOSS, this, __message, 1);
                _prev_thrust_reduction = _ECU.thrust_reduction;
            }
        }

        private static void serialise_float_to_short(float value, byte[] buffer, int buffer_offset)
        {
            if (value > short.MaxValue / MESSAGE_MULTIPLIER)
                value = short.MaxValue / MESSAGE_MULTIPLIER;
            else if (value < short.MinValue / MESSAGE_MULTIPLIER)
                value = short.MinValue / MESSAGE_MULTIPLIER;

            short value16 = (short) (value * MESSAGE_MULTIPLIER);

            buffer[buffer_offset    ] = (byte) (value16 & 0xFF);
            buffer[buffer_offset + 1] = (byte) (value16 >> 8);
        }

        private static void serialise_vector(Vector3 value, byte[] buffer, int buffer_offset)
        {
            serialise_float_to_short(value.X, __message, buffer_offset    );
            serialise_float_to_short(value.Y, __message, buffer_offset + 2);
            serialise_float_to_short(value.Z, __message, buffer_offset + 4);
        }

        private void send_I_terms_message()
        {
            if (_ECU.current_trim != _prev_trim || _ECU.last_trim != _prev_last_trim || _ECU.linear_integral != _prev_linear_integral)
            {
                _prev_trim            = _ECU.current_trim;
                _prev_last_trim       = _ECU.last_trim;
                _prev_linear_integral = _ECU.linear_integral;
                serialise_vector(_prev_trim           , __message, 0);
                serialise_vector(_prev_last_trim      , __message, 6);
                serialise_vector(_prev_linear_integral, __message, 12);
                sync_helper.send_message_to_others(sync_helper.message_types.I_TERMS, this, __message, 18);
            }
        }

        #endregion

        private void handle_user_input(IMyControllableEntity controller)
        {
            var ship_controller = controller as Sandbox.ModAPI.Ingame.IMyShipController;
            Vector3 manual_thrust, manual_rotation;

            if (ship_controller == null)
                manual_thrust = manual_rotation = Vector3.Zero;
            else
            { 
                manual_thrust     = ship_controller.MoveIndicator;
                manual_rotation.X = ship_controller.RotationIndicator.X;
                manual_rotation.Y = ship_controller.RotationIndicator.Y;
                manual_rotation.Z = ship_controller.RollIndicator;
            }
            _ECU.translate_player_input(manual_thrust, manual_rotation, controller);
        }

        public void handle_60Hz()
        {
            check_disposed();
            if (!_grid.IsStatic)
            {
                _ID_on = true;
                foreach (var cur_controller in _ship_controllers)
                {
                    _ID_on = cur_controller.EnabledDamping;
                    break;
                }
                _grid_physics.simulate_gravity_and_torque();

                if (_num_thrusters > 0 || _secondary_grids != null)
                {
                    lock (_ECU)
                    {
                        if (!_is_secondary)
                        {
                            IMyPlayer controlling_player = get_controlling_player();
                            if (controlling_player != null)
                            {
                                if (controlling_player == screen_info.local_player || MyAPIGateway.Multiplayer != null && MyAPIGateway.Multiplayer.IsServer)
                                    handle_user_input(controlling_player.Controller.ControlledEntity);
                            }
                            else
                            {
                                _ECU.reset_user_input(reset_gyros_only: false);
                                if (_secondary_grids != null)
                                {
                                    foreach (var cur_secondary in _secondary_grids)
                                    {
                                        cur_secondary._ECU.reset_user_input(reset_gyros_only: false);
                                    }
                                }
                            }

                            if (_secondary_grids != null)
                            {
                                Vector3D world_linear_velocity, world_angular_velocity;
                                Vector3  target_linear_velocity, linear_control, rotation_control, gyro_override;
                                bool     gyro_override_active, circularisation_on;
                                engine_control_unit.ID_maneuvres current_maneuvre;

                                _ECU.get_primary_control_parameters(out world_linear_velocity, out target_linear_velocity, out world_angular_velocity, 
                                    out linear_control, out rotation_control, out gyro_override_active, out gyro_override, out circularisation_on,
                                    out current_maneuvre);
                                foreach (var cur_secondary in _secondary_grids)
                                {
                                    cur_secondary._ID_on = _ID_on;
                                    cur_secondary._ECU.set_secondary_control_parameters(world_linear_velocity, target_linear_velocity, world_angular_velocity, 
                                        linear_control, rotation_control, gyro_override_active, gyro_override, circularisation_on, current_maneuvre);
                                }
                            }
                        }
                        _ECU.linear_dampers_on = _ID_on;
                        _ECU.handle_60Hz();
                    }
                }
            }
        }

        public void handle_4Hz_foreground()
        {
            check_disposed();

            screen_info.set_displayed_orbital_elements(_grid, _grid_physics.current_elements_reader);
            screen_info.set_displayed_target_plane    (_grid, _grid_physics.plane_alignment_reader );
            if (!_is_secondary)
                session_handler.get_secondary_grids(_grid, ref _secondary_grids);

            lock (_ECU)
            { 
                if (_grid.IsStatic || (_num_thrusters == 0 && _secondary_grids == null))
                    _ECU.reset_ECU();
                else
                {
                    update_ECU_cockpit_controls();
                    _ECU.handle_4Hz_foreground();

                    IMyPlayer controlling_player = get_controlling_player();

                    _ECU.autopilot_on = false;
                    foreach (var cur_RC_block in _RC_blocks)
                        _ECU.check_autopilot(cur_RC_block);
                    screen_info.set_displayed_vertical_speed(_grid, _ECU.vertical_speed, !_is_secondary);

                    if (MyAPIGateway.Multiplayer == null || MyAPIGateway.Multiplayer.IsServer)
                        send_thrust_reduction_message(controlling_player);
                    _was_in_landing_mode = _ECU.landing_mode_on;
                    _prev_player         = controlling_player;
                }
            }
        }

        public void handle_4Hz_background()
        {
            check_disposed();
            if (!_grid.IsStatic && (_num_thrusters > 0 || _secondary_grids != null))
            {
                lock (_ECU)
                {
                    _ECU.handle_4Hz_background();
                }
            }
        }

        public void handle_2s_period_foreground()
        {
            check_disposed();

            if (!_grid.IsStatic && (_num_thrusters > 0 || _secondary_grids != null))
            {
                lock (_ECU)
                { 
                    _ECU.handle_2s_period_foreground();
                    if (MyAPIGateway.Multiplayer != null && MyAPIGateway.Multiplayer.IsServer)
                        send_I_terms_message();
                }
            }
        }

        public void handle_2s_period_background()
        {
            _grid_physics.update_current_reference();
            if (!_grid.IsStatic && (_num_thrusters > 0 || _secondary_grids != null))
            {
                lock (_ECU)
                {
                    _ECU.handle_2s_period_background();
                }
            }
        }

        public void perform_individual_calibration()
        {
            if (!_grid.IsStatic && _num_thrusters > 0)
                _ECU.perform_individual_calibration();
        }

        public grid_logic(IMyCubeGrid new_grid)
        {
            _grid                 = new_grid;
            _grid.OnBlockAdded   += on_block_added;
            _grid.OnBlockRemoved += on_block_removed;
            sync_helper.register_entity(this, _grid.EntityId);

            var block_list = new List<IMySlimBlock>();
            _grid.GetBlocks(block_list,
                delegate (IMySlimBlock block)
                {
                    return block.FatBlock is IMyCockpit || block.FatBlock is IMyRemoteControl;
                }
            );
            foreach (var cur_block in block_list)
                on_block_added(cur_block);

            block_list.Clear();
            _grid.GetBlocks(block_list,
                delegate (IMySlimBlock block)
                {
                    return block.FatBlock is IMyThrust || block.FatBlock is IMyGyro;
                }
            );
            foreach (var cur_block in block_list)
                on_block_added(cur_block);
            if (_ECU == null)
                initialise_ECU_and_physics();
        }

        public void Dispose()
        {
            if (!_disposed)
            {
                _grid.OnBlockAdded   -= on_block_added;
                _grid.OnBlockRemoved -= on_block_removed;
                sync_helper.deregister_entity(_grid.EntityId);
                _grid_physics.Dispose();

                var block_list = new List<IMySlimBlock>();
                _grid.GetBlocks(block_list,
                    delegate (IMySlimBlock block)
                    {
                        return block.FatBlock is IMyThrust || block.FatBlock is IMyGyro || block.FatBlock is IMyCockpit || block.FatBlock is IMyRemoteControl;
                    }
                );
                foreach (var cur_block in block_list)
                    on_block_removed(cur_block);

                _disposed = true;
            }
        }
    }
}
