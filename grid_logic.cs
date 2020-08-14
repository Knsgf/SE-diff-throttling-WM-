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

        const float MESSAGE_MULTIPLIER = 1000.0f;

        private static readonly byte[] __message = new byte[sync_helper.MAX_MESSAGE_LENGTH];

        private readonly IMyCubeGrid                    _grid;
        private readonly HashSet<IMyControllableEntity> _ship_controllers = new HashSet<IMyControllableEntity>();
        private readonly HashSet<IMyRemoteControl>      _RC_blocks        = new HashSet<IMyRemoteControl>();

        private List<grid_logic>    _secondary_grids = null;
        private engine_control_unit _ECU             = null;
        private torque_simulation   _grid_physics    = null;
        private IMyPlayer           _prev_player     = null;

        private int     _num_thrusters = 0, _prev_thrust_reduction = 0;
        private bool    _disposed = false;
        private bool    _ID_on = true, _is_secondary = false;
        
        private Vector3 _prev_trim, _prev_aux_trim, _prev_linear_integral;

        #endregion

        #region Properties

        public bool is_thrust_control_available => _num_thrusters > 0;
        public bool is_landing_mode_available   => is_thrust_control_available && _grid.Physics.Gravity.LengthSquared() > 0.01f;

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

        public Vector3 dampers_axes_enabled => _ECU.dampers_axes_enabled;
        public engine_control_unit ECU => _ECU;

        #endregion

        #region Grid modes and ID overrides

        public void set_CoT_mode(bool CoT_mode_on)
        {
            //log_grid_action("set_CoT_mode", CoT_mode_on.ToString());
            _ECU.CoT_mode_on = CoT_mode_on && is_thrust_control_available;
        }

        public void set_individual_calibration_use(bool use_individual_calibration)
        {
            //log_grid_action("set_individual_calibration_use", use_individual_calibration.ToString());
            _ECU.use_individual_calibration = use_individual_calibration && is_thrust_control_available;
        }

        public void set_touchdown_mode(bool touchdown_mode_on, bool set_secondary = false)
        {
            //log_grid_action("set_touchdown_mode", touchdown_mode_on.ToString());
            if (!set_secondary)
            {
                if (_is_secondary)
                    return;
                touchdown_mode_on &= is_landing_mode_available;
            }
            _ECU.touchdown_mode_on = touchdown_mode_on;
            
            if (!set_secondary && !_is_secondary && _secondary_grids != null)
            {
                foreach (grid_logic cur_secondary in _secondary_grids)
                    cur_secondary.set_touchdown_mode(touchdown_mode_on, true);
            }
        }

        public void set_rotational_damping(bool rotational_damping_on, bool set_secondary = false)
        {
            //log_grid_action("set_rotational_damping", rotational_damping_on.ToString());
            if (!set_secondary)
            {
                if (_is_secondary)
                    return;
                rotational_damping_on |= !is_thrust_control_available;
            }
            _ECU.rotational_damping_on = rotational_damping_on;

            if (!set_secondary && !_is_secondary && _secondary_grids != null)
            {
                foreach (grid_logic cur_secondary in _secondary_grids)
                    cur_secondary.set_rotational_damping(rotational_damping_on, true);
            }
        }

        public void set_damper_enabled_axes(bool fore_aft_enable, bool port_starboard_enable, bool dorsal_ventral_enable)
        {
            //log_grid_action("set_damper_enabled_axes", $"Z={fore_aft_enable} X={port_starboard_enable} Y={dorsal_ventral_enable}");
            _ECU.set_damper_enabled_axes(fore_aft_enable, port_starboard_enable, dorsal_ventral_enable);
        }

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

        public void set_ID_override(IMyTerminalBlock controller, int axis, bool new_state_enabled)
        {
            Vector3 new_override = _ECU.get_damper_override_for_cockpit(controller);
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
            return !_ECU.is_under_control_of(screen_info.local_controller) ? null : screen_info.local_player;
        }

        public void update_ECU_cockpit_controls()
        {
            bool CoT_mode_on = false, landing_mode_on = false, rotational_damping_on = true, use_individual_calibration = false, circularise_on = false;
            engine_control_unit ECU = _ECU;

            foreach (IMyControllableEntity cur_controller in _ship_controllers)
            {
                string controller_data = ((IMyTerminalBlock) cur_controller).CustomData;
                CoT_mode_on                = controller_data.ContainsCOTTag();
                rotational_damping_on      = controller_data.ContainsDAMPINGTag();
                use_individual_calibration = controller_data.ContainsICTag();
                circularise_on             = controller_data.ContainsCIRCULARISETag();
                landing_mode_on            = controller_data.ContainsLANDINGTag() && is_landing_mode_available;
                _ID_on                     = cur_controller.EnabledDamping;
                ECU.translate_damper_override(controller_data.IDOverrides(), (IMyTerminalBlock) cur_controller);
                break;
            }
            ECU.CoT_mode_on                = CoT_mode_on;
            ECU.use_individual_calibration = use_individual_calibration;
            ECU.touchdown_mode_on          = landing_mode_on;
            ECU.rotational_damping_on      = rotational_damping_on;
            ECU.linear_dampers_on          = _ID_on;
            ECU.secondary_ECU              = _is_secondary;
        }

        private void initialise_ECU_and_physics()
        {
            var grid_movement = new thruster_physics(_grid);
            _ECU              = new engine_control_unit(_grid, grid_movement);
            _grid_physics     = grid_movement;
        }

        public void set_grid_CoM(Vector3D new_world_CoM)
        {
            _ECU.new_grid_CoM = Vector3D.Transform(new_world_CoM, _grid.PositionComp.WorldMatrixNormalizedInv);
        }

        public void set_average_connected_grid_mass(float average_mass)
        {
            _ECU.average_grid_mass = average_mass;
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

                    _ship_controllers.Add(controller);
                    session_handler.sample_controller(ship_controller);

                    var RC_block = entity as IMyRemoteControl;
                    if (RC_block != null)
                        _RC_blocks.Add(RC_block);
                    return;
                }

                var thruster = entity as IMyThrust;
                if (thruster != null)
                {
                    _ECU.assign_thruster(thruster);
                    session_handler.sample_thruster(thruster);
                    thruster.AppendingCustomInfo += thruster_and_grid_tagger.show_thrust_limit;
                    ++_num_thrusters;
                    return;
                }

                var gyro = entity as IMyGyro;
                if (gyro != null)
                {
                    _ECU.assign_gyroscope(gyro);
                    return;
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
                    _ship_controllers.Remove(controller);

                    var RC_block = entity as IMyRemoteControl;
                    if (RC_block != null)
                        _RC_blocks.Remove(RC_block);
                    return;
                }

                var thruster = entity as IMyThrust;
                if (thruster != null)
                {
                    thruster.AppendingCustomInfo -= thruster_and_grid_tagger.show_thrust_limit;
                    _ECU.dispose_thruster(thruster);
                    --_num_thrusters;
                    return;
                }

                var gyro = entity as IMyGyro;
                if (gyro != null)
                {
                    _ECU.dispose_gyroscope(gyro);
                    return;
                }
            }
        }

        internal static void thrust_reduction_handler(sync_helper.message_types message_type, object entity, byte[] argument, int length)
        {
            var instance = entity as grid_logic;
            if (length != 1 || instance == null || instance._disposed)
                return;

            screen_info.set_displayed_thrust_reduction(instance._grid, argument[0], !instance._is_secondary && instance._secondary_grids == null);
        }

        private static Vector3 structurise_vector(byte[] buffer, int buffer_offset)
        {
            Vector3 result;

            result.X = sync_helper.decode_signed(2, buffer, buffer_offset    ) / MESSAGE_MULTIPLIER;
            result.Y = sync_helper.decode_signed(2, buffer, buffer_offset + 2) / MESSAGE_MULTIPLIER;
            result.Z = sync_helper.decode_signed(2, buffer, buffer_offset + 4) / MESSAGE_MULTIPLIER;
            return result;
        }

        internal static void I_terms_handler(sync_helper.message_types message_type, object entity, byte[] argument, int length)
        {
            if (length != 18 || sync_helper.running_on_server)
                return;

            var instance = entity as grid_logic;
            if (instance == null || instance._disposed)
                return;

            engine_control_unit current_ECU = instance._ECU;
            current_ECU.current_trim    = structurise_vector(argument, 0);
            current_ECU.aux_trim        = structurise_vector(argument, 6);
            current_ECU.linear_integral = structurise_vector(argument, 12);
        }

        #endregion

        #region event triggers

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

            sync_helper.encode_signed((long) (value * MESSAGE_MULTIPLIER), 2, buffer, buffer_offset);
        }

        private static void serialise_vector(Vector3 value, byte[] buffer, int buffer_offset)
        {
            serialise_float_to_short(value.X, buffer, buffer_offset    );
            serialise_float_to_short(value.Y, buffer, buffer_offset + 2);
            serialise_float_to_short(value.Z, buffer, buffer_offset + 4);
        }

        private void send_I_terms_message(ulong? recipient)
        {
            Vector3 current_trim = _ECU.current_trim, aux_trim = _ECU.aux_trim, linear_integral = _ECU.linear_integral;
            if (current_trim != _prev_trim || aux_trim != _prev_aux_trim || linear_integral != _prev_linear_integral)
            {
                _prev_trim            = current_trim;
                _prev_aux_trim        = aux_trim;
                _prev_linear_integral = linear_integral;
                serialise_vector(   current_trim, __message, 0);
                serialise_vector(       aux_trim, __message, 6);
                serialise_vector(linear_integral, __message, 12);
                if (recipient != null)
                    sync_helper.send_message_to((ulong) recipient, sync_helper.message_types.I_TERMS, this, __message, 18, reliable: false);
                else
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
            _ID_on = true;
            foreach (IMyControllableEntity cur_controller in _ship_controllers)
            {
                _ID_on = cur_controller.EnabledDamping;
                break;
            }
            _grid_physics.simulate_torque();

            if (_num_thrusters > 0 || _secondary_grids != null)
            {
                lock (_ECU)
                {
                    if (!_is_secondary)
                    {
                        IMyPlayer controlling_player = get_controlling_player();
                        if (controlling_player != null)
                        {
                            handle_user_input(controlling_player.Controller.ControlledEntity);
                            if (sync_helper.running_on_server)
                                send_I_terms_message(controlling_player.SteamUserId);
                        }
                        else
                        {
                            _ECU.reset_user_input();
                            if (_secondary_grids != null)
                            {
                                foreach (grid_logic cur_secondary in _secondary_grids)
                                    cur_secondary._ECU.reset_user_input();
                            }
                        }

                        if (_secondary_grids != null)
                        {
                            Vector3D world_linear_velocity, world_angular_velocity;
                            Vector3  target_linear_velocity, linear_control, rotation_control, gyro_override;
                            bool     gyro_override_active;

                            _ECU.get_primary_control_parameters(out world_linear_velocity, out target_linear_velocity, out world_angular_velocity, 
                                out linear_control, out rotation_control, out gyro_override_active, out gyro_override);
                            foreach (grid_logic cur_secondary in _secondary_grids)
                            {
                                cur_secondary._ID_on = _ID_on;
                                cur_secondary._ECU.set_secondary_control_parameters(world_linear_velocity, target_linear_velocity, world_angular_velocity, 
                                    linear_control, rotation_control, gyro_override_active, gyro_override);
                            }
                        }
                    }
                    _ECU.linear_dampers_on = _ID_on;
                    _ECU.handle_60Hz();
                }
            }
        }

        public void handle_4Hz_foreground()
        {
            check_disposed();

            if (!_is_secondary)
                session_handler.get_secondary_grids(_grid, ref _secondary_grids);

            lock (_ECU)
            { 
                if (_num_thrusters == 0 && _secondary_grids == null)
                    _ECU.reset_ECU();
                else
                {
                    _ECU.handle_4Hz_foreground();

                    IMyPlayer controlling_player = get_controlling_player();

                    _ECU.autopilot_on = false;
                    foreach (IMyRemoteControl cur_RC_block in _RC_blocks)
                        _ECU.check_autopilot(cur_RC_block);
                    screen_info.set_displayed_vertical_speed(_grid, _ECU.vertical_speed, !_is_secondary);

                    if (sync_helper.running_on_server)
                        send_thrust_reduction_message(controlling_player);
                    _prev_player = controlling_player;
                }
            }
        }

        public void handle_4Hz_background()
        {
            check_disposed();
            if (_num_thrusters > 0 || _secondary_grids != null)
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

            if (_num_thrusters > 0 || _secondary_grids != null)
            {
                lock (_ECU)
                { 
                    _ECU.handle_2s_period_foreground();
                    if (sync_helper.running_on_server)
                        send_I_terms_message(null);
                }
            }
        }

        public void handle_2s_period_background()
        {
            if (_num_thrusters > 0 || _secondary_grids != null)
            {
                lock (_ECU)
                {
                    _ECU.handle_2s_period_background();
                }
            }
        }

        public void perform_individual_calibration()
        {
            if (_num_thrusters > 0)
                _ECU.perform_individual_calibration();
        }

        public grid_logic(IMyCubeGrid new_grid)
        {
            _grid                 = new_grid;
            _grid.OnBlockAdded   += on_block_added;
            _grid.OnBlockRemoved += on_block_removed;
            sync_helper.register_entity(sync_helper.message_types.I_TERMS    , this, _grid.EntityId);
            sync_helper.register_entity(sync_helper.message_types.THRUST_LOSS, this, _grid.EntityId);

            initialise_ECU_and_physics();
            var block_list = new List<IMySlimBlock>();
            _grid.GetBlocks(block_list,
                delegate (IMySlimBlock block)
                {
                    IMyCubeBlock full_block = block.FatBlock;
                    return full_block is IMyCockpit || full_block is IMyRemoteControl || full_block is IMyThrust || full_block is IMyGyro;
                }
            );
            foreach (IMySlimBlock cur_block in block_list)
                on_block_added(cur_block);

            thruster_and_grid_tagger.load_grid_settings(_grid, this);
        }

        public void Dispose()
        {
            if (!_disposed)
            {
                _grid.OnBlockAdded   -= on_block_added;
                _grid.OnBlockRemoved -= on_block_removed;
                sync_helper.deregister_entity(sync_helper.message_types.I_TERMS    , _grid.EntityId);
                sync_helper.deregister_entity(sync_helper.message_types.THRUST_LOSS, _grid.EntityId);
                thruster_and_grid_tagger.dispose_grid(_grid);

                var block_list = new List<IMySlimBlock>();
                _grid.GetBlocks(block_list,
                    delegate (IMySlimBlock block)
                    {
                        IMyCubeBlock full_block = block.FatBlock;
                        return full_block is IMyThrust || full_block is IMyGyro || full_block is IMyCockpit || full_block is IMyRemoteControl;
                    }
                );
                foreach (IMySlimBlock cur_block in block_list)
                    on_block_removed(cur_block);

                _disposed = true;
            }
        }
    }
}
