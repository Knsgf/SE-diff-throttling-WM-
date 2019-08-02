using System;
using System.Collections.Generic;
//using System.Text;

//using Sandbox.Game;
//using Sandbox.Game.Gui;
using Sandbox.ModAPI;
using VRage.Game;
using VRage.Game.ModAPI;
using VRage.Game.ModAPI.Interfaces;
//using VRage.Input;
using VRage.Utils;
using VRageMath;

namespace ttdtwm
{
    sealed class grid_logic: IDisposable
    {
        #region fields

        const float MESSAGE_MULTIPLIER = 1000.0f, MESSAGE_SHIFT = 128.0f;
        const int   CONTROL_WARNING_OFF = 200, CONTROL_WARNING_ON = 201/*, CLIENT_ANNOUNCE = 202, SERVER_ACKNOWLEDGE = 203*/;
        //const int   CONTROL_MODE_BASE = 204, LANDING_MODE_ON = 1, COT_MODE_ON = 2;  // 204 - 207
        const int   CONTROLS_TIMEOUT = 2;

        private static byte[] __message = new byte[sync_helper.MAX_MESSAGE_LENGTH];

        private session_handler                _session_ref;
        private IMyCubeGrid                    _grid;
        private HashSet<IMyControllableEntity> _ship_controllers     = new HashSet<IMyControllableEntity>();
        private HashSet<IMyRemoteControl>      _RC_blocks            = new HashSet<IMyRemoteControl>();
        private List<grid_logic>               _secondary_grids      = null;
        private engine_control_unit            _ECU                  = null;
        //private Vector3UByte                   _prev_manual_thrust   = new Vector3UByte(128, 128, 128), _prev_manual_rotation = new Vector3UByte(128, 128, 128);
        private IMyPlayer                      _prev_player          = null;

        private int     _num_thrusters = 0, _prev_thrust_reduction = 0/*, _zero_controls_counter = 0*/;
        private bool    _control_limit_reached = false, _disposed = false/*, _status_shown = false*/;
        private bool    _was_in_landing_mode = false, _was_in_CoT_mode = false, _ID_on = true, _is_secondary = false;
        private Vector3 _prev_trim, _prev_last_trim, _prev_linear_integral;
        //private bool _announced = false;

        #endregion

        #region Properties

        public bool is_CoT_mode_available
        {
            get
            {
                return _num_thrusters > 0 && _ECU != null && !_grid.IsStatic && _grid.Physics != null;
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

                if (_ECU == null)
                    return;
                _ECU.CoT_mode_on = value;
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
                return _ECU == null || !is_CoT_mode_available || _ECU.rotational_damping_on;
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

                if (_ECU == null)
                    return;
                _ECU.use_individual_calibration = value;
                foreach (var cur_controller in _ship_controllers)
                {
                    controller_terminal = (IMyTerminalBlock)cur_controller;
                    controller_terminal.CustomData = value ? controller_terminal.CustomData.AddICTag() : controller_terminal.CustomData.RemoveICTag();
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
                _is_secondary = value;
                if (_ECU != null)
                    _ECU.secondary_ECU = value;
                //log_grid_action("is_secondary.set", _grid.DisplayName + (value ? " secondary" : " primary"));
            }
        }

        #endregion

        #region ID overrides

        public bool is_ID_axis_overriden(IMyTerminalBlock controller, int axis)
        {
            if (_ECU == null)
                return false;

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
            if (_ECU == null)
                return;

            /*
            if (controller == null)
            {
                foreach (var cur_controller in _ship_controllers)
                {
                    controller = (IMyTerminalBlock) cur_controller;
                    break;
                }
                if (controller == null)
                    return;
            }
            */

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
            if (_ECU == null || !_ECU.is_under_control_of(screen_info.local_controller))
                return null;
            return screen_info.local_player;
        }

        private void initialise_ECU()
        {
            bool CoT_mode_on = false, landing_mode_on = false, rotational_damping_on = true, use_individual_calibration = false;

            _ECU = new engine_control_unit(_grid);
            foreach (var cur_controller in _ship_controllers)
            {
                string controller_data = ((IMyTerminalBlock) cur_controller).CustomData;
                CoT_mode_on                = controller_data.ContainsCOTTag();
                rotational_damping_on      = controller_data.ContainsDAMPINGTag();
                use_individual_calibration = controller_data.ContainsICTag();
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
            _ECU.secondary_ECU              = _is_secondary;
        }

        private void set_rotational_damping(bool new_state, bool set_secondary = false)
        {
            IMyTerminalBlock controller_terminal;

            if (_ECU == null || !set_secondary && _is_secondary)
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

            if (_ECU == null || !set_secondary && _is_secondary)
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
                    _ship_controllers.Add(controller);
                    _session_ref.sample_controller(ship_controller);
                    if (_ECU != null)
                    {
                        var controller_terminal = (IMyTerminalBlock) controller;

                        if (_ECU.CoT_mode_on)
                            controller_terminal.CustomData = controller_terminal.CustomData.AddCOTTag();
                        if (_ECU.landing_mode_on)
                            controller_terminal.CustomData = controller_terminal.CustomData.AddLANDINGTag();
                        if (!_ECU.rotational_damping_on)
                            controller_terminal.CustomData = controller_terminal.CustomData.RemoveDAMPINGTag();
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
                        initialise_ECU();
                    _ECU.assign_thruster(thruster);
                    _session_ref.sample_thruster(thruster);
                    thruster.AppendingCustomInfo += thruster_tagger.show_thrust_limit;
                    ++_num_thrusters;
                    //log_grid_action("on_block_added", thruster.EntityId.ToString());
                    return;
                }

                var gyro = entity as IMyGyro;
                if (gyro != null)
                {
                    if (_ECU == null)
                        initialise_ECU();
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
                    _ship_controllers.Remove(controller);

                    var RC_block = entity as IMyRemoteControl;
                    if (RC_block != null)
                        _RC_blocks.Remove(RC_block);
                    return;
                }

                if (_ECU != null)
                {
                    var thruster = entity as IMyThrust;
                    if (thruster != null)
                    {
                        thruster.AppendingCustomInfo -= thruster_tagger.show_thrust_limit;
                        _ECU.dispose_thruster(thruster);
                        --_num_thrusters;
                        //log_grid_action("on_block_removed", thruster.EntityId.ToString());
                        return;
                    }

                    var gyro = entity as IMyGyro;
                    if (gyro != null)
                        _ECU.dispose_gyroscope(gyro);
                }
            }
        }

        /*
        private void display_thrust_reduction(int thrust_reduction)
        {
            if (_thrust_redction_text == null)
                return;

            if (_is_secondary || _secondary_grids != null || thrust_reduction < sync_helper.min_displayed_reduction || !sync_helper.show_thrust_reduction)
            {
                if (_thrust_redction_is_visible)
                    _thrust_redction_text.Hide();
                _thrust_redction_is_visible = false;
            }
            else
            {
                _thrust_redction_text.Text = "Thrust loss: " + thrust_reduction.ToString() + " %";
                _thrust_redction_text.Font = (thrust_reduction > 30) ? MyFontEnum.Red : MyFontEnum.White;
                if (!_thrust_redction_is_visible)
                    _thrust_redction_text.Show();
                _thrust_redction_is_visible = true;
            }
        }

        private void display_control_warning(bool is_warning_on)
        {
            if (_control_warning_text == null)
                return;

            if (!is_warning_on || _is_secondary || _secondary_grids != null)
            {
                if (_control_limit_is_visible)
                    _control_warning_text.Hide();
            }
            else
            {
                if (!_control_limit_is_visible)
                    _control_warning_text.Show();
            }
            _control_limit_is_visible = is_warning_on;
        }
        */

        internal static void control_warning_handler(object entity, byte[] argument)
        {
            var instance = entity as grid_logic;
            if (instance == null || instance._disposed || instance._ECU == null)
                return;

            if (instance._ECU.is_under_control_of(screen_info.local_controller))
                screen_info.set_control_loss_warning_visibility(argument[0] != 0 && !instance._is_secondary && instance._secondary_grids == null);
        }

        internal static void thrust_reduction_handler(object entity, byte[] argument)
        {
            var instance = entity as grid_logic;
            if (instance == null || instance._disposed || instance._ECU == null)
                return;

            //instance.log_grid_action("thrust_reduction_handler", string.Format("TL = {0}", argument[0]));
            if (argument[0] <= 100 && instance._ECU.is_under_control_of(screen_info.local_controller))
                screen_info.set_displayed_thrust_reduction(argument[0], !instance._is_secondary && instance._secondary_grids == null);
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

        internal static void I_terms_handler(object entity, byte[] argument)
        {
            if (MyAPIGateway.Multiplayer == null || MyAPIGateway.Multiplayer.IsServer)
                return;

            var instance = entity as grid_logic;
            if (instance == null || instance._disposed || instance._ECU == null)
                return;

            engine_control_unit current_ECU = instance._ECU;
            current_ECU.current_trim    = structurise_vector(argument, 0);
            current_ECU.last_trim       = structurise_vector(argument, 6);
            current_ECU.linear_integral = structurise_vector(argument, 12);
            //instance.log_grid_action("I_terms_handler", string.Format("CT = {0}; LT = {1}; LI = {2}", current_ECU.current_trim, current_ECU.last_trim, current_ECU.linear_integral));
        }

        #endregion

        #region event triggers

        private void send_control_limit_message(IMyPlayer controlling_player)
        {
            if (_ECU != null && controlling_player != null && (controlling_player != _prev_player || _control_limit_reached != _ECU.control_limit_reached))
            {
                __message[0] = (byte) (_ECU.control_limit_reached ? (~0) : 0);
                sync_helper.send_message_to(controlling_player.SteamUserId, sync_helper.message_types.CONTROL_LIMIT, this, __message, 1);
                _control_limit_reached = _ECU.control_limit_reached;
            }
        }

        private void send_thrust_reduction_message(IMyPlayer controlling_player)
        {
            if (_ECU != null && controlling_player != null && (controlling_player != _prev_player || _prev_thrust_reduction != _ECU.thrust_reduction))
            {
                _prev_thrust_reduction = _ECU.thrust_reduction;
                __message[0]           = (byte) _prev_thrust_reduction;
                if (__message[0] > 100)
                    __message[0] = 100;
                //log_grid_action("send_thrust_reduction_message", string.Format("TL = {0}", __message[0]));
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
            if (_ECU != null && (_ECU.current_trim != _prev_trim || _ECU.last_trim != _prev_last_trim || _ECU.linear_integral != _prev_linear_integral))
            {
                _prev_trim            = _ECU.current_trim;
                _prev_last_trim       = _ECU.last_trim;
                _prev_linear_integral = _ECU.linear_integral;
                serialise_vector(_prev_trim           , __message, 0);
                serialise_vector(_prev_last_trim      , __message, 6);
                serialise_vector(_prev_linear_integral, __message, 12);
                //log_grid_action("send_I_terms_message", string.Format("CT = {0}; LT = {1}; LI = {2}", _prev_trim, _prev_last_trim, _prev_linear_integral));
                sync_helper.send_message_to_others(sync_helper.message_types.I_TERMS, this, __message, 18);
            }
        }

        #endregion

        private void handle_user_input(IMyControllableEntity controller)
        {
            if (_ECU == null)
                return;

            /*
            Vector3 manual_thrust = Vector3.Zero, manual_rotation;

            if (sync_helper.is_spectator_mode_on || MyAPIGateway.Gui.GetCurrentScreen != MyTerminalPageEnum.None || MyAPIGateway.Gui.ChatEntryVisible)
                manual_rotation = Vector3.Zero;
            else
            {
                if (MyAPIGateway.Input.IsGameControlPressed(MyControlsSpace.LOOKAROUND))
                    manual_rotation = Vector3.Zero;
                else
                {
                    Vector2 pitch_yaw = MyAPIGateway.Input.GetRotation();
                    manual_rotation.X = pitch_yaw.X;
                    manual_rotation.Y = pitch_yaw.Y;
                    manual_rotation.Z = MyAPIGateway.Input.GetRoll();
                }

                if (MyAPIGateway.Input.IsGameControlPressed(MyControlsSpace.FORWARD))
                    manual_thrust += Vector3.Forward;
                if (MyAPIGateway.Input.IsGameControlPressed(MyControlsSpace.BACKWARD))
                    manual_thrust += Vector3.Backward;
                if (MyAPIGateway.Input.IsGameControlPressed(MyControlsSpace.STRAFE_LEFT))
                    manual_thrust += Vector3.Left;
                if (MyAPIGateway.Input.IsGameControlPressed(MyControlsSpace.STRAFE_RIGHT))
                    manual_thrust += Vector3.Right;
                if (MyAPIGateway.Input.IsGameControlPressed(MyControlsSpace.JUMP))
                    manual_thrust += Vector3.Up;
                if (MyAPIGateway.Input.IsGameControlPressed(MyControlsSpace.CROUCH))
                    manual_thrust += Vector3.Down;
            }
            */

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

            //send_linear_message  (manual_thrust  );
            //send_rotation_message(manual_rotation);
            //_ECU.translate_linear_input  (manual_thrust  , controller);
            //_ECU.translate_rotation_input(manual_rotation, controller);
            _ECU.translate_player_input(manual_thrust, manual_rotation, controller);
            //_zero_controls_counter = 0;
        }

        public void handle_60Hz()
        {
            check_disposed();
            if (!_grid.IsStatic && _ECU != null && (_num_thrusters > 0 || _secondary_grids != null))
            {
                lock (_ECU)
                {
                    if (!_is_secondary)
                    {
                        IMyPlayer controlling_player = get_controlling_player();
                        if (controlling_player == null)
                        {
                            _ECU.reset_user_input(reset_gyros_only: false);
                            //_prev_manual_thrust = _prev_manual_rotation = new Vector3UByte(128, 128, 128);
                            if (_secondary_grids != null)
                            {
                                foreach (var cur_secondary in _secondary_grids)
                                {
                                    if (cur_secondary._ECU != null)
                                        cur_secondary._ECU.reset_user_input(reset_gyros_only: false);
                                }
                            }
                        }
                        else //if (!sync_helper.network_handlers_registered || MyAPIGateway.Multiplayer == null || !MyAPIGateway.Multiplayer.IsServer || MyAPIGateway.Multiplayer.IsServerPlayer(controlling_player.Client))
                            handle_user_input(controlling_player.Controller.ControlledEntity);

                        _ID_on = true;
                        foreach (var cur_controller in _ship_controllers)
                        {

                            _ID_on = cur_controller.EnabledDamping;
                            break;
                        }

                        Vector3D world_linear_velocity, world_angular_velocity;
                        Vector3  linear_control, rotation_control, gyro_override;
                        bool     gyro_override_active;
                        if (_secondary_grids != null)
                        {
                            _ECU.get_primary_control_parameters(out world_linear_velocity, out world_angular_velocity, 
                                out linear_control, out rotation_control, out gyro_override_active, out gyro_override);
                            foreach (var cur_secondary in _secondary_grids)
                            {
                                cur_secondary._ID_on = _ID_on;
                                if (cur_secondary._ECU != null)
                                {
                                    cur_secondary._ECU.set_secondary_control_parameters(world_linear_velocity, world_angular_velocity, 
                                        linear_control, rotation_control, gyro_override_active, gyro_override);
                                }
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
            {
                _session_ref.get_secondary_grids(_grid, ref _secondary_grids);
                if (_secondary_grids != null && _ECU == null)
                    initialise_ECU();
            }

            if (_ECU == null)
                return;
            lock (_ECU)
            { 
                if (_grid.IsStatic || (_num_thrusters == 0 && _secondary_grids == null))
                    _ECU.reset_ECU();
                else
                {
                    _ECU.handle_4Hz_foreground();

                    IMyPlayer controlling_player = get_controlling_player();

                    _ECU.autopilot_on = false;
                    foreach (var cur_RC_block in _RC_blocks)
                        _ECU.check_autopilot(cur_RC_block);

                    /*
                    if (sync_helper.local_player != null)
                    {
                        bool display_notification = _ECU.active_control_enabled && _ECU.is_under_control_of(sync_helper.local_controller);
                        if (_status_shown && !display_notification)
                            _status_shown = false;
                        else if (display_notification && _ID_on && (!_status_shown || _ECU.landing_mode_on != _was_in_landing_mode) && MyAPIGateway.Utilities != null)
                        {
                            MyAPIGateway.Utilities.ShowNotification(_ECU.landing_mode_on ? "Landing mode engaged" : "Flight mode engaged");
                            _status_shown = true;
                        }
                    }
                    */

                    if (_ECU.is_under_control_of(screen_info.local_controller))
                        screen_info.set_displayed_vertical_speed(_ECU.vertical_speed, !_is_secondary);

                    if (MyAPIGateway.Multiplayer == null || MyAPIGateway.Multiplayer.IsServer)
                    {
                        //send_control_modes_message(force_send: false);
                        send_control_limit_message   (controlling_player);
                        send_thrust_reduction_message(controlling_player);
                    }
                    _was_in_landing_mode = _ECU.landing_mode_on;
                    _prev_player         = controlling_player;
                }
            }
        }

        public void handle_4Hz_background()
        {
            check_disposed();
            if (_ECU == null)
                return;
            lock (_ECU)
            {
                if (!_grid.IsStatic && (_num_thrusters > 0 || _secondary_grids != null))
                    _ECU.handle_4Hz_background();
            }
        }

        public void handle_2s_period_foreground()
        {
            check_disposed();

            if (!_grid.IsStatic && _ECU != null && (_num_thrusters > 0 || _secondary_grids != null))
            {
                lock (_ECU)
                { 
                    _ECU.handle_2s_period_foreground();
                    if (MyAPIGateway.Multiplayer != null && MyAPIGateway.Multiplayer.IsServer)
                        send_I_terms_message();

                    /*
                    if (MyAPIGateway.Multiplayer != null && !MyAPIGateway.Multiplayer.IsServer)
                    {
                        //if (!_announced)
                        //    send_client_announce();
                        _prev_manual_rotation = _prev_manual_thrust = new Vector3UByte(128, 128, 128);
                    }
                    else if (_zero_controls_counter++ >= CONTROLS_TIMEOUT)
                    {
                        _ECU.reset_user_input(reset_gyros_only: false);
                        _prev_manual_rotation  = _prev_manual_thrust = new Vector3UByte(128, 128, 128);
                        _zero_controls_counter = 0;
                    }
                    */
                }
            }
        }

        public void handle_2s_period_background()
        {
            if (!_grid.IsStatic && _ECU != null && (_num_thrusters > 0 || _secondary_grids != null))
            {
                lock (_ECU)
                {
                    _ECU.handle_2s_period_background();
                }
            }
        }

        public void perform_individual_calibration()
        {
            if (!_grid.IsStatic && _ECU != null && _num_thrusters > 0)
                _ECU.perform_individual_calibration();
        }

        public grid_logic(IMyCubeGrid new_grid, session_handler session_instance)
        {
            _session_ref          = session_instance;
            _grid                 = new_grid;
            _grid.OnBlockAdded   += on_block_added;
            _grid.OnBlockRemoved += on_block_removed;
            //_ID_on = ((MyObjectBuilder_CubeGrid) _grid.GetObjectBuilder()).DampenersEnabled;
            //log_grid_action(".ctor", "");
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

            //log_grid_action(".ctor", "finished");
        }

        public void Dispose()
        {
            if (!_disposed)
            {
                //log_grid_action("Dispose", "");
                _grid.OnBlockAdded   -= on_block_added;
                _grid.OnBlockRemoved -= on_block_removed;
                sync_helper.deregister_entity(_grid.EntityId);

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
                //log_grid_action("Dispose", "finished");
            }
        }
    }
}
