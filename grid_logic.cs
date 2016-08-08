using System;
using System.Collections.Generic;
using System.Text;

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

        const float MESSAGE_MULTIPLIER = 10.0f, MESSAGE_SHIFT = 128.0f;
        const int   CONTROL_WARNING_OFF = 200, CONTROL_WARNING_ON = 201/*, CLIENT_ANNOUNCE = 202, SERVER_ACKNOWLEDGE = 203*/;
        //const int   CONTROL_MODE_BASE = 204, LANDING_MODE_ON = 1, COT_MODE_ON = 2;  // 204 - 207
        const int   CONTROLS_TIMEOUT = 2;

        private static byte[] __long_message  = new byte[8 + 3];
        private static byte[] __short_message = new byte[8 + 1];

        private static StringBuilder __controller_name = new StringBuilder();

        private IMyCubeGrid                    _grid;
        private HashSet<IMyControllableEntity> _ship_controllers     = new HashSet<IMyControllableEntity>();
        private HashSet<IMyRemoteControl>   _RC_blocks            = new HashSet<IMyRemoteControl>();
        private IMyHudNotification             _thrust_redction_text = null, _control_warning_text = null, _vertical_speed_text = null;
        private engine_control_unit            _ECU                  = null;
        private Vector3UByte                   _prev_manual_thrust   = new Vector3UByte(128, 128, 128), _prev_manual_rotation = new Vector3UByte(128, 128, 128);
        private IMyPlayer                      _prev_player          = null;

        private int  _num_thrusters = 0, _prev_thrust_reduction = 0, _zero_controls_counter = 0;
        private bool _control_limit_is_visible = false, _thrust_redction_is_visible = false, _vertical_speed_is_visible = false, _disposed = false, _status_shown = false;
        private bool _was_in_landing_mode = false, _was_in_CoT_mode = false, _ID_on, _force_CoT_mode_on, _landing_mode_on;
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

        public bool CoT_mode_forced
        {
            get
            {
                return is_CoT_mode_available && _ECU.CoT_mode_forced;
            }
            set
            {
                IMyTerminalBlock controller_terminal;

                if (_ECU == null)
                    return;
                foreach (var cur_controller in _ship_controllers)
                {
                    controller_terminal = (IMyTerminalBlock) cur_controller;
                    controller_terminal.SetCustomName(value ? controller_terminal.CustomName.AddCOTTag() : controller_terminal.CustomName.RemoveCOTTag());
                }
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
                IMyTerminalBlock controller_terminal;

                if (_ECU == null)
                    return;
                foreach (var cur_controller in _ship_controllers)
                {
                    controller_terminal = (IMyTerminalBlock) cur_controller;
                    controller_terminal.SetCustomName(value ? controller_terminal.CustomName.AddLANDINGTag() : controller_terminal.CustomName.RemoveLANDINGTag());
                }
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
            if (_ECU == null || !_ECU.is_under_control_of(sync_helper.local_controller))
                return null;
            return sync_helper.local_player;
        }

        #endregion

        #region event handlers

        private void on_block_added(IMySlimBlock block)
        {
            check_disposed();
            IMyCubeBlock entity = block.FatBlock;
            if (entity != null)
            {
                var controller = entity as IMyControllableEntity;
                if (controller != null)
                    _ship_controllers.Add(controller);
                var RC_block = entity as IMyRemoteControl;
                if (RC_block != null)
                    _RC_blocks.Add(RC_block);

                var thruster = entity as IMyThrust;
                if (thruster != null)
                {
                    if (_ECU == null)
                        _ECU = new engine_control_unit(_grid);
                    _ECU.assign_thruster(thruster);
                    ++_num_thrusters;
                }
                var gyro = entity as IMyGyro;
                if (gyro != null)
                {
                    if (_ECU == null)
                        _ECU = new engine_control_unit(_grid);
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
                    _ship_controllers.Remove(controller);
                var RC_block = entity as IMyRemoteControl;
                if (RC_block != null)
                    _RC_blocks.Remove(RC_block);

                var thruster = entity as IMyThrust;
                if (thruster != null)
                {
                    if (_ECU != null)
                        _ECU.dispose_thruster(thruster);
                    --_num_thrusters;
                }
                if (_ECU != null)
                {
                    var gyro = entity as IMyGyro;
                    if (gyro != null)
                        _ECU.dispose_gyroscope(gyro);
                }
            }
        }

        private void display_thrust_reduction(int thrust_reduction)
        {
            if (_thrust_redction_text == null)
                return;

            if (thrust_reduction < 5)
            {
                _thrust_redction_text.Hide();
                _thrust_redction_is_visible = false;
            }
            else
            {
                _thrust_redction_text.Text = "Thrust reduction: " + thrust_reduction.ToString() + " %";
                _thrust_redction_text.Font = (thrust_reduction > 30) ? MyFontEnum.Red : MyFontEnum.White;
                _thrust_redction_text.Show();
                _thrust_redction_is_visible = true;
            }
        }

        private void display_control_warning(bool is_warning_on)
        {
            if (_control_warning_text == null)
                return;

            if (!is_warning_on)
                _control_warning_text.Hide();
            else
                _control_warning_text.Show();
            _control_limit_is_visible  = is_warning_on;
        }

        internal static void short_message_handler(byte[] argument)
        {
            grid_logic instance = sync_helper.decode_entity_id(argument);
            if (instance == null || instance._disposed || instance._ECU == null)
                return;

            int contents = argument[8];
            if (contents <= 100)
            {
                instance.display_thrust_reduction(contents);
                return;
            }
            switch (contents)
            {
                case CONTROL_WARNING_OFF:
                case CONTROL_WARNING_ON:
                    instance.display_control_warning(contents == CONTROL_WARNING_ON);
                    break;

                /*
                case CONTROL_MODE_BASE:
                case CONTROL_MODE_BASE + LANDING_MODE_ON:
                case CONTROL_MODE_BASE + COT_MODE_ON:
                case CONTROL_MODE_BASE + COT_MODE_ON + LANDING_MODE_ON:
                    int flags = contents - CONTROL_MODE_BASE;
                    instance._ECU.landing_mode_on = (flags & LANDING_MODE_ON) != 0;
                    instance._ECU.CoT_mode_forced = (flags &     COT_MODE_ON) != 0;
                    //instance.log_grid_action("short_message_handler", "landing mode = " + ((contents == LANDING_MODE_ON) ? "on" : "off"));
                    break;

                case CLIENT_ANNOUNCE:
                    instance.send_control_modes_message(force_send: true);
                    instance.send_server_acknowledge();
                    //instance.log_grid_action("short_message_handler", "announce received");
                    break;

                case SERVER_ACKNOWLEDGE:
                    instance._announced = true;
                    //instance.log_grid_action("short_message_handler", "announce complete");
                    break;
                */
            }
        }

        internal static void linear_message_handler(byte[] argument)
        {
            grid_logic instance = sync_helper.decode_entity_id(argument);
            if (instance == null || instance._disposed || instance._ECU == null)
                return;
            Vector3 manual_thrust;
            IMyPlayer controlling_player = instance.get_controlling_player();
            if (controlling_player == null || MyAPIGateway.Multiplayer != null && !MyAPIGateway.Multiplayer.IsServer)
                return;
            manual_thrust.X = (argument[ 8] - MESSAGE_SHIFT) / MESSAGE_MULTIPLIER;
            manual_thrust.Y = (argument[ 9] - MESSAGE_SHIFT) / MESSAGE_MULTIPLIER;
            manual_thrust.Z = (argument[10] - MESSAGE_SHIFT) / MESSAGE_MULTIPLIER;
            instance._ECU.translate_linear_input(manual_thrust, controlling_player.Controller.ControlledEntity);
            instance._zero_controls_counter = 0;
        }

        internal static void rotation_message_handler(byte[] argument)
        {
            grid_logic instance = sync_helper.decode_entity_id(argument);
            if (instance == null || instance._disposed || instance._ECU == null)
                return;
            Vector3 manual_rotation;
            IMyPlayer controlling_player = instance.get_controlling_player();
            if (controlling_player == null || MyAPIGateway.Multiplayer != null && !MyAPIGateway.Multiplayer.IsServer)
                return;
            manual_rotation.X = (argument[ 8] - MESSAGE_SHIFT) / MESSAGE_MULTIPLIER;
            manual_rotation.Y = (argument[ 9] - MESSAGE_SHIFT) / MESSAGE_MULTIPLIER;
            manual_rotation.Z = (argument[10] - MESSAGE_SHIFT) / MESSAGE_MULTIPLIER;
            instance._ECU.translate_rotation_input(manual_rotation, controlling_player.Controller.ControlledEntity);
            instance._zero_controls_counter = 0;
        }

        #endregion

        #region event triggers

        private void send_linear_message(Vector3 manual_thrust)
        {
            if (MyAPIGateway.Multiplayer == null || MyAPIGateway.Multiplayer.IsServer)
                return;

            Vector3UByte packed_vector = Vector3UByte.Round(manual_thrust * MESSAGE_MULTIPLIER + Vector3.One * MESSAGE_SHIFT);
            if (packed_vector == _prev_manual_thrust)
                return;
            sync_helper.encode_entity_id(_grid, __long_message);
            __long_message[ 8] = packed_vector.X;
            __long_message[ 9] = packed_vector.Y;
            __long_message[10] = packed_vector.Z;
            _prev_manual_thrust = packed_vector;
            MyAPIGateway.Multiplayer.SendMessageToServer(sync_helper.LINEAR_MESSAGE_ID, __long_message);
        }

        private void send_rotation_message(Vector3 manual_rotation)
        {
            if (MyAPIGateway.Multiplayer == null || MyAPIGateway.Multiplayer.IsServer)
                return;

            Vector3UByte packed_vector = Vector3UByte.Round(manual_rotation * MESSAGE_MULTIPLIER + Vector3.One * MESSAGE_SHIFT);
            if (packed_vector == _prev_manual_rotation)
                return;
            sync_helper.encode_entity_id(_grid, __long_message);
            __long_message[ 8] = packed_vector.X;
            __long_message[ 9] = packed_vector.Y;
            __long_message[10] = packed_vector.Z;
            _prev_manual_rotation = packed_vector;
            MyAPIGateway.Multiplayer.SendMessageToServer(sync_helper.ROTATION_MESSAGE_ID, __long_message);
        }

        private void send_control_limit_message(IMyPlayer controlling_player)
        {
            if (_ECU == null)
                return;

            if (controlling_player != null && (controlling_player != _prev_player || _control_limit_is_visible != _ECU.control_limit_reached))
            {
                sync_helper.encode_entity_id(_grid, __short_message);
                __short_message[8] = (byte) (_ECU.control_limit_reached ? CONTROL_WARNING_ON : CONTROL_WARNING_OFF);
                _control_limit_is_visible = _ECU.control_limit_reached;
                if (MyAPIGateway.Multiplayer == null || MyAPIGateway.Multiplayer.IsServerPlayer(controlling_player.Client))
                    display_control_warning(_ECU.control_limit_reached);
                else if (MyAPIGateway.Multiplayer.IsServer)
                    MyAPIGateway.Multiplayer.SendMessageTo(sync_helper.SHORT_MESSAGE_ID, __short_message, controlling_player.SteamUserId);
            }
        }

        /*
        private void send_control_modes_message(bool force_send)
        {
            if (_ECU == null || MyAPIGateway.Multiplayer == null || !MyAPIGateway.Multiplayer.IsServer)
                return;

            if (force_send || _was_in_landing_mode != _ECU.landing_mode_on || _was_in_CoT_mode != _ECU.CoT_mode_forced)
            {
                sync_helper.encode_entity_id(_grid, __short_message);
                __short_message[8] = CONTROL_MODE_BASE;
                if (_ECU.landing_mode_on)
                    __short_message[8] += LANDING_MODE_ON;
                if (_ECU.CoT_mode_forced)
                    __short_message[8] += COT_MODE_ON;
                _was_in_landing_mode = _ECU.landing_mode_on;
                _was_in_CoT_mode     = _ECU.CoT_mode_forced;
                MyAPIGateway.Multiplayer.SendMessageToOthers(sync_helper.SHORT_MESSAGE_ID, __short_message);
            }
        }

        private void send_client_announce()
        {
            if (!_announced && MyAPIGateway.Multiplayer != null && !MyAPIGateway.Multiplayer.IsServer)
            {
                sync_helper.encode_entity_id(_grid, __short_message);
                __short_message[8] = CLIENT_ANNOUNCE;
                MyAPIGateway.Multiplayer.SendMessageToServer(sync_helper.SHORT_MESSAGE_ID, __short_message);
            }
        }

        private void send_server_acknowledge()
        {
            if (MyAPIGateway.Multiplayer != null && MyAPIGateway.Multiplayer.IsServer)
            {
                sync_helper.encode_entity_id(_grid, __short_message);
                __short_message[8] = SERVER_ACKNOWLEDGE;
                MyAPIGateway.Multiplayer.SendMessageToOthers(sync_helper.SHORT_MESSAGE_ID, __short_message);
            }
        }
        */

        private void send_thrust_reduction_message(IMyPlayer controlling_player)
        {
            if (_ECU == null)
                return;

            if (controlling_player != null && (controlling_player != _prev_player || _prev_thrust_reduction != _ECU.thrust_reduction))
            {
                sync_helper.encode_entity_id(_grid, __short_message);
                __short_message[8] = (byte) _ECU.thrust_reduction;
                if (__short_message[8] > 100)
                    __short_message[8] = 100;
                _prev_thrust_reduction = __short_message[8];
                if (MyAPIGateway.Multiplayer == null || MyAPIGateway.Multiplayer.IsServerPlayer(controlling_player.Client))
                    display_thrust_reduction(__short_message[8]);
                else if (MyAPIGateway.Multiplayer.IsServer)
                    MyAPIGateway.Multiplayer.SendMessageTo(sync_helper.SHORT_MESSAGE_ID, __short_message, controlling_player.SteamUserId);
            }
        }

        #endregion

        private void handle_user_input(IMyControllableEntity controller)
        {
            if (_ECU == null)
                return;

            /*
            if (sync_helper.is_spectator_mode_on || MyGuiScreenTerminal.GetCurrentScreen() != MyTerminalPageEnum.None || MyGuiScreenGamePlay.ActiveGameplayScreen != null)
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

            Vector3 manual_thrust, manual_rotation;
            var     ship_controller = (Sandbox.Game.Entities.MyShipController) controller;

            manual_thrust     = ship_controller.MoveIndicator;
            manual_rotation.X = ship_controller.RotationIndicator.X;
            manual_rotation.Y = ship_controller.RotationIndicator.Y;
            manual_rotation.Z = ship_controller.RollIndicator;

            send_linear_message  (manual_thrust  );
            send_rotation_message(manual_rotation);
            _ECU.translate_linear_input  (manual_thrust  , controller);
            _ECU.translate_rotation_input(manual_rotation, controller);
            _zero_controls_counter = 0;
        }

        public void handle_60Hz()
        {
            check_disposed();
            if (!_grid.IsStatic && _ECU != null && _num_thrusters > 0)
            {
                IMyPlayer controlling_player = get_controlling_player();
                if (controlling_player == null)
                {
                    _ECU.reset_user_input(reset_gyros_only: false);
                    _prev_manual_thrust = _prev_manual_rotation = new Vector3UByte(128, 128, 128);
                }
                else if (!sync_helper.network_handlers_registered || MyAPIGateway.Multiplayer == null || !MyAPIGateway.Multiplayer.IsServer || MyAPIGateway.Multiplayer.IsServerPlayer(controlling_player.Client))
                    handle_user_input(controlling_player.Controller.ControlledEntity);

                _ID_on = _force_CoT_mode_on = _landing_mode_on = false;
                foreach (var cur_controller in _ship_controllers)
                {
                    ((IMyTerminalBlock) cur_controller).CustomName.ToUpperTo(__controller_name);
                    _force_CoT_mode_on = is_CoT_mode_available     && __controller_name.ContainsCOTTag();
                    _landing_mode_on   = is_landing_mode_available && __controller_name.ContainsLANDINGTag();
                    _ID_on             = cur_controller.EnabledDamping;
                    break;
                }
                _ECU.CoT_mode_forced   = _force_CoT_mode_on;
                _ECU.landing_mode_on   = _landing_mode_on;
                _ECU.linear_dampers_on = _ID_on;
                _ECU.handle_60Hz();
            }
        }

        public void handle_4Hz()
        {
            check_disposed();
            if (_ECU == null)
                return;
            if (_grid.IsStatic || _num_thrusters == 0)
                _ECU.reset_ECU();
            else
            {
                _ECU.handle_4Hz();

                IMyPlayer controlling_player = get_controlling_player();
                if (controlling_player == null)
                {
                    if (_control_limit_is_visible)
                    {
                        _control_warning_text.Hide();
                        _control_limit_is_visible = false;
                    }
                    if (_thrust_redction_is_visible)
                    {
                        _thrust_redction_text.Hide();
                        _thrust_redction_is_visible = false;
                    }
                }

                _ECU.autopilot_on = false;
                foreach (var cur_RC_block in _RC_blocks)
                    _ECU.check_autopilot(cur_RC_block);

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

                if (_vertical_speed_text != null)
                {
                    if (!_ECU.is_under_control_of(sync_helper.local_controller) || Math.Abs(_ECU.vertical_speed) < 0.1f)
                    {
                        if (_vertical_speed_is_visible)
                        {
                            _vertical_speed_text.Hide();
                            _vertical_speed_is_visible = false;
                        }
                    }
                    else
                    {
                        _vertical_speed_text.Text = string.Format("Vertical speed: {0:F1} m/s", _ECU.vertical_speed);
                        if (!_vertical_speed_is_visible)
                        {
                            _vertical_speed_text.Show();
                            _vertical_speed_is_visible = true;
                        }
                    }
                }

                if (MyAPIGateway.Multiplayer == null || MyAPIGateway.Multiplayer.IsServer)
                {
                    //send_control_modes_message(force_send: false);
                    send_control_limit_message(   controlling_player);
                    send_thrust_reduction_message(controlling_player);
                }
                _was_in_landing_mode = _ECU.landing_mode_on;
                _prev_player         = controlling_player;
            }
        }

        public void handle_2s_period()
        {
            check_disposed();
            if (!_grid.IsStatic && _ECU != null && _num_thrusters > 0)
            {
                if (_control_warning_text == null && MyAPIGateway.Utilities != null)
                {
                    _thrust_redction_text = MyAPIGateway.Utilities.CreateNotification("", 0);
                    _control_warning_text = MyAPIGateway.Utilities.CreateNotification("WARNING: Control limit reached", 0, MyFontEnum.Red);
                    _vertical_speed_text  = MyAPIGateway.Utilities.CreateNotification("", 0);
                }

                _ECU.handle_2s_period();

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
            }
        }

        public grid_logic(IMyCubeGrid new_grid)
        {
            _grid                 = new_grid;
            _grid.OnBlockAdded   += on_block_added;
            _grid.OnBlockRemoved += on_block_removed;
            _ID_on = ((MyObjectBuilder_CubeGrid) _grid.GetObjectBuilder()).DampenersEnabled;
            sync_helper.register_logic_object(this, _grid.EntityId);

            var block_list = new List<IMySlimBlock>();
            _grid.GetBlocks(block_list,
                delegate (IMySlimBlock block)
                {
                    return block.FatBlock is IMyThrust || block.FatBlock is IMyGyro;
                }
            );
            if (block_list.Count > 0)
            {
                _ECU = new engine_control_unit(_grid);
                foreach (var cur_block in block_list)
                {
                    var thruster = cur_block.FatBlock as IMyThrust;
                    var gyro     = cur_block.FatBlock as IMyGyro;
                    if (thruster != null)
                    { 
                        _ECU.assign_thruster(thruster);
                        ++_num_thrusters;
                    }
                    if (gyro != null)
                        _ECU.assign_gyroscope(gyro);
                }
            }


            block_list.Clear();
            _grid.GetBlocks(block_list,
                delegate (IMySlimBlock block)
                {
                    return block.FatBlock is IMyCockpit || block.FatBlock is IMyRemoteControl;
                }
            );
            foreach (var cur_controller in block_list)
            {
                _ship_controllers.Add((IMyControllableEntity) cur_controller.FatBlock);
                var RC_block = cur_controller.FatBlock as IMyRemoteControl;
                if (RC_block != null)
                    _RC_blocks.Add(RC_block);
            }
        }

        public void Dispose()
        {
            if (!_disposed)
            {
                _grid.OnBlockAdded   -= on_block_added;
                _grid.OnBlockRemoved -= on_block_removed;
                sync_helper.deregister_logic_object(_grid.EntityId);
                _disposed = true;
            }
        }
    }
}
