using System;
using System.Collections.Generic;
using System.Text;

using Sandbox.Game;
using Sandbox.Game.Gui;
using Sandbox.ModAPI;
using VRage.Game;
using VRage.Game.ModAPI;
using VRage.Game.ModAPI.Interfaces;
using VRage.Input;
using VRage.Utils;
using VRageMath;

using PB = Sandbox.ModAPI.Ingame;

namespace ttdtwm
{
    class grid_logic: IDisposable
    {
        #region fields

        const float MESSAGE_MULTIPLIER = 10.0f, MESSAGE_SHIFT = 128.0f, ANGULAR_VELOCITY_MULTIPLIER = 1024.0f, ANGULAR_VELOCITY_SHIFT = 32768.0f;

        private static byte[] long_message  = new byte[8 + 3];
        private static byte[] short_message = new byte[8 + 1];

        private IMyCubeGrid                    _grid;
        private HashSet<IMyControllableEntity> _ship_controllers      = new HashSet<IMyControllableEntity>();
        private HashSet<PB.IMyRemoteControl>   _RC_blocks             = new HashSet<PB.IMyRemoteControl>();
        private IMyHudNotification             _thrust_redction_text  = null, _control_warning_text = null;
        private engine_control_unit            _ECU                   = null;
        private Vector3UByte                   _prev_manual_thrust    = new Vector3UByte(128, 128, 128), _prev_manual_rotation = new Vector3UByte(128, 128, 128);

        private int  _num_thrusters = 0, _prev_thrust_reduction = 0;
        private bool _ID_on, _control_limit_is_visible = false, _thrust_redction_is_visible = false, _disposed = false, _status_shown = false, _was_in_landing_mode = false;

        #endregion

        #region auxiliaries

        private static void log_grid_action(string method_name, string message)
        {
            MyLog.Default.WriteLine(string.Format("TTDTWM\tgrid_logic.{0}(): {1}", method_name, message));
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
                var RC_block = entity as PB.IMyRemoteControl;
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
                var RC_block = entity as PB.IMyRemoteControl;
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

        internal static void display_thrust_reduction(byte[] argument)
        {
            grid_logic instance = sync_helper.decode_entity_id(argument);
            if (instance == null || instance._disposed || instance._thrust_redction_text == null)
                return;
            int thrust_reduction = argument[8];
            if (thrust_reduction < 5)
            {
                instance._thrust_redction_text.Hide();
                instance._thrust_redction_is_visible = false;
            }
            else
            {
                instance._thrust_redction_text.Text = "Thrust reduction: " + thrust_reduction.ToString() + " %";
                instance._thrust_redction_text.Font = (thrust_reduction > 30) ? MyFontEnum.Red : MyFontEnum.White;
                instance._thrust_redction_text.Show();
                instance._thrust_redction_is_visible = true;
            }
        }

        internal static void display_control_warning(byte[] argument)
        {
            grid_logic instance = sync_helper.decode_entity_id(argument);
            if (instance == null || instance._disposed || instance._control_warning_text == null)
                return;
            if (argument[8] == 0)
            {
                instance._control_warning_text.Hide();
                instance._control_limit_is_visible = false;
            }
            else
            {
                instance._control_warning_text.Show();
                instance._control_limit_is_visible = true;
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
            sync_helper.encode_entity_id(_grid, long_message);
            long_message[ 8] = packed_vector.X;
            long_message[ 9] = packed_vector.Y;
            long_message[10] = packed_vector.Z;
            _prev_manual_thrust = packed_vector;
            MyAPIGateway.Multiplayer.SendMessageToServer(sync_helper.LINEAR_MESSAGE_ID, long_message);
        }

        private void send_rotation_message(Vector3 manual_rotation)
        {
            if (MyAPIGateway.Multiplayer == null || MyAPIGateway.Multiplayer.IsServer)
                return;

            Vector3UByte packed_vector = Vector3UByte.Round(manual_rotation * MESSAGE_MULTIPLIER + Vector3.One * MESSAGE_SHIFT);
            if (packed_vector == _prev_manual_rotation)
                return;
            sync_helper.encode_entity_id(_grid, long_message);
            long_message[ 8] = packed_vector.X;
            long_message[ 9] = packed_vector.Y;
            long_message[10] = packed_vector.Z;
            _prev_manual_rotation = packed_vector;
            MyAPIGateway.Multiplayer.SendMessageToServer(sync_helper.ROTATION_MESSAGE_ID, long_message);
        }

        private void send_control_limit_message()
        {
            if (_ECU == null)
                return;

            IMyPlayer controlling_player = get_controlling_player();
            if (controlling_player != null && _control_limit_is_visible != _ECU.control_limit_reached)
            {
                sync_helper.encode_entity_id(_grid, short_message);
                short_message[8] = (byte) (_ECU.control_limit_reached ? 1 : 0);
                _control_limit_is_visible = _ECU.control_limit_reached;
                if (MyAPIGateway.Multiplayer == null || MyAPIGateway.Multiplayer.IsServerPlayer(controlling_player.Client))
                    display_control_warning(short_message);
                else if (MyAPIGateway.Multiplayer.IsServer)
                    MyAPIGateway.Multiplayer.SendMessageTo(sync_helper.CONTROL_LIMIT_MESSAGE_ID, short_message, controlling_player.SteamUserId);
            }
        }

        private void send_thrust_reduction_message()
        {
            if (_ECU == null)
                return;

            IMyPlayer controlling_player = get_controlling_player();
            if (controlling_player != null && _prev_thrust_reduction != _ECU.thrust_reduction)
            {
                sync_helper.encode_entity_id(_grid, short_message);
                short_message[8] = (byte) _ECU.thrust_reduction;
                _prev_thrust_reduction = _ECU.thrust_reduction;
                if (MyAPIGateway.Multiplayer == null || MyAPIGateway.Multiplayer.IsServerPlayer(controlling_player.Client))
                    display_thrust_reduction(short_message);
                else if (MyAPIGateway.Multiplayer.IsServer)
                    MyAPIGateway.Multiplayer.SendMessageTo(sync_helper.THRUST_REDUCTION_MESSAGE_ID, short_message, controlling_player.SteamUserId);
            }
        }

        #endregion

        private void handle_user_input(IMyControllableEntity controller)
        {
            Vector3 manual_thrust = Vector3.Zero, manual_rotation;

            if (_ECU == null)
                return;
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
            send_linear_message  (manual_thrust  );
            send_rotation_message(manual_rotation);
            _ECU.translate_linear_input  (manual_thrust  , controller);
            _ECU.translate_rotation_input(manual_rotation, controller);
        }

        public void handle_60Hz()
        {
            check_disposed();
            if (!_grid.IsStatic && _ECU != null && _num_thrusters > 0)
            {
                IMyPlayer controlling_player = get_controlling_player();
                if (controlling_player == null)
                {
                    _ECU.reset_user_input();
                    _prev_manual_thrust = _prev_manual_rotation = Vector3UByte.Zero;
                }
                else if (!sync_helper.network_handlers_registered || MyAPIGateway.Multiplayer == null || !MyAPIGateway.Multiplayer.IsServer || MyAPIGateway.Multiplayer.IsServerPlayer(controlling_player.Client))
                    handle_user_input(controlling_player.Controller.ControlledEntity);

                foreach (var cur_controller in _ship_controllers)
                {
                    _ID_on = cur_controller.EnabledDamping;
                    break;
                }
                _ECU.linear_dampers_on = _ID_on;
                _ECU.handle_60Hz();
            }
        }

        public void handle_4Hz()
        {
            check_disposed();
            if (!_grid.IsStatic && _ECU != null && _num_thrusters > 0)
            {
                _ECU.handle_4Hz();

                IMyPlayer controlling_player = get_controlling_player();
                if (controlling_player != null)
                    _ECU.select_flight_mode(controlling_player.Controller.ControlledEntity, _RC_blocks.Count > 0);
                else
                {
                    _ECU.select_flight_mode(null, _RC_blocks.Count > 0);
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

                if (sync_helper.local_player != null)
                {
                    bool display_notification = _ECU.is_under_control_of(sync_helper.local_controller);
                    if (_status_shown && !display_notification)
                        _status_shown = false;
                    else if (display_notification && _ID_on && (!_status_shown || _ECU.landing_mode_on != _was_in_landing_mode) && MyAPIGateway.Utilities != null)
                    {
                        MyAPIGateway.Utilities.ShowNotification(_ECU.landing_mode_on ? "Landing mode engaged" : "Flight mode engaged");
                        _status_shown = true;
                        _was_in_landing_mode = _ECU.landing_mode_on;
                    }
                }

                if (MyAPIGateway.Multiplayer == null || MyAPIGateway.Multiplayer.IsServer)
                {
                    send_control_limit_message();
                    send_thrust_reduction_message();
                }
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
                }

                _ECU.handle_2s_period();
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
                    return block.FatBlock is PB.IMyCockpit || block.FatBlock is PB.IMyRemoteControl;
                }
            );
            foreach (var cur_controller in block_list)
            {
                _ship_controllers.Add((IMyControllableEntity) cur_controller.FatBlock);
                var RC_block = cur_controller.FatBlock as PB.IMyRemoteControl;
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
