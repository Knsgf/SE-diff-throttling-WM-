using System.Collections.Generic;

using Sandbox.Game;
using Sandbox.Game.Gui;
using Sandbox.ModAPI;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.ModAPI;
using VRage.Game.ModAPI.Interfaces;
using VRage.Input;
using VRage.ModAPI;
using VRage.ObjectBuilders;
using VRage.Utils;
using VRageMath;

namespace ttdtwm
{
    [MySessionComponentDescriptor(MyUpdateOrder.NoUpdate)]
    public class session_handler: MySessionComponentBase
    {
        protected override void UnloadData()
        {
            base.UnloadData();
            sync_helper.deregister_handlers();
            MyLog.Default.WriteLine("TTDTWM\t all unloaded");
        }
    }

    [MyEntityComponentDescriptor(typeof(MyObjectBuilder_CubeGrid))]
    public class grid_logic: MyGameLogicComponent
    {
        const float  MESSAGE_MULTIPLIER          = 10.0f;

        private static byte[]  long_message = new byte[8 + 3];
        private static byte[] short_message = new byte[8 + 1];

        private IMyCubeGrid _grid;
        private HashSet<IMyControllableEntity> _ship_controllers;
        private engine_control_unit ECU;
        private int _num_thrusters, prev_thrust_reduction;
        private bool _ID_on;
        private Vector3UByte prev_manual_thrust, prev_manual_rotation;
        private IMyHudNotification thrust_redction_text, control_warning_text;
        private bool control_limit_is_visible, thrust_reduction_is_visible;

        private void on_block_added(IMySlimBlock block)
        {
            IMyCubeBlock entity = block.FatBlock;

            if (entity != null)
            {
                var controller = entity as IMyControllableEntity;
                if (controller != null)
                {
                    _ship_controllers.Add(controller);
                    return;
                }

                var thruster = entity as IMyThrust;
                if (thruster != null)
                {
                    if (ECU == null)
                        ECU = new engine_control_unit(_grid);
                    ECU.assign_thruster(thruster);
                    ++_num_thrusters;
                }
            }
        }

        private void on_block_removed(IMySlimBlock block)
        {
            IMyCubeBlock entity = block.FatBlock;

            if (entity != null)
            {
                var controller = entity as IMyControllableEntity;
                if (controller != null)
                {
                    _ship_controllers.Remove(controller);
                    return;
                }

                var thruster = entity as IMyThrust;
                if (thruster != null)
                {
                    ECU.dispose_thruster(thruster);
                    --_num_thrusters;
                }
            }
        }

        internal static void display_thrust_reduction(byte[] argument)
        {
            grid_logic instance = sync_helper.decode_entity_id(argument);
            if (instance.thrust_redction_text == null)
                return;
            int thrust_reduction = argument[8];
            if (thrust_reduction < 5)
            {
                instance.thrust_redction_text.Hide();
            }
            else
            {
                instance.thrust_redction_text.Text = "Thrust reduction: " + thrust_reduction.ToString() + " %";
                instance.thrust_redction_text.Font = (thrust_reduction > 30) ? MyFontEnum.Red : MyFontEnum.White;
                instance.thrust_redction_text.Show();
            }
        }

        internal static void display_control_warning(byte[] argument)
        {
            grid_logic instance = sync_helper.decode_entity_id(argument);
            if (instance.control_warning_text == null)
                return;
            if (argument[8] == 0)
                instance.control_warning_text.Hide();
            else
                instance.control_warning_text.Show();
        }

        internal static void linear_message_handler(byte[] argument)
        {
            grid_logic instance = sync_helper.decode_entity_id(argument);
            Vector3    manual_thrust;
            IMyPlayer  controlling_player = MyAPIGateway.Multiplayer.Players.GetPlayerControllingEntity(instance._grid);
            if (controlling_player == null || instance.ECU == null || !MyAPIGateway.Multiplayer.IsServer)
                return;
            manual_thrust.X = (argument[ 8] - 128.0f) / MESSAGE_MULTIPLIER;
            manual_thrust.Y = (argument[ 9] - 128.0f) / MESSAGE_MULTIPLIER;
            manual_thrust.Z = (argument[10] - 128.0f) / MESSAGE_MULTIPLIER;
            instance.ECU.translate_linear_input(manual_thrust, controlling_player.Controller.ControlledEntity);
        }

        internal static void rotation_message_handler(byte[] argument)
        {
            grid_logic instance = sync_helper.decode_entity_id(argument);
            Vector3    manual_rotation;
            IMyPlayer  controlling_player = MyAPIGateway.Multiplayer.Players.GetPlayerControllingEntity(instance._grid);
            if (controlling_player == null || instance.ECU == null || !MyAPIGateway.Multiplayer.IsServer)
                return;
            manual_rotation.X = (argument[ 8] - 128.0f) / MESSAGE_MULTIPLIER;
            manual_rotation.Y = (argument[ 9] - 128.0f) / MESSAGE_MULTIPLIER;
            manual_rotation.Z = (argument[10] - 128.0f) / MESSAGE_MULTIPLIER;
            instance.ECU.translate_rotation_input(manual_rotation, controlling_player.Controller.ControlledEntity);
        }

        private void send_linear_message(Vector3 manual_thrust)
        {
            if (MyAPIGateway.Multiplayer == null /*|| MyAPIGateway.Multiplayer.IsServer*/)
                return;
            Vector3UByte packed_vector = Vector3UByte.Round(manual_thrust * MESSAGE_MULTIPLIER + Vector3.One * 128.0f);
            if (packed_vector == prev_manual_thrust)
                return;
            sync_helper.encode_entity_id(_grid, long_message);
            long_message[ 8] = packed_vector.X;
            long_message[ 9] = packed_vector.Y;
            long_message[10] = packed_vector.Z;
            prev_manual_thrust = packed_vector;
            MyAPIGateway.Multiplayer.SendMessageToServer(sync_helper.LINEAR_MESSAGE_ID, long_message);
        }

        private void send_rotation_message(Vector3 manual_rotation)
        {
            if (MyAPIGateway.Multiplayer == null /*|| MyAPIGateway.Multiplayer.IsServer*/)
                return;
            Vector3UByte packed_vector = Vector3UByte.Round(manual_rotation * MESSAGE_MULTIPLIER + Vector3.One * 128.0f);
            if (packed_vector == prev_manual_rotation)
                return;
            sync_helper.encode_entity_id(_grid, long_message);
            long_message[ 8] = packed_vector.X;
            long_message[ 9] = packed_vector.Y;
            long_message[10] = packed_vector.Z;
            prev_manual_rotation = packed_vector;
            MyAPIGateway.Multiplayer.SendMessageToServer(sync_helper.ROTATION_MESSAGE_ID, long_message);
        }

        private void send_control_limit_message()
        {
            if (ECU == null || MyAPIGateway.Multiplayer == null || !MyAPIGateway.Multiplayer.IsServer)
                return;

            IMyPlayer controlling_player = MyAPIGateway.Multiplayer.Players.GetPlayerControllingEntity(_grid);
            if (controlling_player != null && control_limit_is_visible != ECU.control_limit_reached)
            {
                sync_helper.encode_entity_id(_grid, short_message);
                short_message[8] = (byte) (ECU.control_limit_reached ? 1 : 0);
                control_limit_is_visible = ECU.control_limit_reached;
                MyAPIGateway.Multiplayer.SendMessageTo(sync_helper.CONTROL_LIMIT_MESSAGE_ID, short_message, controlling_player.SteamUserId);
            }
        }

        private void send_thrust_reduction_message()
        {
            if (ECU == null || MyAPIGateway.Multiplayer == null || !MyAPIGateway.Multiplayer.IsServer)
                return;

            IMyPlayer controlling_player = MyAPIGateway.Multiplayer.Players.GetPlayerControllingEntity(_grid);
            if (controlling_player != null && prev_thrust_reduction != ECU.thrust_reduction)
            {
                sync_helper.encode_entity_id(_grid, short_message);
                short_message[8] = (byte) ECU.thrust_reduction;
                prev_thrust_reduction = ECU.thrust_reduction;
                //display_thrust_redction(short_message);
                MyAPIGateway.Multiplayer.SendMessageTo(sync_helper.THRUST_REDUCTION_MESSAGE_ID, short_message, controlling_player.SteamUserId);
            }
        }

        private void handle_user_input(IMyControllableEntity controller)
        {
            Vector3 manual_thrust = Vector3.Zero, manual_rotation;

            if (ECU == null)
                return;
            if (MyGuiScreenTerminal.GetCurrentScreen() != MyTerminalPageEnum.None)
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
            //ECU.translate_user_input(manual_thrust, manual_rotation, controller);
            send_linear_message  (  manual_thrust);
            send_rotation_message(manual_rotation);
        }

        public override void Init(MyObjectBuilder_EntityBase object_builder)
        {
            base.Init(object_builder);
            Entity.NeedsUpdate      |= MyEntityUpdateEnum.EACH_FRAME | MyEntityUpdateEnum.EACH_10TH_FRAME | MyEntityUpdateEnum.EACH_100TH_FRAME;
            _grid                    = (IMyCubeGrid) Entity;
            _grid.OnBlockAdded      += on_block_added;
            _grid.OnBlockRemoved    += on_block_removed;
            _ship_controllers        = new HashSet<IMyControllableEntity>();
            _num_thrusters           = prev_thrust_reduction = 0;
            _ID_on                   = ((MyObjectBuilder_CubeGrid) object_builder).DampenersEnabled;
            prev_manual_thrust       = prev_manual_rotation = new Vector3UByte(128, 128, 128);
            control_limit_is_visible = false;
            //thrust_redction_text     = MyAPIGateway.Utilities.CreateNotification("", 0);
            //control_warning_text     = MyAPIGateway.Utilities.CreateNotification("WARNING: Control limit reached", 0, MyFontEnum.Red);
            sync_helper.register_logic_object(this, _grid.EntityId);
        }

        public override void Close()
        {
            base.Close();
            MyLog.Default.WriteLine(string.Format("TTDTWM\t \"{0}\" closed", _grid.DisplayName));
            _grid.OnBlockAdded   -= on_block_added;
            _grid.OnBlockRemoved -= on_block_removed;
            sync_helper.deregister_logic_object(_grid.EntityId);
        }
        
        public override void UpdateAfterSimulation()
        {
            base.UpdateAfterSimulation();
            //MyAPIGateway.Utilities.ShowNotification("UpdateAfterSimulation", 16);
            if (!_grid.IsStatic && _num_thrusters > 0)
            {
                if (MyAPIGateway.Multiplayer == null)
                    handle_user_input(MyAPIGateway.Session.ControlledObject);
                else
                {
                    IMyPlayer controlling_player = MyAPIGateway.Multiplayer.Players.GetPlayerControllingEntity(_grid);
                    if (controlling_player == null)
                        ECU.reset_user_input();
                    else if (!sync_helper.network_handlers_registered || !MyAPIGateway.Multiplayer.IsServer || MyAPIGateway.Multiplayer.IsServerPlayer(controlling_player.Client))
                        handle_user_input(controlling_player.Controller.ControlledEntity);
                }

                foreach (var cur_controller in _ship_controllers)
                {
                    _ID_on = cur_controller.EnabledDamping;
                    break;
                }
                ECU.linear_dampers_on = _ID_on;
                ECU.handle_60Hz();
            }
        }

        public override void UpdateAfterSimulation10()
        {
            base.UpdateAfterSimulation10();
            //MyAPIGateway.Utilities.ShowNotification("UpdateAfterSimulation10", 160);
            if (sync_helper.network_handlers_registered)
            {
                send_control_limit_message();
                send_thrust_reduction_message();
            }
            if (!_grid.IsStatic && _num_thrusters > 0)
                ECU.handle_4Hz();
        }

        public override void UpdateAfterSimulation100()
        {
            base.UpdateAfterSimulation100();
            //var controller = MyAPIGateway.Session.ControlledObject as MyShipController;
            //MyAPIGateway.Utilities.ShowNotification(string.Format("UpdateAfterSimulation100: {0}, ID: {1}", controller, (controller == null) ? "N/A" : ((PB.IMyShipController) controller).DampenersOverride.ToString()), 1600);
            //var player = MyAPIGateway.Multiplayer.Players.GetPlayerControllingEntity(_grid);
            //var controller = (player == null) ? null : (player.Controller.ControlledEntity as MyShipController);
            //MyAPIGateway.Utilities.ShowNotification(string.Format("UpdateAfterSimulation100 = \"{0}\": {1}, ID: {2}, MC: {3}", _grid.DisplayName, (player == null) ? "none" : player.DisplayName, (controller == null) ? "N/A" : ((PB.IMyShipController) controller).DampenersOverride.ToString(), _grid.HasMainCockpit() ? "set" : "none"), 1600);
            //MyAPIGateway.Utilities.ShowNotification(string.Format("UpdateAfterSimulation100: {0}", (player == null) ? "none" : player.DisplayName), 1600);
            //MyAPIGateway.Utilities.ShowNotification(string.Format("UpdateAfterSimulation100: {0} {1}", MyAPIGateway.Input.GetRotation(), MyAPIGateway.Input.GetRoll()), 1600);
            /*
            foreach (var cur_controller in _ship_controllers)
            {
                _ID_on = cur_controller.EnabledDamping;
                break;
            }
            */
            //MyAPIGateway.Utilities.ShowNotification(string.Format("UpdateAfterSimulation100 = \"{0}\": ID: {1}, MC: {2}", _grid.DisplayName, _ID_on, ((MyCubeGrid) _grid).HasMainCockpit() ? "set" : "none"), 1600);
            if (!_grid.IsStatic && _num_thrusters > 0)
            {
                if (!sync_helper.network_handlers_registered)
                    sync_helper.try_register_handlers();
                if (control_warning_text == null && MyAPIGateway.Utilities != null)
                {
                    thrust_redction_text = MyAPIGateway.Utilities.CreateNotification("", 0);
                    control_warning_text = MyAPIGateway.Utilities.CreateNotification("WARNING: Control limit reached", 0, MyFontEnum.Red);
                }
                ECU.handle_2s_period();
            }
        }

        /*
        public override void UpdateBeforeSimulation()
        {
            base.UpdateBeforeSimulation();
            MyAPIGateway.Utilities.ShowNotification("UpdateBeforeSimulation", 16);
        }

        
        public override void UpdateBeforeSimulation10()
        {
            base.UpdateBeforeSimulation10();
            MyAPIGateway.Utilities.ShowNotification("UpdateBeforeSimulation10", 100);
        }

        public override void UpdateBeforeSimulation100()
        {
            base.UpdateBeforeSimulation100();
            MyAPIGateway.Utilities.ShowNotification("UpdateBeforeSimulation100", 1000);
        }
        */

        public override MyObjectBuilder_EntityBase GetObjectBuilder(bool copy = false)
        {
            return Entity.GetObjectBuilder(copy);
        }
    }
}
