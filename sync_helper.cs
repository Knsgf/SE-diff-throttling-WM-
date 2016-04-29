using System.Collections.Generic;

using Sandbox.Engine.Utils;
//using Sandbox.Game;
using Sandbox.ModAPI;
using VRage;
using VRage.Game.ModAPI;
using VRage.Game.ModAPI.Interfaces;
//using VRage.ModAPI;

namespace ttdtwm
{
    static class sync_helper
    {
        internal const ushort LINEAR_MESSAGE_ID   = 17370;
        internal const ushort ROTATION_MESSAGE_ID = 17371;
        internal const ushort SHORT_MESSAGE_ID    = 17372;

        private static Dictionary<long, grid_logic> entities = new Dictionary<long, grid_logic>();
        //private static bool F8_pressed = false;

        public static bool network_handlers_registered { get; private set; }
        public static bool        is_spectator_mode_on { get; private set; }

        public static IMyPlayer local_player                 { get; private set; }
        public static IMyControllableEntity local_controller { get; private set; }

        public static void try_register_handlers()
        {
            if (!network_handlers_registered && MyAPIGateway.Multiplayer != null)
            {
                if (MyAPIGateway.Multiplayer.IsServer)
                {
                    MyAPIGateway.Multiplayer.RegisterMessageHandler(  LINEAR_MESSAGE_ID, grid_logic.linear_message_handler  );
                    MyAPIGateway.Multiplayer.RegisterMessageHandler(ROTATION_MESSAGE_ID, grid_logic.rotation_message_handler);
                }
                MyAPIGateway.Multiplayer.RegisterMessageHandler(SHORT_MESSAGE_ID, grid_logic.short_message_handler);
                network_handlers_registered = true;
            }
        }

        public static void deregister_handlers()
        {
            if (!network_handlers_registered)
                return;
            if (MyAPIGateway.Multiplayer.IsServer)
            {
                MyAPIGateway.Multiplayer.UnregisterMessageHandler(  LINEAR_MESSAGE_ID, grid_logic.linear_message_handler  );
                MyAPIGateway.Multiplayer.UnregisterMessageHandler(ROTATION_MESSAGE_ID, grid_logic.rotation_message_handler);
            }
            MyAPIGateway.Multiplayer.UnregisterMessageHandler(SHORT_MESSAGE_ID, grid_logic.short_message_handler);
        }

        public static void register_logic_object(grid_logic obj, long entity_id)
        {
            entities.Add(entity_id, obj);
        }

        public static void deregister_logic_object(long entity_id)
        {
            entities.Remove(entity_id);
        }

        public static void encode_entity_id(IMyCubeGrid entity, byte[] message)
        {
            long entity_id = entity.EntityId;
            for (int cur_byte = 0; cur_byte < 8; ++cur_byte)
            {
                message[cur_byte] = (byte)(entity_id & 0xFF);
                entity_id >>= 8;
            }
        }

        public static grid_logic decode_entity_id(byte[] message)
        {
            long entity_id = 0;
            for (int cur_byte = 7; cur_byte >= 0; --cur_byte)
                entity_id = (entity_id << 8) | message[cur_byte];
            return entities.ContainsKey(entity_id) ? entities[entity_id] : null;
        }

        public static void handle_60Hz()
        {
            /*
            if (MyAPIGateway.Session.SessionSettings.EnableSpectator && MyAPIGateway.Input != null)
            {
                if (MyAPIGateway.Input.IsGameControlPressed(MyControlsSpace.SPECTATOR_FREE))
                    F8_pressed = true;
                else if (MyAPIGateway.Input.IsGameControlPressed(MyControlsSpace.SPECTATOR_NONE)
                         || MyAPIGateway.Input.IsGameControlPressed(MyControlsSpace.SPECTATOR_DELTA)
                         || MyAPIGateway.Input.IsGameControlPressed(MyControlsSpace.SPECTATOR_STATIC))
                {
                    F8_pressed = false;
                }
            }
            */

            is_spectator_mode_on = false;
            if (MyAPIGateway.Session.SessionSettings.EnableSpectator)
            {
                var spectator_controller = MyAPIGateway.Session.CameraController as MySpectatorCameraController;
                if (spectator_controller != null)
                    is_spectator_mode_on = spectator_controller.SpectatorCameraMovement == MySpectatorCameraMovementEnum.UserControlled;
            }
            //is_spectator_mode_on = F8_pressed && MyAPIGateway.Session != null && MyAPIGateway.Session.SessionSettings.EnableSpectator 
            //    && !(MyAPIGateway.Session.CameraController is IMyEntity || MyAPIGateway.Session.CameraController is IMyPlayer);
            /*
            if (MyAPIGateway.Utilities != null)
            {
                MyAPIGateway.Utilities.ShowNotification("Spectator free = " + is_spectator_mode_on.ToString(), 16);
            }
            */

            local_player = MyAPIGateway.Session.LocalHumanPlayer;
            local_controller = (local_player == null) ? null : local_player.Controller.ControlledEntity;
        }
    }
}
