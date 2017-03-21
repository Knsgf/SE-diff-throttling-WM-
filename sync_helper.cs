using System;
using System.Collections.Generic;
using System.IO;

//using Sandbox.Engine.Utils;
using Sandbox.ModAPI;
//using VRage;
using VRage.Game.ModAPI;
using VRage.Game.ModAPI.Interfaces;
using VRage.Utils;

namespace ttdtwm
{
    public struct display_settings
    {
        public bool show_thrust_reduction, show_vertical_speed;
    };

    static class sync_helper
    {
        const string settings_file = "TTDTWM.CFG";

        const ushort SYNC_MESSAGE_ID = 17370;

        internal const int MAX_MESSAGE_LENGTH = 18;
        internal enum message_types { LINEAR_INPUT, ROTATION_INPUT, I_TERMS, MANUAL_THROTTLE, CONTROL_LIMIT, THRUST_LOSS };
        private static readonly int num_messages = Enum.GetValues(typeof(message_types)).Length;

        const int SIGNATURE_LENGTH = 5;
        private static Dictionary<uint, byte[]> out_buffers = new Dictionary<uint, byte[]>();
        private static byte[] in_buffer = new byte[MAX_MESSAGE_LENGTH];
        private static readonly byte[] signature = { 6, 60, 33, 39, 66 };

        private static Dictionary<  long, object> entities   = new Dictionary<  long, object>();
        private static Dictionary<object,   long> entity_ids = new Dictionary<object,   long>();
        private static readonly Action<object, byte[]>[] message_handlers;
        private static bool settings_loaded = false;
        //private static bool F8_pressed = false;

        public static bool network_handlers_registered { get; private set; }
        //public static bool        is_spectator_mode_on { get; private set; }
        public static bool       show_thrust_reduction { get; private set; }
        public static bool         show_vertical_speed { get; private set; }

        public static IMyPlayer             local_player     { get; private set; }
        public static IMyControllableEntity local_controller { get; private set; }

        static sync_helper()
        {
            show_thrust_reduction = show_vertical_speed = true;

            message_handlers = new Action<object, byte[]>[num_messages];
            message_handlers[(int) message_types.I_TERMS        ] = grid_logic.I_terms_handler;
            message_handlers[(int) message_types.CONTROL_LIMIT  ] = grid_logic.control_warning_handler;
            message_handlers[(int) message_types.THRUST_LOSS    ] = grid_logic.thrust_reduction_handler;
            message_handlers[(int) message_types.MANUAL_THROTTLE] = engine_control_unit.on_manual_throttle_changed;
        }

        private static void log_sync_action(string method_name, string message)
        {
            string player_name;

            if (MyAPIGateway.Multiplayer == null)
                player_name = "LOCAL";
            else if (MyAPIGateway.Multiplayer.IsServer)
                player_name = "SERVER";
            else
            {
                IMyPlayer local_player = MyAPIGateway.Session?.LocalHumanPlayer;
                player_name = (local_player == null) ? "LOCAL" : local_player.SteamUserId.ToString();
            }
            MyLog.Default.WriteLine("TTDTWM SYNC\tsync_helper." + method_name + "(): [" + player_name + "] " + message);
        }

        #region Chat command handling

        private static void command_handler(string message, ref bool dummy)
        {
            bool settings_changed = false;

            message = message.ToLower();
            if (message.StartsWith("/tpdt-tl"))
            {
                show_thrust_reduction = !show_thrust_reduction;
                MyAPIGateway.Utilities.ShowMessage("TP&DT", "Thrust loss indicator is now " + (show_thrust_reduction ? "visible" : "hidden"));
                settings_changed = true;
            }
            else if (message.StartsWith("/tpdt-vs"))
            {
                show_vertical_speed = !show_vertical_speed;
                MyAPIGateway.Utilities.ShowMessage("TP&DT", "Vertical speed indicator is now " + (show_vertical_speed ? "visible" : "hidden"));
                settings_changed = true;
            }

            if (settings_changed)
            {
                display_settings stored_settings;
                stored_settings.show_thrust_reduction = show_thrust_reduction;
                stored_settings.show_vertical_speed   = show_vertical_speed;

                try
                {
                    TextWriter output = MyAPIGateway.Utilities.WriteFileInLocalStorage(settings_file, typeof(display_settings));
                    output.Write(MyAPIGateway.Utilities.SerializeToXML(stored_settings));
                    output.Close();
                }
                catch (Exception error)
                {
                    MyLog.Default.WriteLine(error);
                }
            }
        }

        #endregion

        #region Network handlers

        private static byte[] fill_message(message_types message_id, object entity, byte[] message, uint length)
        {
            if (length > MAX_MESSAGE_LENGTH)
                return null;
            if (!out_buffers.ContainsKey(length))
            {
                out_buffers.Add(length, new byte[SIGNATURE_LENGTH + 1 + 8 + length]);
                for (uint index = 0; index < SIGNATURE_LENGTH; ++index)
                    out_buffers[length][index] = signature[index];
                //log_sync_action("fill_message", string.Format("allocated buffer of length {0} ({1} length {2})", SIGNATURE_LENGTH + 1 + 8 + length, message_id, length));
            }
            byte[] message_buffer = out_buffers[length];
            message_buffer[SIGNATURE_LENGTH] = (byte) message_id;
            if (!encode_entity_id(entity, message_buffer))
                return null;
            //log_sync_action("fill_message", string.Format("entity valid ({0})", message_id));
            for (uint index = 0; index < length; ++index)
                message_buffer[index + SIGNATURE_LENGTH + 1 + 8] = message[index];
            return message_buffer;
        }

        private static void on_message_received(byte[] message)
        {
            int length = message.Length - (SIGNATURE_LENGTH + 1 + 8);
            //log_sync_action("on_message_received", string.Format("length = {0}", length));
            if (length <= 0 || length > MAX_MESSAGE_LENGTH || message[SIGNATURE_LENGTH] >= num_messages)
                return;
            //log_sync_action("on_message_received", string.Format("type = {0}", (message_types) message[SIGNATURE_LENGTH]));
            Action<object, byte[]> invoke_handler = message_handlers[message[SIGNATURE_LENGTH]];
            if (invoke_handler == null)
                return;
            for (int index = 0; index < SIGNATURE_LENGTH; ++index)
            {
                if (message[index] != signature[index])
                    return;
            }
            //log_sync_action("on_message_received", "signature valid");
            object entity = decode_entity_id(message);
            if (entity == null)
                return;
            //log_sync_action("on_message_received", "entity valid");
            for (int index = 0; index < length; ++index)
                in_buffer[index] = message[SIGNATURE_LENGTH + 1 + 8 + index];
            invoke_handler(entity, in_buffer);
        }

        public static void send_message_to_others(message_types message_id, object entity, byte[] message, uint length)
        {
            if (!network_handlers_registered)
                return;
            byte[] message_buffer = fill_message(message_id, entity, message, length);
            if (message_buffer != null)
                MyAPIGateway.Multiplayer.SendMessageToOthers(SYNC_MESSAGE_ID, message_buffer);
        }

        public static void send_message_to(ulong recipient, message_types message_id, object entity, byte[] message, uint length)
        {
            if (!network_handlers_registered || !MyAPIGateway.Multiplayer.IsServer)
                return;
            byte[] message_buffer = fill_message(message_id, entity, message, length);
            if (message_buffer != null)
                MyAPIGateway.Multiplayer.SendMessageTo(SYNC_MESSAGE_ID, message_buffer, recipient);
        }

        public static void try_register_handlers()
        {
            if (!network_handlers_registered && MyAPIGateway.Multiplayer != null && MyAPIGateway.Utilities != null)
            {
                MyAPIGateway.Multiplayer.RegisterMessageHandler(SYNC_MESSAGE_ID, on_message_received);
                MyAPIGateway.Utilities.MessageEntered += command_handler;
                network_handlers_registered = true;
            }
        }

        public static void deregister_handlers()
        {
            if (!network_handlers_registered)
                return;
            MyAPIGateway.Multiplayer.UnregisterMessageHandler(SYNC_MESSAGE_ID, on_message_received);
            MyAPIGateway.Utilities.MessageEntered -= command_handler;
            network_handlers_registered = false;
        }

        #endregion

        #region Entity management

        public static void register_entity(object entity, long entity_id)
        {
            entities.Add(entity_id,    entity);
            entity_ids.Add( entity, entity_id);
        }

        public static void deregister_entity(long entity_id)
        {
            entity_ids.Remove(entities[entity_id]);
            entities.Remove(entity_id);
        }

        private static bool encode_entity_id(object entity, byte[] message)
        {
            if (!entity_ids.ContainsKey(entity))
                return false;
            long entity_id = entity_ids[entity];
            for (int cur_byte = 0; cur_byte < 8; ++cur_byte)
            {
                message[cur_byte + SIGNATURE_LENGTH + 1] = (byte) (entity_id & 0xFF);
                entity_id >>= 8;
            }
            return true;
        }

        private static object decode_entity_id(byte[] message)
        {
            long entity_id = 0;
            for (int cur_byte = 7; cur_byte >= 0; --cur_byte)
                entity_id = (entity_id << 8) | message[cur_byte + SIGNATURE_LENGTH + 1];
            return entities.ContainsKey(entity_id) ? entities[entity_id] : null;
        }

        #endregion

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

            /*
            is_spectator_mode_on = false;
            if (MyAPIGateway.Session.SessionSettings.EnableSpectator)
            {
                var spectator_controller = MyAPIGateway.Session.CameraController as MySpectatorCameraController;
                if (spectator_controller != null)
                    is_spectator_mode_on = spectator_controller.SpectatorCameraMovement == MySpectatorCameraMovementEnum.UserControlled;
            }
            */

            local_player     = MyAPIGateway.Session.LocalHumanPlayer;
            local_controller = (local_player == null) ? null : local_player.Controller.ControlledEntity;

            if (!settings_loaded && network_handlers_registered)
            {
                if (MyAPIGateway.Utilities.FileExistsInLocalStorage(settings_file, typeof(display_settings)))
                {
                    try
                    {
                        TextReader input    = MyAPIGateway.Utilities.ReadFileInLocalStorage(settings_file, typeof(display_settings));
                        var loaded_settings = MyAPIGateway.Utilities.SerializeFromXML<display_settings>(input.ReadToEnd());
                        input.Close();
                        show_thrust_reduction = loaded_settings.show_thrust_reduction;
                        show_vertical_speed   = loaded_settings.show_vertical_speed;
                    }
                    catch (Exception error)
                    {
                        MyLog.Default.WriteLine(error);
                    }
                }
                settings_loaded = true;
            }
        }
    }
}
