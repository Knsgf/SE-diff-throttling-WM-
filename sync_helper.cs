using System;
using System.Collections.Generic;
using System.Text;
using System.IO;

using Sandbox.ModAPI;
using VRage.Game.ModAPI;
using VRage.Game.ModAPI.Interfaces;
using VRage.Utils;

namespace ttdtwm
{

    static class sync_helper
    {
        const ushort SYNC_MESSAGE_ID = 17370;

        internal const int MAX_MESSAGE_LENGTH = 200;

        internal enum message_types { I_TERMS, MANUAL_THROTTLE, CONTROL_LIMIT, THRUST_LOSS, GET_THRUST_LIMIT, REMOTE_SCREEN_TEXT };
        private static readonly int _num_messages = Enum.GetValues(typeof(message_types)).Length;

        const int SIGNATURE_LENGTH = 6;
        private static readonly Dictionary<int, byte[]> _out_buffers = new Dictionary<int, byte[]>();
        private static readonly                 byte[]  _in_buffer   = new byte[MAX_MESSAGE_LENGTH];
        private static readonly                 byte[]  _signature   = { 0, 0, 0x7B, 0x87, 0xAC, 0xC0 };

        private static readonly Dictionary<  long, object> _entities   = new Dictionary<  long, object>();
        private static readonly Dictionary<object,   long> _entity_ids = new Dictionary<object,   long>();

        private static readonly Action<object, byte[], int>[] _message_handlers;

        public static bool network_handlers_registered { get; private set; }

        static sync_helper()
        {
            _message_handlers = new Action<object, byte[], int>[_num_messages];
            _message_handlers[(int) message_types.I_TERMS           ] = grid_logic.I_terms_handler;
            _message_handlers[(int) message_types.THRUST_LOSS       ] = grid_logic.thrust_reduction_handler;
            _message_handlers[(int) message_types.MANUAL_THROTTLE   ] = engine_control_unit.on_manual_throttle_changed;
            _message_handlers[(int) message_types.GET_THRUST_LIMIT  ] = engine_control_unit.extract_thrust_limit;
            _message_handlers[(int) message_types.REMOTE_SCREEN_TEXT] = screen_info.show_remote_text;
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

        #region Network handlers

        private static byte[] fill_message(message_types message_id, object entity, byte[] message, int length)
        {
            if (length > MAX_MESSAGE_LENGTH)
                return null;
            if (!_out_buffers.ContainsKey(length))
            {
                _out_buffers.Add(length, new byte[SIGNATURE_LENGTH + 1 + 8 + length]);
                for (uint index = 0; index < SIGNATURE_LENGTH; ++index)
                    _out_buffers[length][index] = _signature[index];
                //log_sync_action("fill_message", string.Format("allocated buffer of length {0} ({1} length {2})", SIGNATURE_LENGTH + 1 + 8 + length, message_id, length));
            }
            byte[] message_buffer = _out_buffers[length];
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
            if (length <= 0 || length > MAX_MESSAGE_LENGTH || message[SIGNATURE_LENGTH] >= _num_messages)
                return;
            //log_sync_action("on_message_received", string.Format("type = {0}", (message_types) message[SIGNATURE_LENGTH]));
            Action<object, byte[], int> invoke_handler = _message_handlers[message[SIGNATURE_LENGTH]];
            if (invoke_handler == null)
                return;
            for (int index = 0; index < SIGNATURE_LENGTH; ++index)
            {
                if (message[index] != _signature[index])
                    return;
            }
            //log_sync_action("on_message_received", "signature valid");
            object entity = decode_entity_id(message);
            //log_sync_action("on_message_received", "entity valid");
            for (int index = 0; index < length; ++index)
                _in_buffer[index] = message[SIGNATURE_LENGTH + 1 + 8 + index];
            invoke_handler(entity, _in_buffer, length);
        }

        public static void send_message_to_self(message_types message_id, long entity_id, byte[] message, int length)
        {
            if (!_entities.ContainsKey(entity_id))
                return;

            byte[] message_buffer = fill_message(message_id, _entities[entity_id], message, length);
            if (message_buffer != null)
                on_message_received(message_buffer);
        }

        public static void send_message_to_others(message_types message_id, object entity, byte[] message, int length)
        {
            if (!network_handlers_registered)
                return;
            byte[] message_buffer = fill_message(message_id, entity, message, length);
            if (message_buffer != null)
                MyAPIGateway.Multiplayer.SendMessageToOthers(SYNC_MESSAGE_ID, message_buffer);
        }

        public static void send_message_to(ulong recipient, message_types message_id, object entity, byte[] message, int length)
        {
            if (!network_handlers_registered || !MyAPIGateway.Multiplayer.IsServer)
                return;
            byte[] message_buffer = fill_message(message_id, entity, message, length);
            if (message_buffer != null)
                MyAPIGateway.Multiplayer.SendMessageTo(SYNC_MESSAGE_ID, message_buffer, recipient);
        }

        public static void send_message_to_server(message_types message_id, object entity, byte[] message, int length)
        {
            if (!network_handlers_registered || MyAPIGateway.Multiplayer.IsServer)
                return;
            byte[] message_buffer = fill_message(message_id, entity, message, length);
            if (message_buffer != null)
                MyAPIGateway.Multiplayer.SendMessageToServer(SYNC_MESSAGE_ID, message_buffer);
        }

        public static void try_register_handlers()
        {
            if (!network_handlers_registered && MyAPIGateway.Multiplayer != null)
            {
                MyAPIGateway.Multiplayer.RegisterMessageHandler(SYNC_MESSAGE_ID, on_message_received);
                network_handlers_registered = true;
            }
        }

        public static void deregister_handlers()
        {
            if (!network_handlers_registered)
                return;
            MyAPIGateway.Multiplayer.UnregisterMessageHandler(SYNC_MESSAGE_ID, on_message_received);
            network_handlers_registered = false;
        }

        #endregion

        #region Entity management

        public static void register_entity(object entity, long entity_id)
        {
            _entities.Add  (entity_id,    entity);
            _entity_ids.Add(   entity, entity_id);
        }

        public static void deregister_entity(long entity_id)
        {
            if (_entities.ContainsKey(entity_id))
            {
                _entity_ids.Remove(_entities[entity_id]);
                _entities.Remove(entity_id);
            }
        }

        private static bool encode_entity_id(object entity, byte[] message)
        {
            long entity_id;

            if (entity == null)
                entity_id = 0;
            else
            {
                if (!_entity_ids.ContainsKey(entity))
                    return false;
                entity_id = _entity_ids[entity];
            }
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
            return (entity_id != 0 && _entities.ContainsKey(entity_id)) ? _entities[entity_id] : null;
        }

        #endregion
    }
}
