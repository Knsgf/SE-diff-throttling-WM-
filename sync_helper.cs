using System;
using System.Collections.Generic;

using Sandbox.ModAPI;
using VRage.Game.ModAPI;
using VRage.Utils;

namespace ttdtwm
{

    static class sync_helper
    {
        const ushort SYNC_MESSAGE_ID = 17370;

        internal const int MAX_MESSAGE_LENGTH = 200;
        internal enum message_types: byte 
        { 
            I_TERMS, THRUSTER_MODES, MANUAL_THROTTLE, GRID_MODES, THRUST_LOSS, REMOTE_SCREEN_TEXT, GRID_CONTROL_SENSITIVITY
        };
        private static readonly int _num_messages;

        const int SIGNATURE_LENGTH = 6;
        private static readonly Dictionary<int, byte[]> _out_buffers = new Dictionary<int, byte[]>();
        private static readonly                 byte[]  _in_buffer   = new byte[MAX_MESSAGE_LENGTH];
        private static readonly                 byte[]  _signature   = { 0, 0, 0x7B, 0x87, 0xAC, 0xC0 };

        private static readonly Dictionary<  long, object>[] _entities;
        private static readonly Dictionary<object,   long>[] _entity_ids;

        private static readonly Action<message_types, object, byte[], int>[] _message_handlers;

        private static readonly double log2 = Math.Log(2.0);

        public static bool network_handlers_registered { get; private set; } = false;
        public static bool running_on_server           { get; private set; } = true;

        static sync_helper()
        {
            _num_messages     = Enum.GetValues(typeof(message_types)).Length;
            _message_handlers = new Action<message_types, object, byte[], int>[_num_messages];
            _message_handlers[(int) message_types.I_TERMS                 ] = grid_logic.I_terms_handler;
            _message_handlers[(int) message_types.THRUST_LOSS             ] = grid_logic.thrust_reduction_handler;
            _message_handlers[(int) message_types.GRID_MODES              ] = thruster_and_grid_tagger.remote_grid_settings;
            _message_handlers[(int) message_types.GRID_CONTROL_SENSITIVITY] = thruster_and_grid_tagger.remote_grid_settings;
            _message_handlers[(int) message_types.THRUSTER_MODES          ] = thruster_and_grid_tagger.remote_thrust_settings;
            _message_handlers[(int) message_types.MANUAL_THROTTLE         ] = thruster_and_grid_tagger.remote_thrust_settings;
            _message_handlers[(int) message_types.REMOTE_SCREEN_TEXT      ] = screen_info.show_remote_text;

            _entities   = new Dictionary<long, object>[_num_messages];
            _entity_ids = new Dictionary<object, long>[_num_messages];
        }

        private static void log_sync_action(string method_name, string message)
        {
            string player_name;

            if (MyAPIGateway.Multiplayer == null)
                player_name = "LOCAL";
            else if (running_on_server)
                player_name = "SERVER";
            else
            {
                IMyPlayer local_player = MyAPIGateway.Session?.LocalHumanPlayer;
                player_name = (local_player == null) ? "LOCAL" : local_player.SteamUserId.ToString();
            }
            MyLog.Default.WriteLine("TTDTWM SYNC\tsync_helper." + method_name + "(): [" + player_name + "] " + message);
        }

        #region little-endian integer serialisers

        public static void encode_unsigned(ulong value, int bytes, byte[] message, int message_offset)
        {
            int end_offset = message_offset + bytes;
            for (int cur_byte = message_offset; cur_byte < end_offset; ++cur_byte)
            {
                message[cur_byte] = (byte) (value & 0xFF);
                value >>= 8;
            }
        }

        public static void encode_signed(long value, int bytes, byte[] message, int message_offset)
        {
            int end_offset = message_offset + bytes;
            for (int cur_byte = message_offset; cur_byte < end_offset; ++cur_byte)
            {
                message[cur_byte] = (byte) (value & 0xFF);
                value >>= 8;
            }
        }

        public static ulong decode_unsigned(int bytes, byte[] message, int message_offset)
        {
            ulong result = 0;
            int end_offset = message_offset + bytes;

            for (int cur_byte = end_offset - 1; cur_byte >= message_offset; --cur_byte)
                result = (result << 8) | message[cur_byte];
            return result;
        }

        public static long decode_signed(int bytes, byte[] message, int message_offset)
        {
            int end_offset = message_offset + bytes - 1;
            long result = (message[end_offset] >= 0x80) ? -1 : 0;

            for (int cur_byte = end_offset; cur_byte >= message_offset; --cur_byte)
                result = (result << 8) | message[cur_byte];
            return result;
        }

        #endregion

        #region Floating-point number serialisers

        public static void encode_double(double value, byte[] message, int message_offset)
        {
            if (value == 0.0)
            {
                encode_signed(0, 7, message, message_offset    );
                encode_signed(0, 2, message, message_offset + 7);
            }
            else
            {
                int power2 = (int) (Math.Log(Math.Abs(value)) / log2);
                double normalised_value = value / Math.Pow(2.0, power2);
                encode_signed((long) (normalised_value * (1L << 54)), 7, message, message_offset);
                encode_signed(power2, 2, message, message_offset + 7);
            }
        }

        public static double decode_double(byte[] message, int message_offset)
        {
            long premultiplied_value = decode_signed(7, message, message_offset);
            if (premultiplied_value == 0)
                return 0.0;
            
            double normalised_value = ((double) premultiplied_value) / (1L << 54);
            int power2 = (int) decode_signed(2, message, message_offset + 7);
            return normalised_value * Math.Pow(2.0, power2);
        }

        #endregion

        #region Network handlers

        private static byte[] fill_message(message_types message_id, object entity, byte[] message, int length)
        {
            if (length > MAX_MESSAGE_LENGTH)
                return null;
            if (length < 0)
                length = 0;
            if (!_out_buffers.ContainsKey(length))
            {
                _out_buffers.Add(length, new byte[SIGNATURE_LENGTH + 1 + 8 + length]);
                for (uint index = 0; index < SIGNATURE_LENGTH; ++index)
                    _out_buffers[length][index] = _signature[index];
                //log_sync_action("fill_message", string.Format("allocated buffer of length {0} ({1} length {2})", SIGNATURE_LENGTH + 1 + 8 + length, message_id, length));
            }
            byte[] message_buffer = _out_buffers[length];
            message_buffer[SIGNATURE_LENGTH] = (byte) message_id;
            if (!encode_entity_id(message_id, entity, message_buffer))
                return null;
            //log_sync_action("fill_message", string.Format("entity valid ({0})", message_id));
            int buffer_index = SIGNATURE_LENGTH + 1 + 8;
            for (int index = 0; index < length; ++index)
                message_buffer[buffer_index++] = message[index];
            return message_buffer;
        }

        private static void on_message_received(byte[] message)
        {
            int  length     = message.Length - (SIGNATURE_LENGTH + 1 + 8);
            byte message_id = message[SIGNATURE_LENGTH];
            //log_sync_action("on_message_received", string.Format("length = {0}", length));
            if (length <= 0 || length > MAX_MESSAGE_LENGTH || message_id >= _num_messages)
                return;
            //log_sync_action("on_message_received", string.Format("type = {0}", (message_types) message[SIGNATURE_LENGTH]));
            Action<message_types, object, byte[], int> invoke_handler = _message_handlers[message_id];
            if (invoke_handler == null)
                return;
            for (int index = 0; index < SIGNATURE_LENGTH; ++index)
            {
                if (message[index] != _signature[index])
                    return;
            }
            //log_sync_action("on_message_received", "signature valid");
            object entity       = decode_entity_id((message_types) message_id, message);
            int    buffer_index = SIGNATURE_LENGTH + 1 + 8;
            for (int index = 0; index < length; ++index)
                _in_buffer[index] = message[buffer_index++];
            invoke_handler((message_types) message_id, entity, _in_buffer, length);
        }

        /*
        public static void send_message_to_self(message_types message_id, long entity_id, byte[] message, int length)
        {
            if (!_entities.ContainsKey(entity_id))
                return;

            byte[] message_buffer = fill_message(message_id, _entities[entity_id], message, length);
            if (message_buffer != null)
                on_message_received(message_buffer);
        }
        */

        public static void send_message_to_others(message_types message_id, object entity, byte[] message, int length)
        {
            if (!network_handlers_registered)
                return;
            byte[] message_buffer = fill_message(message_id, entity, message, length);
            if (message_buffer != null)
                MyAPIGateway.Multiplayer.SendMessageToOthers(SYNC_MESSAGE_ID, message_buffer);
        }

        public static void send_message_to(ulong recipient, message_types message_id, object entity, byte[] message, int length, bool reliable = true)
        {
            if (!network_handlers_registered || !running_on_server)
                return;
            byte[] message_buffer = fill_message(message_id, entity, message, length);
            if (message_buffer != null)
                MyAPIGateway.Multiplayer.SendMessageTo(SYNC_MESSAGE_ID, message_buffer, recipient, reliable);
        }

        public static void send_message_to_server(message_types message_id, object entity, byte[] message, int length)
        {
            if (!network_handlers_registered || running_on_server)
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
                running_on_server           = MyAPIGateway.Multiplayer.IsServer;
            }
        }

        public static void deregister_handlers()
        {
            if (!network_handlers_registered)
                return;
            MyAPIGateway.Multiplayer.UnregisterMessageHandler(SYNC_MESSAGE_ID, on_message_received);
            network_handlers_registered = false;
            running_on_server           = true;
        }

        #endregion

        #region Entity management

        public static void register_entity(message_types message_type, object entity, long entity_id)
        {
            if (_entities[(int) message_type] == null)
            {
                _entities  [(int) message_type] = new Dictionary<  long, object>();
                _entity_ids[(int) message_type] = new Dictionary<object,   long>();
            }
            _entities  [(int) message_type].Add(entity_id,    entity);
            _entity_ids[(int) message_type].Add(   entity, entity_id);
        }

        public static void deregister_entity(message_types message_type, long entity_id)
        {
            if (_entities[(int) message_type] != null && _entities[(int) message_type].ContainsKey(entity_id))
            {
                _entity_ids[(int) message_type].Remove(_entities[(int) message_type][entity_id]);
                _entities  [(int) message_type].Remove(                              entity_id );
            }
        }

        private static bool encode_entity_id(message_types message_type, object entity, byte[] message)
        {
            long entity_id;

            if (entity == null)
                entity_id = 0;
            else
            {
                if (!_entity_ids[(int) message_type].ContainsKey(entity))
                    return false;
                entity_id = _entity_ids[(int) message_type][entity];
            }
            encode_signed(entity_id, 8, message, SIGNATURE_LENGTH + 1);
            return true;
        }

        private static object decode_entity_id(message_types message_type, byte[] message)
        {
            long entity_id = decode_signed(8, message, SIGNATURE_LENGTH + 1);
            return (entity_id != 0 && _entities[(int) message_type] != null && _entities[(int) message_type].ContainsKey(entity_id)) ? _entities[(int) message_type][entity_id] : null;
        }

        #endregion
    }
}
