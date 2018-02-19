﻿using System.Text;

using Sandbox.ModAPI;

namespace ttdtwm
{
    static class thruster_tagger
    {
        private static string           _thruster_data, _throttle_setting;
        private static IMyTerminalBlock _current_thruster = null;
        private static bool             _current_active_control_available, _current_active_control_on, _current_anti_slip_on, _current_thrust_limiter_on;
        private static uint             _manual_throttle;
        private static byte[]           _message = new byte[1];

        public static int displayed_thrust_limit { get; set; }

        private static void update_flags(IMyTerminalBlock thruster)
        {
            if (!(thruster is IMyThrust))
            {
                _current_thruster = null;
                _throttle_setting = null;
                _current_active_control_available = _current_active_control_on = _current_anti_slip_on = _current_thrust_limiter_on = false;
                _manual_throttle = 0;
                return;
            }
            if (thruster == _current_thruster)
                return;

            _current_thruster  = thruster;
            _thruster_data     = thruster.CustomData;
            bool contains_RCS = _thruster_data.ContainsRCSTag();
            _current_active_control_available = ((IMyFunctionalBlock) thruster).Enabled;
            _current_active_control_on        = _thruster_data.ContainsTHRTag()  ||  contains_RCS;
            _current_anti_slip_on             = _thruster_data.ContainsSLPTag();
            _current_thrust_limiter_on        = _thruster_data.ContainsSTATTag() && !contains_RCS;

            _throttle_setting  = "TTDTWM_MT_" + thruster.EntityId.ToString();
            bool setting_saved = MyAPIGateway.Utilities.GetVariable(_throttle_setting, out _manual_throttle);
            if (!setting_saved)
                _manual_throttle = 0;

            thruster.RefreshCustomInfo();
        }

        public static void show_thrust_limit(IMyTerminalBlock thruster, StringBuilder info_text)
        {
            sync_helper.send_message_to_self(sync_helper.message_types.SHOW_THRUST_LIMIT, thruster.EntityId, _message, 1);
            info_text.Append("Balanced Thrust Level: ");
            if (displayed_thrust_limit < 0)
                info_text.Append("N/A");
            else
                info_text.Append(displayed_thrust_limit).Append(" %");
        }

        public static bool is_active_control_available(IMyTerminalBlock thruster)
        {
            update_flags(thruster);
            return thruster is IMyThrust;
        }

        public static bool is_under_active_control(IMyTerminalBlock thruster)
        {
            update_flags(thruster);
            return _current_active_control_on;
        }

        public static void set_active_control(IMyTerminalBlock thruster, bool new_state_on)
        {
            update_flags(thruster);
            if (!is_active_control_available(thruster))
                return;
            if (new_state_on)
            {
                thruster.CustomData        = _current_anti_slip_on ? thruster.CustomData.RemoveRCSTag().AddTHRTag() : thruster.CustomData.RemoveTHRTag().AddRCSTag();
                _current_active_control_on = true;
            }
            else
            {
                thruster.CustomData        = thruster.CustomData.RemoveTHRTag().RemoveRCSTag();
                _current_active_control_on = false;
            }
        }

        public static bool is_anti_slip_available(IMyTerminalBlock thruster)
        {
            update_flags(thruster);
            return _current_active_control_on && !_current_thrust_limiter_on;
        }

        public static bool is_anti_slip(IMyTerminalBlock thruster)
        {
            update_flags(thruster);
            return _current_anti_slip_on;
        }

        public static void set_anti_slip(IMyTerminalBlock thruster, bool new_state_on)
        {
            update_flags(thruster);
            if (!is_active_control_available(thruster))
                return;
            if (is_under_active_control(thruster) && !is_thrust_limited(thruster))
            {
                if (!new_state_on)
                {
                    thruster.CustomData   = thruster.CustomData.RemoveTHRTag().AddRCSTag();
                    _current_anti_slip_on = false;
                }
                else
                {
                    thruster.CustomData   = thruster.CustomData.RemoveRCSTag().AddTHRTag();
                    _current_anti_slip_on = true;
                }
            }
        }

        public static bool is_thrust_limiter_available(IMyTerminalBlock thruster)
        {
            update_flags(thruster);
            return is_active_control_available(thruster) && (!_current_active_control_on || _current_anti_slip_on);
        }

        public static bool is_thrust_limited(IMyTerminalBlock thruster)
        {
            update_flags(thruster);
            return _current_thrust_limiter_on;
        }

        public static void set_thrust_limited(IMyTerminalBlock thruster, bool new_state_on)
        {
            update_flags(thruster);
            if (!is_active_control_available(thruster))
                return;
            if (!new_state_on)
            {
                thruster.CustomData        = thruster.CustomData.RemoveSTATTag();
                _current_thrust_limiter_on = false;
            }
            else if (is_thrust_limiter_available(thruster))
            {
                thruster.CustomData        = thruster.CustomData.AddSTATTag();
                _current_thrust_limiter_on = true;
            }
        }

        public static float get_manual_throttle(IMyTerminalBlock thruster)
        {
            update_flags(thruster);
            return _manual_throttle;
        }

        public static void set_manual_throttle(IMyTerminalBlock thruster, float new_setting)
        {
            update_flags(thruster);
            if (new_setting < 0.0f)
                _manual_throttle = 0;
            else
            {
                _manual_throttle = (uint) new_setting;
                if (_manual_throttle > 100)
                    _manual_throttle = 100;
            }
            //MyAPIGateway.Utilities.SetVariable(_throttle_setting, _manual_throttle);
            _message[0] = (byte) (_manual_throttle | 0x80U);
            sync_helper.send_message_to_self(sync_helper.message_types.MANUAL_THROTTLE, thruster.EntityId, _message, 1);
        }

        public static void throttle_status(IMyTerminalBlock thruster, StringBuilder status)
        {
            float throttle = get_manual_throttle(thruster);
            status.Clear();
            status.Append((throttle < 1.0f) ? "Disabled" : string.Format("{0:F0} %", throttle));
        }
    }
}
