using System.Text;

using Sandbox.ModAPI;

namespace ttdtwm
{
    static class thruster_tagger
    {
        private static StringBuilder    thruster_name    = new StringBuilder();
        private static IMyTerminalBlock current_thruster = null;
        private static bool             current_active_control_available, current_active_control_on, current_anti_slip_on, current_thrust_limiter_on;
        private static uint             manual_throttle;

        private static void update_flags(IMyTerminalBlock thruster)
        {
            if (!(thruster is IMyThrust))
            {
                current_thruster = null;
                current_active_control_available = current_active_control_on = current_anti_slip_on = current_thrust_limiter_on = false;
                manual_throttle = 0;
                return;
            }
            if (thruster == current_thruster)
                return;

            current_thruster = thruster;
            thruster.CustomName.ToUpperTo(thruster_name);
            bool contains_RCS = thruster_name.ContainsRCSTag();
            current_active_control_available = ((IMyFunctionalBlock) thruster).Enabled;
            current_active_control_on        = thruster_name.ContainsTHRTag()  ||  contains_RCS;
            current_anti_slip_on             = current_active_control_on       && !contains_RCS;
            current_thrust_limiter_on        = thruster_name.ContainsSTATTag() && !contains_RCS;

            string throttle_setting = "TTDTWM_MT_" + thruster.EntityId.ToString();
            bool   setting_saved    = MyAPIGateway.Utilities.GetVariable(throttle_setting, out manual_throttle);
            if (!setting_saved)
                manual_throttle = 0;
        }

        public static bool is_active_control_available(IMyTerminalBlock thruster)
        {
            update_flags(thruster);
            return thruster is IMyThrust;
        }

        public static bool is_under_active_control(IMyTerminalBlock thruster)
        {
            update_flags(thruster);
            return current_active_control_on;
        }

        public static void set_active_control(IMyTerminalBlock thruster, bool new_state_on)
        {
            if (!is_active_control_available(thruster))
                return;
            if (new_state_on)
            {
                thruster.SetCustomName(current_anti_slip_on ? thruster.CustomName.RemoveRCSTag().AddTHRTag() : thruster.CustomName.RemoveTHRTag().AddRCSTag());
                current_active_control_on = true;
            }
            else
            {
                thruster.SetCustomName(thruster.CustomName.RemoveTHRTag().RemoveRCSTag());
                current_active_control_on = current_anti_slip_on = false;
            }
        }

        public static bool is_anti_slip_available(IMyTerminalBlock thruster)
        {
            update_flags(thruster);
            return current_active_control_on && !current_thrust_limiter_on;
        }

        public static bool is_anti_slip(IMyTerminalBlock thruster)
        {
            update_flags(thruster);
            return current_anti_slip_on;
        }

        public static void set_anti_slip(IMyTerminalBlock thruster, bool new_state_on)
        {
            if (!is_active_control_available(thruster))
                return;
            if (is_under_active_control(thruster))
            {
                if (!new_state_on)
                {
                    thruster.SetCustomName(thruster.CustomName.RemoveTHRTag().RemoveSTATTag().AddRCSTag());
                    current_anti_slip_on = current_thrust_limiter_on = false;
                }
                else
                {
                    thruster.SetCustomName(thruster.CustomName.AddTHRTag().RemoveRCSTag());
                    current_anti_slip_on = true;
                }
            }
        }

        public static bool is_thrust_limiter_available(IMyTerminalBlock thruster)
        {
            return is_active_control_available(thruster) && (!current_active_control_on || current_anti_slip_on);
        }

        public static bool is_thrust_limited(IMyTerminalBlock thruster)
        {
            update_flags(thruster);
            return current_thrust_limiter_on;
        }

        public static void set_thrust_limited(IMyTerminalBlock thruster, bool new_state_on)
        {
            if (!is_active_control_available(thruster))
                return;
            if (!new_state_on)
            {
                thruster.SetCustomName(thruster.CustomName.RemoveSTATTag());
                current_thrust_limiter_on = false;
            }
            else if (is_thrust_limiter_available(thruster))
            {
                thruster.SetCustomName(thruster.CustomName.AddSTATTag());
                current_thrust_limiter_on = true;
            }
        }

        public static float get_manual_throttle(IMyTerminalBlock thruster)
        {
            update_flags(thruster);
            return manual_throttle;
        }

        public static void set_manual_throttle(IMyTerminalBlock thruster, float new_setting)
        {
            if (new_setting < 0.0f || !is_under_active_control(thruster))
                manual_throttle = 0;
            else
            {
                manual_throttle = (uint) new_setting;
                if (manual_throttle > 100)
                    manual_throttle = 100;
            }
            string throttle_setting = "TTDTWM_MT_" + thruster.EntityId.ToString();
            MyAPIGateway.Utilities.SetVariable(throttle_setting, manual_throttle);
        }

        public static void throttle_status(IMyTerminalBlock thruster, StringBuilder status)
        {
            float throttle = get_manual_throttle(thruster);
            status.Clear();
            status.Append((throttle < 1.0f) ? "Disabled" : string.Format("{0:F0} %", throttle));
        }
    }
}
