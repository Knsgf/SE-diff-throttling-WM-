using System.Text;

using Sandbox.ModAPI;

namespace ttdtwm
{
    static class thruster_tagger
    {
        private static StringBuilder    thruster_name    = new StringBuilder();
        private static IMyTerminalBlock current_thruster = null;
        private static bool             current_active_control_available, current_active_control_on, current_anti_slip_on, current_thrust_limiter_on;

        private static void update_flags(IMyTerminalBlock thruster)
        {
            if (!(thruster is IMyThrust))
            {
                current_thruster = null;
                current_active_control_available = current_active_control_on = current_anti_slip_on = current_thrust_limiter_on = false;
                return;
            }
            if (thruster == current_thruster)
                return;

            current_thruster = thruster;
            thruster.CustomName.ToUpperTo(thruster_name);
            bool contains_THR = thruster_name.ContainsTHRTag(), contains_RCS = thruster_name.ContainsRCSTag();
            current_active_control_available = ((IMyFunctionalBlock) thruster).Enabled;
            current_active_control_on        = current_active_control_available && (contains_THR || contains_RCS);
            current_anti_slip_on             = current_active_control_on        && !contains_RCS;
            current_thrust_limiter_on        = current_active_control_available && !contains_RCS && thruster_name.ContainsSTATTag();
        }

        public static bool is_active_control_available(IMyTerminalBlock thruster)
        {
            update_flags(thruster);
            return current_active_control_available;
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
            if (!new_state_on && is_under_active_control(thruster))
            {
                thruster.SetCustomName(thruster.CustomName.RemoveTHRTag().RemoveSTATTag().AddRCSTag());
                current_anti_slip_on = current_thrust_limiter_on = false;
            }
            else
            {
                string new_name = is_under_active_control(thruster) ? thruster.CustomName.AddTHRTag() : thruster.CustomName;
                if (is_thrust_limited(thruster))
                {
                    new_name = new_name.AddSTATTag();
                    current_thrust_limiter_on = true;
                }
                thruster.SetCustomName(new_name.RemoveRCSTag());
                current_anti_slip_on = true;
            }
        }

        public static bool is_thrust_limiter_available(IMyTerminalBlock thruster)
        {
            update_flags(thruster);
            return !current_active_control_on || current_anti_slip_on;
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
    }
}
