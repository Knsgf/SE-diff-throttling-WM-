﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using Sandbox.ModAPI;
using Sandbox.ModAPI.Interfaces;
using Sandbox.ModAPI.Interfaces.Terminal;
using VRage.Game.Components;
using VRage.Game.ModAPI;
using VRage.ModAPI;
using VRage.Utils;

namespace ttdtwm
{
    [MySessionComponentDescriptor(MyUpdateOrder.BeforeSimulation)]
    public class session_handler: MySessionComponentBase
    {
        #region fields

        private Dictionary<IMyCubeGrid, grid_logic> _grids = new Dictionary<IMyCubeGrid, grid_logic>();
        private Action _grids_handle_60Hz = null, _grids_handle_4Hz = null, _grids_handle_2s_period = null;

        private IMyThrust         _sample_thruster   = null;
        private IMyShipController _sample_controller = null;

        private int  _count15 = 15, _count8 = 8;
        private bool _entity_events_set = false, _panel_controls_set = false;

        #endregion

        #region debug

        private void log_session_action(string method_name, string message)
        {
            MyLog.Default.WriteLine(string.Format("TTDTWM\tsession_handler.{0}(): {1}\n\t\tTotal grids: {2}", method_name, message, _grids.Count));
        }

        #endregion

        #region event handlers

        private void on_entity_added(IMyEntity entity)
        {
            var grid = entity as IMyCubeGrid;
            if (grid != null)
            {
                var new_grid_logic = new grid_logic(grid, this);
                _grids_handle_60Hz      += new_grid_logic.handle_60Hz;
                _grids_handle_4Hz       += new_grid_logic.handle_4Hz;
                _grids_handle_2s_period += new_grid_logic.handle_2s_period;
                _grids.Add(grid, new_grid_logic);
            }
        }

        private void on_entity_removed(IMyEntity entity)
        {
            var grid = entity as IMyCubeGrid;
            if (grid != null && _grids.ContainsKey(grid))
            {
                grid_logic grid_logic_to_remove = _grids[grid];
                _grids_handle_60Hz      -= grid_logic_to_remove.handle_60Hz;
                _grids_handle_4Hz       -= grid_logic_to_remove.handle_4Hz;
                _grids_handle_2s_period -= grid_logic_to_remove.handle_2s_period;
                grid_logic_to_remove.Dispose();
                _grids.Remove(grid);
            }
        }

        private bool is_grid_CoT_mode_on(IMyTerminalBlock controller)
        {
            IMyCubeGrid grid = controller.CubeGrid;
            return _grids[grid].CoT_mode_forced;
        }

        private bool is_grid_CoT_mode_available(IMyTerminalBlock controller)
        {
            IMyCubeGrid grid = controller.CubeGrid;
            return _grids[grid].is_CoT_mode_available;
        }

        private void set_grid_CoT_mode(IMyTerminalBlock controller, bool new_state)
        {
            IMyCubeGrid grid = controller.CubeGrid;
            _grids[grid].CoT_mode_forced = new_state;
        }

        private bool is_grid_landing_mode_on(IMyTerminalBlock controller)
        {
            IMyCubeGrid grid = controller.CubeGrid;
            return _grids[grid].landing_mode_on;
        }

        private bool is_grid_landing_mode_available(IMyTerminalBlock controller)
        {
            IMyCubeGrid grid = controller.CubeGrid;
            return _grids[grid].is_landing_mode_available;
        }

        private void set_grid_landing_mode(IMyTerminalBlock controller, bool new_state)
        {
            IMyCubeGrid grid = controller.CubeGrid;
            _grids[grid].landing_mode_on = new_state;
        }

        #endregion

        #region UI helpers

        private void create_toggle<_block_>(string id, string title, string enabled_text, string disabled_text, Action<IMyTerminalBlock> action, 
            Func<IMyTerminalBlock, bool> getter, Func<IMyTerminalBlock, bool> state, string icon)
        {
            var toggle_action = MyAPIGateway.TerminalControls.CreateAction<_block_>(id);

            toggle_action.Action = action;
            if (state != null)
                toggle_action.Enabled = state;
            if (icon != null && icon != "")
                toggle_action.Icon = @"Textures\GUI\Icons\Actions\" + icon + ".dds";
            toggle_action.Name = new StringBuilder(title);
            toggle_action.ValidForGroups = true;
            toggle_action.Writer = delegate (IMyTerminalBlock block, StringBuilder output)
            {
                output.Clear();
                output.Append(getter(block) ? enabled_text : disabled_text);
            };
            MyAPIGateway.TerminalControls.AddAction<_block_>(toggle_action);
        }

        private void create_checkbox<_block_>(string id, string title, string tooltip, string toolbar_enabled_text, string toolbar_disabled_text,
            Func<IMyTerminalBlock, bool> getter, Action<IMyTerminalBlock, bool> setter, Func<IMyTerminalBlock, bool> state)
        {
            var panel_checkbox = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlCheckbox, _block_>(id);

            panel_checkbox.Getter = getter;
            panel_checkbox.Setter = setter;
            if (state != null)
                panel_checkbox.Enabled = state;
            panel_checkbox.Title = MyStringId.GetOrCompute(title);
            if (tooltip != null)
                panel_checkbox.Tooltip = MyStringId.GetOrCompute(tooltip);
            panel_checkbox.SupportsMultipleBlocks = true;
            MyAPIGateway.TerminalControls.AddControl<_block_>(panel_checkbox);


            create_toggle<_block_>(id + "Toggle", title + " On/Off", toolbar_enabled_text, toolbar_disabled_text,
                delegate (IMyTerminalBlock block)
                {
                    setter(block, !getter(block));
                },
                getter, state, "LargeShipToggle");
        }

        private void create_switch<_block_>(string id, string title, string tooltip, string enabled_text, string disabled_text, string toolbar_enabled_text, string toolbar_disabled_text,
            Func<IMyTerminalBlock, bool> getter, Action<IMyTerminalBlock, bool> setter, Func<IMyTerminalBlock, bool> state)
        {
            var panel_switch = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlOnOffSwitch, _block_>(id);

            panel_switch.Getter = getter;
            panel_switch.Setter = setter;
            if (state != null)
                panel_switch.Enabled = state;
            panel_switch.Title = MyStringId.GetOrCompute(title);
            if (tooltip != null)
                panel_switch.Tooltip = MyStringId.GetOrCompute(tooltip);
            panel_switch.OnText  = MyStringId.GetOrCompute( enabled_text);
            panel_switch.OffText = MyStringId.GetOrCompute(disabled_text);
            panel_switch.SupportsMultipleBlocks = true;
            MyAPIGateway.TerminalControls.AddControl<_block_>(panel_switch);

            create_toggle<_block_>(id + "OnOff", title + " On/Off", toolbar_enabled_text, toolbar_disabled_text,
                delegate (IMyTerminalBlock block)
                {
                    setter(block, !getter(block));
                },
                getter, state, "MissileToggle");
            create_toggle<_block_>(id + "OnOff_On", title + " On", toolbar_enabled_text, toolbar_disabled_text,
                delegate (IMyTerminalBlock block)
                {
                    setter(block, true);
                },
                getter, state, "MissileSwitchOn");
            create_toggle<_block_>(id + "OnOff_Off", title + " Off", toolbar_enabled_text, toolbar_disabled_text,
                delegate (IMyTerminalBlock block)
                {
                    setter(block, false);
                },
                getter, state, "MissileSwitchOff");
        }

        private void create_slider_action<_block_>(string id, string title, Action<IMyTerminalBlock> action, Action<IMyTerminalBlock, StringBuilder> status, string icon)
        {
            var toggle_action = MyAPIGateway.TerminalControls.CreateAction<_block_>(id);

            toggle_action.Action = action;
            if (icon != null && icon != "")
                toggle_action.Icon = @"Textures\GUI\Icons\Actions\" + icon + ".dds";
            toggle_action.Name           = new StringBuilder(title);
            toggle_action.ValidForGroups = true;
            toggle_action.Writer         = status;
            MyAPIGateway.TerminalControls.AddAction<_block_>(toggle_action);
        }

        #endregion

        internal void sample_thruster(IMyThrust thruster)
        {
            if (!_panel_controls_set && _sample_thruster == null)
                _sample_thruster = thruster;
        }

        internal void sample_controller(IMyShipController controller)
        {
            if (!_panel_controls_set && _sample_controller == null)
                _sample_controller = controller;
        }

        private void try_register_handlers()
        {
            if (!sync_helper.network_handlers_registered)
                sync_helper.try_register_handlers();
            if (!_entity_events_set && MyAPIGateway.Entities != null)
            {
                var existing_entities = new HashSet<IMyEntity>();
                MyAPIGateway.Entities.GetEntities(existing_entities);
                foreach (var cur_entity in existing_entities)
                    on_entity_added(cur_entity);
                MyAPIGateway.Entities.OnEntityAdd    += on_entity_added;
                MyAPIGateway.Entities.OnEntityRemove += on_entity_removed;
                _entity_events_set = true;
            }
            if (!_panel_controls_set && _sample_thruster != null && _sample_controller != null && MyAPIGateway.TerminalControls != null)
            {
                try
                {
                    _sample_thruster.GetValueFloat("Override");
                    _sample_controller.GetValueBool("DampenersOverride");
                }
                catch (NullReferenceException dummy)
                {
                    return;
                }

                IMyTerminalControlSeparator cockpit_line = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlSeparator, IMyCockpit>("TTDTWM_LINE1");
                MyAPIGateway.TerminalControls.AddControl<IMyCockpit>(cockpit_line);
                create_checkbox<IMyCockpit>("ForceCoTMode", "Force CoT mode", null,               "CoT",   "Auto",     is_grid_CoT_mode_on,     set_grid_CoT_mode,     is_grid_CoT_mode_available);
                create_switch  <IMyCockpit>( "LandingMode",   "Landing mode", null, "On", "Off", "Land", "Flight", is_grid_landing_mode_on, set_grid_landing_mode, is_grid_landing_mode_available);

                IMyTerminalControlSeparator RC_line = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlSeparator, IMyRemoteControl>("TTDTWM_LINE1");
                MyAPIGateway.TerminalControls.AddControl<IMyRemoteControl>(RC_line);
                create_checkbox<IMyRemoteControl>("ForceCoTMode", "Force CoT mode", null,               "CoT",   "Auto",     is_grid_CoT_mode_on,     set_grid_CoT_mode,     is_grid_CoT_mode_available);
                create_switch  <IMyRemoteControl>( "LandingMode",   "Landing mode", null, "On", "Off", "Land", "Flight", is_grid_landing_mode_on, set_grid_landing_mode, is_grid_landing_mode_available);

                IMyTerminalControlSeparator thruster_line = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlSeparator, IMyThrust>("TTDTWM_LINE1");
                MyAPIGateway.TerminalControls.AddControl<IMyThrust>(thruster_line);
                create_switch<IMyThrust>("ActiveControl",        "Active Control", null, "On", "Off", "On", "Off", thruster_tagger.is_under_active_control, thruster_tagger.set_active_control, thruster_tagger.is_active_control_available);
                create_switch<IMyThrust>(     "AntiSlip",       "Slip Prevention", null, "On", "Off", "On", "Off", thruster_tagger.is_anti_slip           , thruster_tagger.set_anti_slip     , thruster_tagger.is_anti_slip_available     );
                create_switch<IMyThrust>(  "StaticLimit", "Passive Stabilisation", null, "On", "Off", "On", "Off", thruster_tagger.is_thrust_limited      , thruster_tagger.set_thrust_limited, thruster_tagger.is_thrust_limiter_available);

                IMyTerminalControlSlider manual_throttle = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlSlider, IMyThrust>("ManualThrottle");
                manual_throttle.Getter  = thruster_tagger.get_manual_throttle;
                manual_throttle.Setter  = thruster_tagger.set_manual_throttle;
                //manual_throttle.Enabled = thruster_tagger.is_under_active_control;
                manual_throttle.SupportsMultipleBlocks = true;
                manual_throttle.Title  = MyStringId.GetOrCompute("Manual throttle");
                manual_throttle.Writer = thruster_tagger.throttle_status;
                manual_throttle.SetLimits(0.0f, 100.0f);
                MyAPIGateway.TerminalControls.AddControl<IMyThrust>(manual_throttle);
                create_slider_action<IMyThrust>("IncreaseThrottle", "Increase Manual Throttle",
                    delegate (IMyTerminalBlock thruster)
                    {
                        thruster_tagger.set_manual_throttle(thruster, thruster_tagger.get_manual_throttle(thruster) + 5.0f);
                    },
                    thruster_tagger.throttle_status, "Increase");
                create_slider_action<IMyThrust>("DecreaseThrottle", "Decrease Manual Throttle",
                    delegate (IMyTerminalBlock thruster)
                    {
                        thruster_tagger.set_manual_throttle(thruster, thruster_tagger.get_manual_throttle(thruster) - 5.0f);
                    },
                    thruster_tagger.throttle_status, "Decrease");

                _panel_controls_set = true;
                _sample_thruster    = null;
                _sample_controller  = null;
            }
        }

        public override void UpdateBeforeSimulation()
        {
            base.UpdateBeforeSimulation();

            sync_helper.handle_60Hz();
            if (_grids_handle_60Hz != null)
                _grids_handle_60Hz();
            if (--_count15 <= 0)
            {
                _count15 = 15;
                if (_grids_handle_4Hz != null)
                    _grids_handle_4Hz();
                if (--_count8 <= 0)
                {
                    _count8 = 8;
                    try_register_handlers();
                    if (_grids_handle_2s_period != null)
                        _grids_handle_2s_period();
                }
            }
        }

        protected override void UnloadData()
        {
            base.UnloadData();
            if (sync_helper.network_handlers_registered)
                sync_helper.deregister_handlers();
            foreach (var leftover_grid in _grids.Keys.ToList())
                on_entity_removed(leftover_grid);
            MyAPIGateway.Entities.OnEntityAdd    -= on_entity_added;
            MyAPIGateway.Entities.OnEntityRemove -= on_entity_removed;
        }
    }
}
