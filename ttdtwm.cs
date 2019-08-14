using ParallelTasks;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using Sandbox.Game.Entities;
using Sandbox.ModAPI;
using Sandbox.ModAPI.Interfaces;
using Sandbox.ModAPI.Interfaces.Terminal;
using VRage.Game.Components;
using VRage.Game.ModAPI;
using VRage.ModAPI;
using VRage.Utils;
using PB = Sandbox.ModAPI.Ingame;

namespace ttdtwm
{
    [MySessionComponentDescriptor(MyUpdateOrder.AfterSimulation)]
    public class session_handler: MySessionComponentBase
    {
        public const bool SINGLE_THREADED_EXEC = false;
        
        #region fields

        private Dictionary<IMyCubeGrid, grid_logic> _grids = new Dictionary<IMyCubeGrid, grid_logic>();
        private Action _grids_handle_60Hz = null, _grids_handle_4Hz_foreground = null, _grids_handle_2s_period_foreground = null;
        private Action _grids_handle_4Hz_background = null, _grids_handle_2s_period_background = null, _grids_perform_calibration = null;
        private Task _manager_task, _calibration_task;

        private IMyThrust         _sample_thruster   = null;
        private IMyShipController _sample_controller = null;

        private int  _count15 = 15, _count8_foreground = 8, _count8_background = 8;
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
                _grids_handle_60Hz                 += new_grid_logic.handle_60Hz;
                _grids_handle_4Hz_foreground       += new_grid_logic.handle_4Hz_foreground;
                _grids_handle_2s_period_foreground += new_grid_logic.handle_2s_period_foreground;
                _grids_handle_4Hz_background       += new_grid_logic.handle_4Hz_background;
                _grids_handle_2s_period_background += new_grid_logic.handle_2s_period_background;
                _grids_perform_calibration         += new_grid_logic.perform_individual_calibration;
                _grids.Add(grid, new_grid_logic);
            }

            var planetoid = entity as MyPlanet;
            if (planetoid != null)
                gravity_and_physics.register_gravity_source(planetoid);
        }

        private void on_entity_removed(IMyEntity entity)
        {
            var grid = entity as IMyCubeGrid;
            if (grid != null && _grids.ContainsKey(grid))
            {
                grid_logic grid_logic_to_remove = _grids[grid];
                _grids_handle_60Hz                 -= grid_logic_to_remove.handle_60Hz;
                _grids_handle_4Hz_foreground       -= grid_logic_to_remove.handle_4Hz_foreground;
                _grids_handle_2s_period_foreground -= grid_logic_to_remove.handle_2s_period_foreground;
                _grids_handle_4Hz_background       -= grid_logic_to_remove.handle_4Hz_background;
                _grids_handle_2s_period_background -= grid_logic_to_remove.handle_2s_period_background;
                _grids_perform_calibration         -= grid_logic_to_remove.perform_individual_calibration;
                grid_logic_to_remove.Dispose();
                _grids.Remove(grid);
            }

            var planetoid = entity as MyPlanet;
            if (planetoid != null)
                gravity_and_physics.deregister_gravity_source(planetoid);
        }

        private bool is_grid_CoT_mode_on(IMyTerminalBlock controller)
        {
            return _grids[controller.CubeGrid].CoT_mode_on;
        }

        private bool is_grid_CoT_mode_available(IMyTerminalBlock controller)
        {
            if (!((PB.IMyShipController) controller).ControlThrusters)
                return false;

            IMyCubeGrid grid = controller.CubeGrid;
            if (((MyCubeGrid) grid).HasMainCockpit() && !((PB.IMyShipController) controller).IsMainCockpit)
                return false;
            return _grids[grid].is_CoT_mode_available;
        }

        private void set_grid_CoT_mode(IMyTerminalBlock controller, bool new_state)
        {
            _grids[controller.CubeGrid].CoT_mode_on = new_state;
        }

        private bool use_individual_calibration(IMyTerminalBlock controller)
        {
            return _grids[controller.CubeGrid].use_individual_calibration;
        }

        private void choose_calibration_method(IMyTerminalBlock controller, bool use_individual_calibration)
        {
            _grids[controller.CubeGrid].use_individual_calibration = use_individual_calibration;
        }

        private bool is_grid_rotational_damping_on(IMyTerminalBlock controller)
        {
            return _grids[controller.CubeGrid].rotational_damping_on;
        }

        private void set_grid_rotational_damping(IMyTerminalBlock controller, bool new_state)
        {
            _grids[controller.CubeGrid].rotational_damping_on = new_state;
        }

        private bool is_grid_landing_mode_on(IMyTerminalBlock controller)
        {
            return _grids[controller.CubeGrid].landing_mode_on;
        }

        private bool is_grid_landing_mode_available(IMyTerminalBlock controller)
        {
            return is_grid_CoT_mode_available(controller) && _grids[controller.CubeGrid].is_landing_mode_available;
        }

        private void set_grid_landing_mode(IMyTerminalBlock controller, bool new_state)
        {
            _grids[controller.CubeGrid].landing_mode_on = new_state;
        }

        private Func<IMyTerminalBlock, bool> create_damper_override_reader(int axis)
        {
            return delegate(IMyTerminalBlock controller)
            {
                return _grids[controller.CubeGrid].is_ID_axis_overriden(controller, axis);
            };
        }

        private Action<IMyTerminalBlock, bool> create_damper_override_setter(int axis)
        {
            return delegate(IMyTerminalBlock controller, bool new_state)
            {
                _grids[controller.CubeGrid].set_ID_override(controller, axis, new_state);
            };
        }

        private bool is_grid_circularise_mode_available(IMyTerminalBlock controller)
        {
            return is_grid_CoT_mode_available(controller) && _grids[controller.CubeGrid].is_circularisation_avaiable && !_grids[controller.CubeGrid].circularise;
        }

        private bool is_grid_full_stop_mode_available(IMyTerminalBlock controller)
        {
            return is_grid_CoT_mode_available(controller) && _grids[controller.CubeGrid].circularise;
        }

        private void select_grid_full_stop_mode(IMyTerminalBlock controller)
        {
            _grids[controller.CubeGrid].circularise = false;
        }

        private void select_grid_circularisation_mode(IMyTerminalBlock controller)
        {
            _grids[controller.CubeGrid].circularise = true;
        }

        private bool is_grid_maneuvre_available(IMyTerminalBlock controller)
        {
            return is_grid_CoT_mode_available(controller) && _grids[controller.CubeGrid].is_circularisation_avaiable;
        }

        private Action<IMyTerminalBlock> create_maneuvre_starter(engine_control_unit.ID_maneuvres maneuvre)
        {
            return delegate (IMyTerminalBlock controller)
            {
                _grids[controller.CubeGrid].start_maneuvre(maneuvre);
            };
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

            create_toggle<_block_>(id + "OnOff", title + " " + enabled_text + "/" + disabled_text, toolbar_enabled_text, toolbar_disabled_text,
                delegate (IMyTerminalBlock block)
                {
                    setter(block, !getter(block));
                },
                getter, state, "MissileToggle");
            create_toggle<_block_>(id + "OnOff_On", title + " " + enabled_text, toolbar_enabled_text, toolbar_disabled_text,
                delegate (IMyTerminalBlock block)
                {
                    setter(block, true);
                },
                getter, state, "MissileSwitchOn");
            create_toggle<_block_>(id + "OnOff_Off", title + " " + disabled_text, toolbar_enabled_text, toolbar_disabled_text,
                delegate (IMyTerminalBlock block)
                {
                    setter(block, false);
                },
                getter, state, "MissileSwitchOff");
        }

        private void create_button<_block_>(string id, string title, string tooltip, string action_prefix, Action<IMyTerminalBlock> button_function, Func<IMyTerminalBlock, bool> state)
        {
            var new_button    = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlButton, _block_>(id);
            var button_action = MyAPIGateway.TerminalControls.CreateAction<_block_>(id + "Activate");

            new_button.Title   = MyStringId.GetOrCompute(title);
            button_action.Name = new StringBuilder(action_prefix + " " + title);
            if (tooltip != null)
                new_button.Tooltip = MyStringId.GetOrCompute(tooltip);
            new_button.Action = button_action.Action = button_function;
            if (state != null)
                new_button.Enabled = button_action.Enabled = state;
            new_button.SupportsMultipleBlocks = button_action.ValidForGroups = false;
            button_action.Icon = @"Textures\GUI\Icons\Actions\MissileToggle.dds";
            MyAPIGateway.TerminalControls.AddControl<_block_>(   new_button);
            MyAPIGateway.TerminalControls.AddAction <_block_>(button_action);
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

        private void clear_grid_secondary_flag(List<grid_logic> secondary_grids)
        {
            foreach (var cur_grid_object in secondary_grids)
                cur_grid_object.is_secondary = false;
        }

        internal void get_secondary_grids(IMyCubeGrid primary, ref List<grid_logic> secondary_grids)
        {
            List<IMyCubeGrid> grid_list = MyAPIGateway.GridGroups.GetGroup(primary, GridLinkTypeEnum.Logical);

            _grids[primary].is_secondary = false;
            if (grid_list.Count <= 1 || !((MyCubeGrid) primary).HasMainCockpit())
            {
                if (secondary_grids != null)
                    clear_grid_secondary_flag(secondary_grids);
                secondary_grids = null;
                return;
            }
            if (secondary_grids == null)
                secondary_grids = new List<grid_logic>();
            else
            {
                clear_grid_secondary_flag(secondary_grids);
                secondary_grids.Clear();
            }

            int primary_count = 0;
            foreach (var cur_grid in grid_list)
            {
                if (((MyCubeGrid) cur_grid).HasMainCockpit())
                    ++primary_count;
                else if (cur_grid != primary)
                {
                    grid_logic cur_grid_object = _grids[cur_grid];
                    secondary_grids.Add(cur_grid_object);
                    cur_grid_object.is_secondary = true;
                }
            }
            if (primary_count != 1)
            {
                clear_grid_secondary_flag(secondary_grids);
                secondary_grids = null;
            }
        }

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

        private void create_controller_widgets<_controller_type_>()
        {
            var controller_line = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlSeparator, _controller_type_>("TTDTWM_LINE1");
            MyAPIGateway.TerminalControls.AddControl<_controller_type_>(controller_line);
            create_checkbox<_controller_type_>(    "RotationalDamping",        "Rotational Damping", null,                   "On",    "Off", is_grid_rotational_damping_on, set_grid_rotational_damping,     is_grid_CoT_mode_available);
            create_switch  <_controller_type_>(              "CoTMode",  "Active Control Reference", null,  "CoT",  "CoM",  "CoT",    "CoM",           is_grid_CoT_mode_on,           set_grid_CoT_mode,     is_grid_CoT_mode_available);
            create_switch  <_controller_type_>("IndividualCalibration", "Thrust Calibration Method", null, "Ind.", "Quad", "Ind.",   "Quad",    use_individual_calibration,   choose_calibration_method,     is_grid_CoT_mode_available);
            create_switch  <_controller_type_>(          "LandingMode",            "Touchdown Mode", null,   "On",  "Off", "Land", "Flight",       is_grid_landing_mode_on,       set_grid_landing_mode, is_grid_landing_mode_available);

            var controller_line2 = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlSeparator, _controller_type_>("TTDTWM_LINE2");
            MyAPIGateway.TerminalControls.AddControl<_controller_type_>(controller_line2);
            var controller_line_ID_override_label = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlLabel, _controller_type_>("TTDTWM_ID_OVR");
            controller_line_ID_override_label.Label = MyStringId.GetOrCompute("Inertia Damper Overrides");
            MyAPIGateway.TerminalControls.AddControl<_controller_type_>(controller_line_ID_override_label);
            create_checkbox<_controller_type_>(      "ForeAftIDDisable", "Disable fore/aft"      , null, "On", "Off", create_damper_override_reader(2), create_damper_override_setter(2), is_grid_CoT_mode_available);
            create_checkbox<_controller_type_>("PortStarboardIDDisable", "Disable port/starboard", null, "On", "Off", create_damper_override_reader(0), create_damper_override_setter(0), is_grid_CoT_mode_available);
            create_checkbox<_controller_type_>("DorsalVentralIDDisable", "Disable dorsal/ventral", null, "On", "Off", create_damper_override_reader(1), create_damper_override_setter(1), is_grid_CoT_mode_available);

            var controller_line_ID_mode   = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlLabel, _controller_type_>("TTDTWM_IDMODE");
            controller_line_ID_mode.Label = MyStringId.GetOrCompute("Inertia Damper Mode");
            MyAPIGateway.TerminalControls.AddControl<_controller_type_>(controller_line_ID_mode);
            create_button<_controller_type_>("IDFullStop"   ,   "Full Stop", null, "Select", select_grid_full_stop_mode      , is_grid_full_stop_mode_available  );
            create_button<_controller_type_>("IDCircularise", "Circularise", null, "Select", select_grid_circularisation_mode, is_grid_circularise_mode_available);

            var controller_line_maneuvre   = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlLabel, _controller_type_>("TTDTWM_IDMANEUVRE");
            controller_line_maneuvre.Label = MyStringId.GetOrCompute("Maneuvres");
            MyAPIGateway.TerminalControls.AddControl<_controller_type_>(controller_line_maneuvre);
            create_button<_controller_type_>("IDPrograde"  ,    "Prograde", null, "Start", create_maneuvre_starter(engine_control_unit.ID_maneuvres.burn_prograde  ), is_grid_maneuvre_available);
            create_button<_controller_type_>("IDRetrograde",  "Retrograde", null, "Start", create_maneuvre_starter(engine_control_unit.ID_maneuvres.burn_retrograde), is_grid_maneuvre_available);
            create_button<_controller_type_>("IDNormal"    ,      "Normal", null, "Start", create_maneuvre_starter(engine_control_unit.ID_maneuvres.burn_normal    ), is_grid_maneuvre_available);
            create_button<_controller_type_>("IDAntiNormal", "Anti-normal", null, "Start", create_maneuvre_starter(engine_control_unit.ID_maneuvres.burn_antinormal), is_grid_maneuvre_available);
            create_button<_controller_type_>("IDOutward"   ,     "Outward", null, "Start", create_maneuvre_starter(engine_control_unit.ID_maneuvres.burn_outward   ), is_grid_maneuvre_available);
            create_button<_controller_type_>("IDInward"    ,      "Inward", null, "Start", create_maneuvre_starter(engine_control_unit.ID_maneuvres.burn_inward    ), is_grid_maneuvre_available);
        }

        private void try_register_handlers()
        {
            sync_helper.try_register_handlers();
            screen_info.try_register_handlers();
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

                create_controller_widgets<IMyCockpit>();
                create_controller_widgets<IMyRemoteControl>();

                var thruster_line = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlSeparator, IMyThrust>("TTDTWM_LINE1");
                MyAPIGateway.TerminalControls.AddControl<IMyThrust>(thruster_line);
                create_switch  <IMyThrust>(     "ActiveControl",             "Steering", null, "On", "Off", "On", "Off", thruster_tagger.is_under_active_control, thruster_tagger.set_active_control , thruster_tagger.is_active_control_available);
                create_switch  <IMyThrust>(          "AntiSlip",      "Thrust Trimming", null, "On", "Off", "On", "Off", thruster_tagger.is_anti_slip           , thruster_tagger.set_anti_slip      , thruster_tagger.is_anti_slip_available     );
                create_checkbox<IMyThrust>("DisableLinearInput", "Disable linear input", null, "On", "Off",              thruster_tagger.is_rotational_only     , thruster_tagger.toggle_linear_input, thruster_tagger.is_active_control_available);
                create_switch  <IMyThrust>(       "StaticLimit",       "Thrust Limiter", null, "On", "Off", "On", "Off", thruster_tagger.is_thrust_limiter_on   , thruster_tagger.set_thrust_limiter , thruster_tagger.is_thrust_limiter_available);

                var manual_throttle = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlSlider, IMyThrust>("ManualThrottle");
                manual_throttle.Getter  = thruster_tagger.get_manual_throttle;
                manual_throttle.Setter  = thruster_tagger.set_manual_throttle;
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

                IMyTerminalControlProperty<float> balanced_level = MyAPIGateway.TerminalControls.CreateProperty<float, IMyThrust>("BalancedLevel");
                balanced_level.Getter = thruster_tagger.get_thrust_limit;
                MyAPIGateway.TerminalControls.AddControl<IMyThrust>(balanced_level);

                _panel_controls_set = true;
                _sample_thruster    = null;
                _sample_controller  = null;
            }
        }

        private void calibration_thread()
        {
            try
            {
                _grids_perform_calibration();
            }
            catch (Exception err)
            {
                MyLog.Default.WriteLine(err);
                throw;
            }
        }

        private void manager_thread()
        {
            try
            {
                if (--_count8_background <= 0)
                {
                    _count8_background = 8;
                    if (SINGLE_THREADED_EXEC)
                        calibration_thread();
                    else if (!_calibration_task.valid || _calibration_task.IsComplete)
                        _calibration_task = MyAPIGateway.Parallel.Start(calibration_thread);
                    _grids_handle_2s_period_background();
                }
                _grids_handle_4Hz_background();
            }
            catch (Exception err)
            {
                MyLog.Default.WriteLine(err);
                throw;
            }
        }

        public override void UpdateAfterSimulation()
        {
            base.UpdateAfterSimulation();

            screen_info.refresh_local_player_info();
            if (_grids_handle_60Hz == null)
            {
                try_register_handlers();
                return;
            }
            if (--_count15 <= 0)
            {
                _count15 = 15;
                screen_info.refresh_local_player_HUD();
                if (SINGLE_THREADED_EXEC)
                    manager_thread();
                else
                {
                    if (_manager_task.valid && !_manager_task.IsComplete)
                        _manager_task.Wait();
                    _manager_task = MyAPIGateway.Parallel.Start(manager_thread);
                }
                if (--_count8_foreground <= 0)
                {
                    _count8_foreground = 8;
                    _grids_handle_2s_period_foreground();
                    try_register_handlers();
                }
                _grids_handle_4Hz_foreground();
            }
            _grids_handle_60Hz();
        }

        protected override void UnloadData()
        {
            base.UnloadData();
            sync_helper.deregister_handlers();
            screen_info.deregister_handlers();
            foreach (var leftover_grid in _grids.Keys.ToList())
                on_entity_removed(leftover_grid);
            MyAPIGateway.Entities.OnEntityAdd    -= on_entity_added;
            MyAPIGateway.Entities.OnEntityRemove -= on_entity_removed;
        }
    }
}
