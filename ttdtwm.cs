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
using VRageMath;
using PB = Sandbox.ModAPI.Ingame;

namespace ttdtwm
{
    [MySessionComponentDescriptor(MyUpdateOrder.BeforeSimulation | MyUpdateOrder.AfterSimulation)]
    public class session_handler: MySessionComponentBase
    {
        public const bool SINGLE_THREADED_EXEC = false;
        
        #region fields

        private readonly Dictionary<IMyCubeGrid, grid_logic> _grids = new Dictionary<IMyCubeGrid, grid_logic>();
        private readonly HashSet<IMyCubeGrid> _examined_grids = new HashSet<IMyCubeGrid>(), _inactive_grids = new HashSet<IMyCubeGrid>();
        private readonly List<IMyCubeGrid> _grids_to_move = new List<IMyCubeGrid>();
        private readonly List<List<grid_logic>> _connected_grid_lists = new List<List<grid_logic>>();
        private int _num_connected_grid_lists = 0;

        private Action _grids_handle_60Hz = null, _grids_handle_4Hz_foreground = null, _grids_handle_2s_period_foreground = null;
        private Action _grids_handle_4Hz_background = null, _grids_handle_2s_period_background = null, _grids_perform_calibration = null;
        private Task   _manager_task, _calibration_task;

        private static IMyThrust            _sample_thruster   = null;
        private static IMyShipController    _sample_controller = null;

        private int  _count15 = 15, _count8_foreground = 8, _count8_background = 8;
        private bool _setup_complete = false, _entity_events_set = false, _retry_registration = true;

        private static bool _thruster_controls_set = false, _ship_controller_controls_set = false;

        private static session_handler _session_ref;

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
                _inactive_grids.Add(grid);
                return;
            }
        }

        private void on_entity_removed(IMyEntity entity)
        {
            var grid = entity as IMyCubeGrid;
            if (grid != null)
            {
                deactivate_grid(grid);
                _inactive_grids.Remove(grid);
                return;
            }
        }

        #endregion

        #region UI helpers

        private void create_toggle<_block_>(string id, string title, string enabled_text, string disabled_text, Action<IMyTerminalBlock> action, 
            Func<IMyTerminalBlock, bool> getter, Func<IMyTerminalBlock, bool> state, string icon)
        {
            IMyTerminalAction toggle_action = MyAPIGateway.TerminalControls.CreateAction<_block_>(id);

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
            IMyTerminalControlCheckbox panel_checkbox = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlCheckbox, _block_>(id);

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
            IMyTerminalControlOnOffSwitch panel_switch = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlOnOffSwitch, _block_>(id);

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

        private void create_button<_block_>(string id, string title, string tooltip, string action_prefix, Action<IMyTerminalBlock> button_function, 
            Func<IMyTerminalBlock, bool> state, Action<IMyTerminalBlock, StringBuilder> status)
        {
            IMyTerminalControlButton new_button    = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlButton, _block_>(id);
            IMyTerminalAction        button_action = MyAPIGateway.TerminalControls.CreateAction<_block_>(id + "Activate");

            new_button.Title   = MyStringId.GetOrCompute(title);
            button_action.Name = new StringBuilder(action_prefix + " " + title);
            if (tooltip != null)
                new_button.Tooltip = MyStringId.GetOrCompute(tooltip);
            new_button.Action = button_action.Action = button_function;
            if (state != null)
                new_button.Enabled = state;
            new_button.SupportsMultipleBlocks = button_action.ValidForGroups = false;
            button_action.Icon   = @"Textures\GUI\Icons\Actions\MissileToggle.dds";
            button_action.Writer = status;
            MyAPIGateway.TerminalControls.AddControl<_block_>(   new_button);
            MyAPIGateway.TerminalControls.AddAction <_block_>(button_action);
        }

        private void create_slider_action<_block_>(string id, string title, Action<IMyTerminalBlock> action, Action<IMyTerminalBlock, StringBuilder> status, string icon)
        {
            IMyTerminalAction toggle_action = MyAPIGateway.TerminalControls.CreateAction<_block_>(id);

            toggle_action.Action = action;
            if (icon != null && icon != "")
                toggle_action.Icon = @"Textures\GUI\Icons\Actions\" + icon + ".dds";
            toggle_action.Name           = new StringBuilder(title);
            toggle_action.ValidForGroups = true;
            toggle_action.Writer         = status;
            MyAPIGateway.TerminalControls.AddAction<_block_>(toggle_action);
        }

        private void create_PB_property<_type_, _block_>(string id, Func<IMyTerminalBlock, _type_> getter, Action<IMyTerminalBlock, _type_> setter = null)
        {
            IMyTerminalControlProperty<_type_> new_property = MyAPIGateway.TerminalControls.CreateProperty<_type_, _block_>(id);
            new_property.Getter = getter;
            if (setter != null)
                new_property.Setter = setter;
            MyAPIGateway.TerminalControls.AddControl<_block_>(new_property);
        }

        #endregion

        private void activate_grid(IMyCubeGrid grid)
        {
            if (_inactive_grids.Contains(grid))
            {
                _inactive_grids.Remove(grid);
                var new_grid_logic = new grid_logic(grid);
                _grids_handle_60Hz                 += new_grid_logic.handle_60Hz;
                _grids_handle_4Hz_foreground       += new_grid_logic.handle_4Hz_foreground;
                _grids_handle_2s_period_foreground += new_grid_logic.handle_2s_period_foreground;
                _grids_handle_4Hz_background       += new_grid_logic.handle_4Hz_background;
                _grids_handle_2s_period_background += new_grid_logic.handle_2s_period_background;
                _grids_perform_calibration         += new_grid_logic.perform_individual_calibration;
                _grids.Add(grid, new_grid_logic);
            }
        }

        private void deactivate_grid(IMyCubeGrid grid)
        {
            if (_grids.ContainsKey(grid))
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
                _inactive_grids.Add(grid);
            }
        }

        private void disable_inactive_grids()
        {
            List<IMyCubeGrid> grids_to_disable = _grids_to_move;

            grids_to_disable.Clear();
            foreach (IMyCubeGrid grid in _grids.Keys)
            {
                if (grid.IsStatic || grid.Physics == null || !grid.Physics.Enabled)
                    grids_to_disable.Add(grid);
            }
            foreach (IMyCubeGrid grid in grids_to_disable)
                deactivate_grid(grid);
        }

        private void enable_active_grids()
        {
            List<IMyCubeGrid> grids_to_enable = _grids_to_move;

            grids_to_enable.Clear();
            foreach (IMyCubeGrid grid in _inactive_grids)
            {
                if (!grid.IsStatic && grid.Physics != null && grid.Physics.Enabled)
                    grids_to_enable.Add(grid);
            }
            foreach (IMyCubeGrid grid in grids_to_enable)
                activate_grid(grid);
        }

        private static void clear_grid_secondary_flag(List<grid_logic> secondary_grids)
        {
            foreach (grid_logic cur_grid_object in secondary_grids)
                cur_grid_object.is_secondary = false;
        }

        internal static void get_secondary_grids(IMyCubeGrid primary, ref List<grid_logic> secondary_grids)
        {
            List      <IMyCubeGrid>             connected_grid_list = MyAPIGateway.GridGroups.GetGroup(primary, GridLinkTypeEnum.Logical);
            Dictionary<IMyCubeGrid, grid_logic> grid_list           = _session_ref._grids;

            grid_list[primary].is_secondary = false;
            if (connected_grid_list.Count <= 1 || !((MyCubeGrid) primary).HasMainCockpit())
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
            foreach (IMyCubeGrid cur_grid in connected_grid_list)
            {
                if (((MyCubeGrid) cur_grid).HasMainCockpit())
                    ++primary_count;
                else if (cur_grid != primary)
                {
                    grid_logic cur_grid_object;
                    if (grid_list.TryGetValue(cur_grid, out cur_grid_object))
                    {
                        secondary_grids.Add(cur_grid_object);
                        cur_grid_object.is_secondary = true;
                    }
                }
            }
            if (primary_count != 1)
            {
                clear_grid_secondary_flag(secondary_grids);
                secondary_grids = null;
            }
        }

        private void refresh_connected_grids()
        {
            HashSet<IMyCubeGrid> examined_grids = _examined_grids;
            List<List<grid_logic>> connected_grid_lists = _connected_grid_lists;

            examined_grids.Clear();
            _num_connected_grid_lists = 0;
            foreach (IMyCubeGrid grid in _grids.Keys)
            {
                if (examined_grids.Contains(grid))
                    continue;
                MyPhysicsComponentBase grid_body = grid.Physics;

                List<IMyCubeGrid> grid_list     = MyAPIGateway.GridGroups.GetGroup(grid, GridLinkTypeEnum.Physical);
                Vector3D          static_moment = Vector3D.Zero;
                float             cur_mass;
                
                float total_mass = 0.0f;
                foreach (IMyCubeGrid cur_grid in grid_list)
                {
                    examined_grids.Add(cur_grid);
                    if (cur_grid.IsStatic)
                        continue;
                    grid_body = cur_grid.Physics;
                    if (grid_body == null || !grid_body.Enabled)
                        continue;
                    cur_mass       = grid_body.Mass;
                    static_moment += cur_mass * grid_body.CenterOfMassWorld;
                    total_mass    += cur_mass;
                }

                List<grid_logic> connected_grids;
                if (connected_grid_lists.Count > _num_connected_grid_lists)
                {
                    connected_grids = connected_grid_lists[_num_connected_grid_lists];
                    connected_grids.Clear();
                }
                else
                {
                    connected_grids = new List<grid_logic>();
                    connected_grid_lists.Add(connected_grids);
                }
                ++_num_connected_grid_lists;

                if (total_mass < 1.0f)
                {
                    foreach (IMyCubeGrid cur_grid in grid_list)
                    {
                        grid_logic grid_object;
                        if (!_grids.TryGetValue(cur_grid, out grid_object))
                            continue;
                        grid_object.set_grid_CoM(cur_grid.Physics.CenterOfMassWorld);
                        grid_object.set_average_connected_grid_mass(cur_grid.Physics.Mass);
                        connected_grids.Add(grid_object);
                    }
                }
                else
                {
                    Vector3D world_combined_CoM = static_moment / total_mass;
                    float    average_mass       = total_mass    / grid_list.Count;
                    foreach (IMyCubeGrid cur_grid in grid_list)
                    {
                        grid_logic grid_object;
                        if (!_grids.TryGetValue(cur_grid, out grid_object))
                            continue;
                        grid_object.set_grid_CoM(world_combined_CoM);
                        grid_object.set_average_connected_grid_mass(average_mass);
                        connected_grids.Add(grid_object);
                    }
                }
            }
        }

        internal static void sample_thruster(IMyThrust thruster)
        {
            if (!_thruster_controls_set && _sample_thruster == null)
                _sample_thruster = thruster;
        }

        internal static void sample_controller(IMyShipController controller)
        {
            if (!_ship_controller_controls_set && _sample_controller == null)
                _sample_controller = controller;
        }

        private void create_controller_widgets<_controller_type_>()
        {
            IMyTerminalControlSeparator controller_line = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlSeparator, _controller_type_>("TTDTWM_LINE1");
            MyAPIGateway.TerminalControls.AddControl<_controller_type_>(controller_line);
            create_checkbox<_controller_type_>(    "RotationalDamping",        "Rotational Damping", null,                   "On",    "Off", thruster_and_grid_tagger.is_grid_rotational_damping_on, thruster_and_grid_tagger.set_grid_rotational_damping, thruster_and_grid_tagger.is_grid_control_available       );
            create_switch  <_controller_type_>(              "CoTMode",  "Active Control Reference", null,  "CoT",  "CoM",  "CoT",    "CoM", thruster_and_grid_tagger.is_grid_CoT_mode_on          , thruster_and_grid_tagger.set_grid_CoT_mode          , thruster_and_grid_tagger.is_grid_control_available       );
            create_switch  <_controller_type_>("IndividualCalibration", "Thrust Calibration Method", null, "Ind.", "Quad", "Ind.",   "Quad", thruster_and_grid_tagger.use_individual_calibration   , thruster_and_grid_tagger.choose_calibration_method  , thruster_and_grid_tagger.is_grid_control_available       );
            create_switch  <_controller_type_>(          "LandingMode",            "Touchdown Mode", null,   "On",  "Off", "Land", "Flight", thruster_and_grid_tagger.is_grid_touchdown_mode_on    , thruster_and_grid_tagger.set_grid_touchdown_mode    , thruster_and_grid_tagger.is_grid_touchdown_mode_available);

            IMyTerminalControlSeparator controller_line2 = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlSeparator, _controller_type_>("TTDTWM_LINE2");
            MyAPIGateway.TerminalControls.AddControl<_controller_type_>(controller_line2);
            IMyTerminalControlLabel controller_line_ID_override_label = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlLabel, _controller_type_>("TTDTWM_ID_OVR");
            controller_line_ID_override_label.Label = MyStringId.GetOrCompute("Inertia Damper Overrides");
            MyAPIGateway.TerminalControls.AddControl<_controller_type_>(controller_line_ID_override_label);
            create_checkbox<_controller_type_>(      "ForeAftIDDisable", "Disable fore/aft"      , null, "On", "Off", thruster_and_grid_tagger.create_damper_override_reader(2), thruster_and_grid_tagger.create_damper_override_setter(2), thruster_and_grid_tagger.is_grid_control_available);
            create_checkbox<_controller_type_>("PortStarboardIDDisable", "Disable port/starboard", null, "On", "Off", thruster_and_grid_tagger.create_damper_override_reader(0), thruster_and_grid_tagger.create_damper_override_setter(0), thruster_and_grid_tagger.is_grid_control_available);
            create_checkbox<_controller_type_>("DorsalVentralIDDisable", "Disable dorsal/ventral", null, "On", "Off", thruster_and_grid_tagger.create_damper_override_reader(1), thruster_and_grid_tagger.create_damper_override_setter(1), thruster_and_grid_tagger.is_grid_control_available);
        }

        private void try_register_handlers()
        {
            sync_helper.try_register_handlers();
            screen_info.try_register_handlers();
            if (!_entity_events_set && screen_info.settings_loaded && MyAPIGateway.Entities != null)
            {
                var existing_entities = new HashSet<IMyEntity>();
                MyAPIGateway.Entities.GetEntities(existing_entities);
                foreach (IMyEntity cur_entity in existing_entities)
                    on_entity_added(cur_entity);
                MyAPIGateway.Entities.OnEntityAdd    += on_entity_added;
                MyAPIGateway.Entities.OnEntityRemove += on_entity_removed;
                _entity_events_set = true;
            }
            
            if (!screen_info.settings_loaded || MyAPIGateway.TerminalControls == null)
                return;
            
            if (!_ship_controller_controls_set)
            {
                if (_sample_controller?.GetProperty("DampenersOverride") == null)
                    return;
                create_controller_widgets<IMyCockpit>();
                create_controller_widgets<IMyRemoteControl>();
                _ship_controller_controls_set = true;
                _sample_controller            = null;
            }
            
            if (!_thruster_controls_set)
            {
                if (_sample_thruster?.GetProperty("Override") == null)
                    return;
                _thruster_controls_set = true;
                _sample_thruster       = null;
                IMyTerminalControlSeparator thruster_line = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlSeparator, IMyThrust>("TTDTWM_LINE1");
                MyAPIGateway.TerminalControls.AddControl<IMyThrust>(thruster_line);
                create_switch  <IMyThrust>(     "ActiveControl",             "Steering", null, "On", "Off", "On", "Off", thruster_and_grid_tagger.is_under_active_control, thruster_and_grid_tagger.set_active_control , thruster_and_grid_tagger.is_active_control_available);
                create_switch  <IMyThrust>(          "AntiSlip",      "Thrust Trimming", null, "On", "Off", "On", "Off", thruster_and_grid_tagger.is_anti_slip           , thruster_and_grid_tagger.set_anti_slip      , thruster_and_grid_tagger.is_anti_slip_available     );
                create_checkbox<IMyThrust>("DisableLinearInput", "Disable linear input", null, "On", "Off",              thruster_and_grid_tagger.is_rotational_only     , thruster_and_grid_tagger.toggle_linear_input, thruster_and_grid_tagger.is_active_control_available);
                create_switch  <IMyThrust>(       "StaticLimit",       "Thrust Limiter", null, "On", "Off", "On", "Off", thruster_and_grid_tagger.is_thrust_limiter_on   , thruster_and_grid_tagger.set_thrust_limiter , thruster_and_grid_tagger.is_thrust_limiter_available);

                IMyTerminalControlSlider manual_throttle = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlSlider, IMyThrust>("ManualThrottle");
                manual_throttle.Getter = thruster_and_grid_tagger.get_manual_throttle;
                manual_throttle.Setter = thruster_and_grid_tagger.set_manual_throttle;
                manual_throttle.SupportsMultipleBlocks = true;
                manual_throttle.Title  = MyStringId.GetOrCompute("Manual throttle");
                manual_throttle.Writer = thruster_and_grid_tagger.throttle_status;
                manual_throttle.SetLimits(0.0f, 100.0f);
                MyAPIGateway.TerminalControls.AddControl<IMyThrust>(manual_throttle);
                create_slider_action<IMyThrust>("IncreaseThrottle", "Increase Manual Throttle",
                    delegate (IMyTerminalBlock thruster)
                    {
                        thruster_and_grid_tagger.set_manual_throttle(thruster, thruster_and_grid_tagger.get_manual_throttle(thruster) + 5.0f);
                    },
                    thruster_and_grid_tagger.throttle_status, "Increase");
                create_slider_action<IMyThrust>("DecreaseThrottle", "Decrease Manual Throttle",
                    delegate (IMyTerminalBlock thruster)
                    {
                        thruster_and_grid_tagger.set_manual_throttle(thruster, thruster_and_grid_tagger.get_manual_throttle(thruster) - 5.0f);
                    },
                    thruster_and_grid_tagger.throttle_status, "Decrease");
                create_PB_property<float, IMyThrust>("BalancedLevel", thruster_and_grid_tagger.get_thrust_limit);
            }

            _setup_complete = _ship_controller_controls_set && _thruster_controls_set && _entity_events_set
                && screen_info.settings_loaded && sync_helper.network_handlers_registered;
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

        public override void UpdateBeforeSimulation()
        {
            base.UpdateBeforeSimulation();

            screen_info.refresh_local_player_info();
            if (_grids_handle_60Hz == null)
            {
                enable_active_grids();
                return;
            }

            disable_inactive_grids();
            if (--_count15 <= 0)
            {
                _count15 = 15;
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
                    _count8_foreground  = 8;
                    _retry_registration = true;
                    enable_active_grids();
                    _grids_handle_2s_period_foreground();
                }
                screen_info.refresh_local_player_HUD();
                refresh_connected_grids();
                thruster_and_grid_tagger.handle_4Hz();
                _grids_handle_4Hz_foreground();
            }
            _grids_handle_60Hz();
        }

        public override void UpdateAfterSimulation()
        {
            base.UpdateAfterSimulation();
            if (!_setup_complete && _retry_registration)
            {
                try_register_handlers();
                _retry_registration = false;
            }
        }

        public session_handler()
        {
            _session_ref = this;
        }

        protected override void UnloadData()
        {
            base.UnloadData();
            MyAPIGateway.Entities.OnEntityAdd    -= on_entity_added;
            MyAPIGateway.Entities.OnEntityRemove -= on_entity_removed;
            sync_helper.deregister_handlers();
            screen_info.deregister_handlers();
            foreach (IMyCubeGrid leftover_grid in _grids.Keys.ToList())
                on_entity_removed(leftover_grid);
            _sample_controller  = null;
            _sample_thruster    = null;
            _setup_complete     = _entity_events_set = _ship_controller_controls_set = _thruster_controls_set = false;
            _session_ref        = null;
        }
    }
}
