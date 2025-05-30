﻿using ParallelTasks;
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

namespace orbiter_SE
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
        private static IMyProgrammableBlock _sample_PB         = null;

        private int  _count15 = 15, _count8_foreground = 8, _count8_background = 8;
        private bool _setup_complete = false, _entity_events_set = false, _retry_registration = true;

        private static bool _thruster_controls_set = false, _ship_controller_controls_set = false, _programmable_block_properties_set = false;

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

            var planetoid = entity as MyPlanet;
            if (planetoid != null)
            {
                gravity_and_physics.register_gravity_source(planetoid);
                return;
            }

            var character = entity as IMyCharacter;
            if (character != null && character.IsPlayer)
            {
                gravity_and_physics.register_player(character);
                return;
            }

            var floating_object = entity as MyFloatingObject;
            if (floating_object != null)
                gravity_and_physics.register_floating_object(floating_object);
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

            var PB = entity as IMyProgrammableBlock;
            if (PB != null)
            {
                gravity_and_physics.dispose_PB(PB);
                return;
            }

            var planetoid = entity as MyPlanet;
            if (planetoid != null)
            {
                gravity_and_physics.deregister_gravity_source(planetoid);
                return;
            }

            var character = entity as IMyCharacter;
            if (character != null && character.IsPlayer)
            {
                gravity_and_physics.deregister_player(character);
                return;
            }

            var floating_object = entity as MyFloatingObject;
            if (floating_object != null)
                gravity_and_physics.deregister_floating_object(floating_object);
        }

        #endregion

        #region PB routines

        private int get_current_ID_mode(IMyTerminalBlock controller)
        {
            grid_logic controller_grid = _grids[controller.CubeGrid];
            engine_control_unit.ID_manoeuvres current_manoeuvre = controller_grid.current_manoeuvre;

            if (current_manoeuvre != engine_control_unit.ID_manoeuvres.manoeuvre_off)
                return (int) current_manoeuvre + 1;
            return controller_grid.circularise ? 1 : 0;
        }

        private Func<double, double, double> get_true_to_mean_converter(IMyTerminalBlock dummy)
        {
            return gravity_and_physics.convert_true_anomaly_to_mean;
        }

        private Func<double, double, double> get_mean_to_true_converter(IMyTerminalBlock dummy)
        {
            return gravity_and_physics.convert_mean_anomaly_to_true;
        }

        private Func<double, double, Vector3D> get_orbit_normal_calculator(IMyTerminalBlock dummy)
        {
            return orbit_elements.calculate_orbit_normal;
        }

        private Func<double, double, double, double, double, double, ValueTuple<double, double>?> get_intersection_calculator(IMyTerminalBlock dummy)
        {
            return gravity_and_physics.compute_orbit_intersections;
        }

        private Func<Vector3D, Vector3D, Vector3D, double, double> get_radius_to_anomaly_converter(IMyTerminalBlock PB)
        {
            return delegate (Vector3D offset, Vector3D normal, Vector3D AN_vector, double argument_of_periapsis)
            {
                return gravity_and_physics.get_true_anomaly(PB, offset, normal, AN_vector, argument_of_periapsis);
            };
        }

        private Func<string, string, bool> get_ship_elements_calculator(IMyTerminalBlock PB)
        {
            return delegate (string reference_name, string grid_name)
            {
                return gravity_and_physics.calculate_elements_for_PB(PB, reference_name, grid_name);
            };
        }

        private Func<string, Vector3D, Vector3D, bool> get_vector_elements_calculator(IMyTerminalBlock PB)
        {
            return delegate (string reference_name, Vector3D radius, Vector3D velocity)
            {
                return gravity_and_physics.calculate_elements_for_PB_from_vectors(PB, reference_name, radius, velocity);
            };
        }

        private Func<string> get_reference_name_fetcher(IMyTerminalBlock PB)
        {
            return delegate()
            {
                return gravity_and_physics.retrieve_reference_name(PB);
            };
        }

        private Action<Vector3D[]> get_vector_fetcher(IMyTerminalBlock PB)
        {
            return delegate (Vector3D[] vector_elements)
            {
                gravity_and_physics.retrieve_primary_vectors(PB, vector_elements);
            };
        }

        private Action<double[]> get_scalar_fetcher(IMyTerminalBlock PB)
        {
            return delegate (double[] scalar_elements)
            {
                gravity_and_physics.retrieve_primary_scalars(PB, scalar_elements);
            };
        }

        private Action<double[]> get_derived_fetcher(IMyTerminalBlock PB)
        {
            return delegate (double[] derived_elements)
            {
                gravity_and_physics.retrieve_derived_elements(PB, derived_elements);
            };
        }

        private Action<double?, double[]> get_positional_fetcher(IMyTerminalBlock PB)
        {
            return delegate (double? true_anomaly, double[] positional_elements)
            {
                gravity_and_physics.retrieve_positional_elements(PB, true_anomaly, positional_elements);
            };
        }

        private Action<double, Vector3D[]> get_state_vector_fetcher(IMyTerminalBlock PB)
        {
            return delegate (double true_anomaly, Vector3D[] positional_vectors)
            {
                gravity_and_physics.retrieve_positional_vectors(PB, true_anomaly, positional_vectors);
            };
        }

        #endregion

        #region UI helpers

        private void create_toggle<_block_>(string id, string title, string enabled_text, string disabled_text, Action<IMyTerminalBlock> action, 
            Func<IMyTerminalBlock, bool> getter, Func<IMyTerminalBlock, bool> state, string icon) where _block_: IMyTerminalBlock
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
            Func<IMyTerminalBlock, bool> getter, Action<IMyTerminalBlock, bool> setter, Func<IMyTerminalBlock, bool> state) where _block_: IMyTerminalBlock
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
            Func<IMyTerminalBlock, bool> getter, Action<IMyTerminalBlock, bool> setter, Func<IMyTerminalBlock, bool> state) where _block_: IMyTerminalBlock
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
            Func<IMyTerminalBlock, bool> state, Action<IMyTerminalBlock, StringBuilder> status) where _block_: IMyTerminalBlock
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

        private void create_slider_action<_block_>(string id, string title, Action<IMyTerminalBlock> action, 
            Action<IMyTerminalBlock, StringBuilder> status, string icon) where _block_: IMyTerminalBlock
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

        private void create_slider<_block_>(string id, string title, Func<IMyTerminalBlock, float> getter, Action<IMyTerminalBlock, float> setter, 
            Action<IMyTerminalBlock, StringBuilder> status, float minimum, float maximum, float change_amount, 
            string increase_action_name, string decrease_action_name, string increase_text, string decrease_text) where _block_: IMyTerminalBlock
        {
            IMyTerminalControlSlider slider = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlSlider, _block_>(id);
            slider.Getter = getter;
            slider.Setter = setter;
            slider.Title  = MyStringId.GetOrCompute(title);
            slider.Writer = status;
            slider.SupportsMultipleBlocks = true;
            slider.SetLimits(minimum, maximum);
            MyAPIGateway.TerminalControls.AddControl<_block_>(slider);
            create_slider_action<_block_>(increase_action_name, increase_text,
                delegate (IMyTerminalBlock block)
                {
                    setter(block, getter(block) + change_amount);
                },
                status, "Increase");
            create_slider_action<_block_>(decrease_action_name, decrease_text,
                delegate (IMyTerminalBlock block)
                {
                    setter(block, getter(block) - change_amount);
                },
                status, "Decrease");
        }

        private void create_PB_property<_type_, _block_>(string id, Func<IMyTerminalBlock, _type_> getter, Action<IMyTerminalBlock, _type_> setter = null)
            where _block_: IMyTerminalBlock
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

        private void synchronise_orbit_stabilisation_for_connected_grids()
        {
            List<List<grid_logic>> connected_grid_lists = _connected_grid_lists;
            int num_connected_grid_lists = _num_connected_grid_lists;

            for (int index = 0; index < num_connected_grid_lists; ++index)
            {
                List<grid_logic> connected_grids = connected_grid_lists[index];
                bool suppress_stabilisation = false;

                foreach (grid_logic cur_grid in connected_grids)
                    suppress_stabilisation |= cur_grid.internal_suppress_stabilisation;
                foreach (grid_logic cur_grid in connected_grids)
                    cur_grid.external_suppress_stabilisation = suppress_stabilisation;
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

        internal static void sample_PB(IMyProgrammableBlock PB)
        {
            if (!_programmable_block_properties_set && _sample_PB == null)
                _sample_PB = PB;
        }

        private void create_controller_widgets<_controller_type_>() where _controller_type_: IMyShipController
        {
            if (!screen_info.torque_disabled)
            { 
                IMyTerminalControlSeparator controller_line = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlSeparator, _controller_type_>("TTDTWM_LINE1");
                MyAPIGateway.TerminalControls.AddControl<_controller_type_>(controller_line);
                create_checkbox<_controller_type_>(    "RotationalDamping",        "Rotational Damping", null,                   "On",    "Off", thruster_and_grid_tagger.is_grid_rotational_damping_on, thruster_and_grid_tagger.set_grid_rotational_damping, thruster_and_grid_tagger.is_grid_control_available       );
                create_switch  <_controller_type_>(              "CoTMode",  "Active Control Reference", null,  "CoT",  "CoM",  "CoT",    "CoM", thruster_and_grid_tagger.is_grid_CoT_mode_on          , thruster_and_grid_tagger.set_grid_CoT_mode          , thruster_and_grid_tagger.is_grid_control_available       );
                create_switch  <_controller_type_>("IndividualCalibration", "Thrust Calibration Method", null, "Ind.", "Quad", "Ind.",   "Quad", thruster_and_grid_tagger.use_individual_calibration   , thruster_and_grid_tagger.choose_calibration_method  , thruster_and_grid_tagger.is_grid_control_available       );
                create_switch  <_controller_type_>(          "LandingMode",            "Touchdown Mode", null,   "On",  "Off", "Land", "Flight", thruster_and_grid_tagger.is_grid_touchdown_mode_on    , thruster_and_grid_tagger.set_grid_touchdown_mode    , thruster_and_grid_tagger.is_grid_touchdown_mode_available);

                create_slider<_controller_type_>("ControlSensitivity", "Thrust Control Sensitivity", 
                    thruster_and_grid_tagger.get_control_sensitivity, thruster_and_grid_tagger.set_control_sensitivity, thruster_and_grid_tagger.control_sensitivity_text, 
                    0.1f, 20.0f, 0.1f, "IncreaseSensitivity", "DecreaseSensitivity", "Increase Control Sensitivity", "Decrease Control Sensitivity");

                IMyTerminalControlSeparator controller_line2 = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlSeparator, _controller_type_>("TTDTWM_LINE2");
                MyAPIGateway.TerminalControls.AddControl<_controller_type_>(controller_line2);
                IMyTerminalControlLabel controller_line_ID_override_label = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlLabel, _controller_type_>("TTDTWM_ID_OVR");
                controller_line_ID_override_label.Label = MyStringId.GetOrCompute("Inertia Damper Overrides");
                MyAPIGateway.TerminalControls.AddControl<_controller_type_>(controller_line_ID_override_label);
                create_checkbox<_controller_type_>(      "ForeAftIDDisable", "Disable fore/aft"      , null, "On", "Off", thruster_and_grid_tagger.create_damper_override_reader(2), thruster_and_grid_tagger.create_damper_override_setter(2), thruster_and_grid_tagger.is_grid_control_available);
                create_checkbox<_controller_type_>("PortStarboardIDDisable", "Disable port/starboard", null, "On", "Off", thruster_and_grid_tagger.create_damper_override_reader(0), thruster_and_grid_tagger.create_damper_override_setter(0), thruster_and_grid_tagger.is_grid_control_available);
                create_checkbox<_controller_type_>("DorsalVentralIDDisable", "Disable dorsal/ventral", null, "On", "Off", thruster_and_grid_tagger.create_damper_override_reader(1), thruster_and_grid_tagger.create_damper_override_setter(1), thruster_and_grid_tagger.is_grid_control_available);
            }

            IMyTerminalControlLabel controller_line_ID_mode = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlLabel, _controller_type_>("TTDTWM_IDMODE");
            controller_line_ID_mode.Label = MyStringId.GetOrCompute("Inertia Damper Mode");
            MyAPIGateway.TerminalControls.AddControl<_controller_type_>(controller_line_ID_mode);
            create_button<_controller_type_>("IDFullStop"   ,   "Full Stop", null, "Select", thruster_and_grid_tagger.create_ID_mode_selector(false), thruster_and_grid_tagger.is_grid_control_available         , thruster_and_grid_tagger.create_ID_mode_indicator(false));
            create_button<_controller_type_>("IDCircularise", "Circularise", null, "Select", thruster_and_grid_tagger.create_ID_mode_selector( true), thruster_and_grid_tagger.is_grid_circularise_mode_available, thruster_and_grid_tagger.create_ID_mode_indicator( true));

            IMyTerminalControlLabel controller_line_manoeuvre = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlLabel, _controller_type_>("TTDTWM_IDmanoeuvre");
            controller_line_manoeuvre.Label = MyStringId.GetOrCompute("Manoeuvres");
            MyAPIGateway.TerminalControls.AddControl<_controller_type_>(controller_line_manoeuvre);
            create_button<_controller_type_>("IDPrograde"  ,    "Prograde", null, "Start", thruster_and_grid_tagger.create_manoeuvre_starter(engine_control_unit.ID_manoeuvres.burn_prograde  ), thruster_and_grid_tagger.is_grid_circularise_mode_available, thruster_and_grid_tagger.create_manoeuvre_indicator(engine_control_unit.ID_manoeuvres.burn_prograde  ));
            create_button<_controller_type_>("IDRetrograde",  "Retrograde", null, "Start", thruster_and_grid_tagger.create_manoeuvre_starter(engine_control_unit.ID_manoeuvres.burn_retrograde), thruster_and_grid_tagger.is_grid_circularise_mode_available, thruster_and_grid_tagger.create_manoeuvre_indicator(engine_control_unit.ID_manoeuvres.burn_retrograde));
            create_button<_controller_type_>("IDNormal"    ,      "Normal", null, "Start", thruster_and_grid_tagger.create_manoeuvre_starter(engine_control_unit.ID_manoeuvres.burn_normal    ), thruster_and_grid_tagger.is_grid_circularise_mode_available, thruster_and_grid_tagger.create_manoeuvre_indicator(engine_control_unit.ID_manoeuvres.burn_normal    ));
            create_button<_controller_type_>("IDAntiNormal", "Anti-normal", null, "Start", thruster_and_grid_tagger.create_manoeuvre_starter(engine_control_unit.ID_manoeuvres.burn_antinormal), thruster_and_grid_tagger.is_grid_circularise_mode_available, thruster_and_grid_tagger.create_manoeuvre_indicator(engine_control_unit.ID_manoeuvres.burn_antinormal));
            create_button<_controller_type_>("IDOutward"   ,     "Outward", null, "Start", thruster_and_grid_tagger.create_manoeuvre_starter(engine_control_unit.ID_manoeuvres.burn_outward   ), thruster_and_grid_tagger.is_grid_circularise_mode_available, thruster_and_grid_tagger.create_manoeuvre_indicator(engine_control_unit.ID_manoeuvres.burn_outward   ));
            create_button<_controller_type_>("IDInward"    ,      "Inward", null, "Start", thruster_and_grid_tagger.create_manoeuvre_starter(engine_control_unit.ID_manoeuvres.burn_inward    ), thruster_and_grid_tagger.is_grid_circularise_mode_available, thruster_and_grid_tagger.create_manoeuvre_indicator(engine_control_unit.ID_manoeuvres.burn_inward    ));

            create_PB_property<int, _controller_type_>("CurrentIDMode", get_current_ID_mode);
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
                if (!screen_info.torque_disabled && _sample_thruster?.GetProperty("Override") == null)
                    return;
                _thruster_controls_set = true;
                _sample_thruster       = null;
                if (screen_info.torque_disabled)
                    return;
                IMyTerminalControlSeparator thruster_line = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlSeparator, IMyThrust>("TTDTWM_LINE1");
                MyAPIGateway.TerminalControls.AddControl<IMyThrust>(thruster_line);
                create_switch  <IMyThrust>(     "ActiveControl",             "Steering", null, "On", "Off", "On", "Off", thruster_and_grid_tagger.is_under_active_control, thruster_and_grid_tagger.set_active_control , thruster_and_grid_tagger.is_active_control_available);
                create_switch  <IMyThrust>(          "AntiSlip",      "Thrust Trimming", null, "On", "Off", "On", "Off", thruster_and_grid_tagger.is_anti_slip           , thruster_and_grid_tagger.set_anti_slip      , thruster_and_grid_tagger.is_anti_slip_available     );
                create_checkbox<IMyThrust>("DisableLinearInput", "Disable linear input", null, "On", "Off",              thruster_and_grid_tagger.is_rotational_only     , thruster_and_grid_tagger.toggle_linear_input, thruster_and_grid_tagger.is_active_control_available);
                create_switch  <IMyThrust>(       "StaticLimit",       "Thrust Limiter", null, "On", "Off", "On", "Off", thruster_and_grid_tagger.is_thrust_limiter_on   , thruster_and_grid_tagger.set_thrust_limiter , thruster_and_grid_tagger.is_thrust_limiter_available);
                create_slider<IMyThrust>("ManualThrottle", "Manual throttle", 
                    thruster_and_grid_tagger.get_manual_throttle, thruster_and_grid_tagger.set_manual_throttle, thruster_and_grid_tagger.throttle_status,
                    0.0f, 100.0f, 5.0f, "IncreaseThrottle", "DecreaseThrottle", "Increase Manual Throttle", "Decrease Manual Throttle");
                create_PB_property<float, IMyThrust>("BalancedLevel", thruster_and_grid_tagger.get_thrust_limit);
            }

            if (!_programmable_block_properties_set)
            {
                if (_sample_PB?.GetProperty("Content") == null)
                    return;
                create_PB_property<Func<string,             string, bool>, IMyProgrammableBlock>("ComputeOrbitElements"           , get_ship_elements_calculator  );
                create_PB_property<Func<string, Vector3D, Vector3D, bool>, IMyProgrammableBlock>("ComputeOrbitElementsFromVectors", get_vector_elements_calculator);
                create_PB_property<Func<string>, IMyProgrammableBlock>("GetReferenceBodyName", get_reference_name_fetcher);
                create_PB_property<Action<Vector3D[]>, IMyProgrammableBlock>("GetPrimaryVectors" , get_vector_fetcher );
                create_PB_property<Action<  double[]>, IMyProgrammableBlock>("GetPrimaryScalars" , get_scalar_fetcher );
                create_PB_property<Action<  double[]>, IMyProgrammableBlock>("GetDerivedElements", get_derived_fetcher);
                create_PB_property<Action<double?,   double[]>, IMyProgrammableBlock>("GetPositionalElements", get_positional_fetcher  );
                create_PB_property<Action<double , Vector3D[]>, IMyProgrammableBlock>("GetStateVectors"      , get_state_vector_fetcher);

                create_PB_property<Func<double, double,   double>, IMyProgrammableBlock>("ConvertTrueAnomalyToMean", get_true_to_mean_converter );
                create_PB_property<Func<double, double,   double>, IMyProgrammableBlock>("ConvertMeanAnomalyToTrue", get_mean_to_true_converter );
                create_PB_property<Func<double, double, Vector3D>, IMyProgrammableBlock>(      "ComputeOrbitNormal", get_orbit_normal_calculator);
                create_PB_property<Func<Vector3D, Vector3D, Vector3D, double, double>, IMyProgrammableBlock>("ConvertRadialToTtrueAnomaly", get_radius_to_anomaly_converter);
                create_PB_property<Func<double, double, double, double, double, double, ValueTuple<double, double>?>, IMyProgrammableBlock>
                    ("ComputeOrbitIntersections", get_intersection_calculator);
                _programmable_block_properties_set = true;
                _sample_PB                         = null;
            }

            _setup_complete = _ship_controller_controls_set && _thruster_controls_set && _programmable_block_properties_set && _entity_events_set
                && screen_info.settings_loaded && sync_helper.network_handlers_registered;
        }

        private void calibration_thread()
        {
            try
            {
                _grids_perform_calibration?.Invoke();
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
                    _grids_handle_2s_period_background?.Invoke();
                    gravity_and_physics.update_player_and_floating_object_reference_bodies();
                }
                _grids_handle_4Hz_background?.Invoke();
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
                    thruster_and_grid_tagger.handle_2s_period();
                    _grids_handle_2s_period_foreground?.Invoke();
                }
                screen_info.refresh_local_player_HUD();
                refresh_connected_grids();
                thruster_and_grid_tagger.handle_4Hz();
                _grids_handle_4Hz_foreground?.Invoke();
            }
            synchronise_orbit_stabilisation_for_connected_grids();
            _grids_handle_60Hz?.Invoke();
            gravity_and_physics.apply_gravity_to_players_and_floating_objects();
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
            gravity_and_physics.dispose_all_PBs();
            foreach (IMyCubeGrid leftover_grid in _grids.Keys.ToList())
                on_entity_removed(leftover_grid);
            gravity_and_physics.deregister_all_sources();
            _sample_controller  = null;
            _sample_thruster    = null;
            _sample_PB          = null;
            _setup_complete     = _entity_events_set = _ship_controller_controls_set = _thruster_controls_set = _programmable_block_properties_set = false;
            _session_ref        = null;
        }
    }
}
