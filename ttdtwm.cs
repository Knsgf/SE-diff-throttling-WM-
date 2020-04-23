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
    [MySessionComponentDescriptor(MyUpdateOrder.AfterSimulation)]
    public class session_handler: MySessionComponentBase
    {
        public const bool SINGLE_THREADED_EXEC = false;
        
        #region fields

        private readonly Dictionary<IMyCubeGrid, grid_logic> _grids = new Dictionary<IMyCubeGrid, grid_logic>();
        private Action _grids_handle_60Hz = null, _grids_handle_4Hz_foreground = null, _grids_handle_2s_period_foreground = null;
        private Action _grids_handle_4Hz_background = null, _grids_handle_2s_period_background = null, _grids_perform_calibration = null;
        private Task _manager_task, _calibration_task;

        private static IMyThrust         _sample_thruster   = null;
        private static IMyShipController _sample_controller = null;

        private IMyCubeGrid _last_grid;

        private int  _count15 = 15, _count8_foreground = 8, _count8_background = 8;
        private bool _entity_events_set = false;

        private static bool _panel_controls_set = false;

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
                var new_grid_logic = new grid_logic(grid);
                _grids_handle_60Hz                 += new_grid_logic.handle_60Hz;
                _grids_handle_4Hz_foreground       += new_grid_logic.handle_4Hz_foreground;
                _grids_handle_2s_period_foreground += new_grid_logic.handle_2s_period_foreground;
                _grids_handle_4Hz_background       += new_grid_logic.handle_4Hz_background;
                _grids_handle_2s_period_background += new_grid_logic.handle_2s_period_background;
                _grids_perform_calibration         += new_grid_logic.perform_individual_calibration;
                _grids.Add(grid, new_grid_logic);
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
                gravity_and_physics.register_player(character);
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
                gravity_and_physics.deregister_player(character);
        }

        private bool is_grid_CoT_mode_on(IMyTerminalBlock controller)
        {
            return _grids[controller.CubeGrid].CoT_mode_on;
        }

        private bool is_grid_control_available(IMyTerminalBlock controller)
        {
            if (!((PB.IMyShipController) controller).ControlThrusters)
                return false;

            IMyCubeGrid grid = controller.CubeGrid;
            if (((MyCubeGrid) grid).HasMainCockpit() && !((PB.IMyShipController) controller).IsMainCockpit)
                return false;
            if (grid != _last_grid)
            {
                controller.RefreshCustomInfo();
                _last_grid = grid;
            }
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
            return is_grid_control_available(controller) && _grids[controller.CubeGrid].is_landing_mode_available;
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
            return is_grid_control_available(controller) && _grids[controller.CubeGrid].is_circularisation_avaiable;
        }

        private Action<IMyTerminalBlock> create_ID_mode_selector(bool select_circularise)
        {
            return delegate (IMyTerminalBlock controller)
            {
                _grids[controller.CubeGrid].circularise = select_circularise;
            };
        }

        private Action<IMyTerminalBlock, StringBuilder> create_ID_mode_indicator(bool circularisation_indicator)
        {
            return delegate (IMyTerminalBlock controller, StringBuilder status)
            {
                status.Clear();
                if (_grids[controller.CubeGrid].circularise == circularisation_indicator)
                    status.Append("Select");
            };
        }

        private Action<IMyTerminalBlock> create_manoeuvre_starter(engine_control_unit.ID_manoeuvres manoeuvre)
        {
            return delegate (IMyTerminalBlock controller)
            {
                _grids[controller.CubeGrid].start_manoeuvre(manoeuvre, true);
            };
        }

        private Action<IMyTerminalBlock, StringBuilder> create_manoeuvre_indicator(engine_control_unit.ID_manoeuvres manoeuvre)
        {
            return delegate (IMyTerminalBlock controller, StringBuilder status)
            {
                status.Clear();
                if (_grids[controller.CubeGrid].current_manoeuvre == manoeuvre)
                    status.Append("Active");
            };
        }

        private Func<double, double, double> get_true_to_mean_converter(IMyTerminalBlock dummy)
        {
            return orbit_elements.convert_true_anomaly_to_mean;
        }

        private Func<double, double, double> get_mean_to_true_converter(IMyTerminalBlock dummy)
        {
            return orbit_elements.convert_mean_anomaly_to_true;
        }

        private Func<double, double, Vector3D> get_orbit_normal_calculator(IMyTerminalBlock dummy)
        {
            return orbit_elements.calculate_orbit_normal;
        }

        private Func<string, string, bool> get_elements_calculator(IMyTerminalBlock PB)
        {
            return delegate (string reference_name, string grid_name)
            {
                return gravity_and_physics.calculate_elements_for_PB(PB, reference_name, grid_name);
            };
        }

        private Action<Dictionary<string, Vector3D>> get_vector_fetcher(IMyTerminalBlock PB)
        {
            return delegate (Dictionary<string, Vector3D> vector_elements)
            {
                gravity_and_physics.retrieve_primary_vectors(PB, vector_elements);
            };
        }

        private Action<Dictionary<string, double>> get_scalar_fetcher(IMyTerminalBlock PB)
        {
            return delegate (Dictionary<string, double> scalar_elements)
            {
                gravity_and_physics.retrieve_primary_scalars(PB, scalar_elements);
            };
        }

        private Action<Dictionary<string, double>> get_derived_fetcher(IMyTerminalBlock PB)
        {
            return delegate (Dictionary<string, double> derived_elements)
            {
                gravity_and_physics.retrieve_derived_elements(PB, derived_elements);
            };
        }

        private Action<double?, Dictionary<string, double>> get_positional_fetcher(IMyTerminalBlock PB)
        {
            return delegate (double? true_anomaly, Dictionary<string, double> positional_elements)
            {
                gravity_and_physics.retrieve_positional_elements(PB, true_anomaly, positional_elements);
            };
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
                    grid_logic cur_grid_object = grid_list[cur_grid];
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

        internal static void sample_thruster(IMyThrust thruster)
        {
            if (!_panel_controls_set && _sample_thruster == null)
                _sample_thruster = thruster;
        }

        internal static void sample_controller(IMyShipController controller)
        {
            if (!_panel_controls_set && _sample_controller == null)
                _sample_controller = controller;
        }

        private void create_controller_widgets<_controller_type_>()
        {
            IMyTerminalControlSeparator controller_line = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlSeparator, _controller_type_>("TTDTWM_LINE1");
            MyAPIGateway.TerminalControls.AddControl<_controller_type_>(controller_line);
            create_checkbox<_controller_type_>(    "RotationalDamping",        "Rotational Damping", null,                   "On",    "Off", is_grid_rotational_damping_on, set_grid_rotational_damping,      is_grid_control_available);
            create_switch  <_controller_type_>(              "CoTMode",  "Active Control Reference", null,  "CoT",  "CoM",  "CoT",    "CoM",           is_grid_CoT_mode_on,           set_grid_CoT_mode,      is_grid_control_available);
            create_switch  <_controller_type_>("IndividualCalibration", "Thrust Calibration Method", null, "Ind.", "Quad", "Ind.",   "Quad",    use_individual_calibration,   choose_calibration_method,      is_grid_control_available);
            create_switch  <_controller_type_>(          "LandingMode",            "Touchdown Mode", null,   "On",  "Off", "Land", "Flight",       is_grid_landing_mode_on,       set_grid_landing_mode, is_grid_landing_mode_available);

            IMyTerminalControlSeparator controller_line2 = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlSeparator, _controller_type_>("TTDTWM_LINE2");
            MyAPIGateway.TerminalControls.AddControl<_controller_type_>(controller_line2);
            IMyTerminalControlLabel controller_line_ID_override_label = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlLabel, _controller_type_>("TTDTWM_ID_OVR");
            controller_line_ID_override_label.Label = MyStringId.GetOrCompute("Inertia Damper Overrides");
            MyAPIGateway.TerminalControls.AddControl<_controller_type_>(controller_line_ID_override_label);
            create_checkbox<_controller_type_>(      "ForeAftIDDisable", "Disable fore/aft"      , null, "On", "Off", create_damper_override_reader(2), create_damper_override_setter(2), is_grid_control_available);
            create_checkbox<_controller_type_>("PortStarboardIDDisable", "Disable port/starboard", null, "On", "Off", create_damper_override_reader(0), create_damper_override_setter(0), is_grid_control_available);
            create_checkbox<_controller_type_>("DorsalVentralIDDisable", "Disable dorsal/ventral", null, "On", "Off", create_damper_override_reader(1), create_damper_override_setter(1), is_grid_control_available);

            IMyTerminalControlLabel controller_line_ID_mode = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlLabel, _controller_type_>("TTDTWM_IDMODE");
            controller_line_ID_mode.Label = MyStringId.GetOrCompute("Inertia Damper Mode");
            MyAPIGateway.TerminalControls.AddControl<_controller_type_>(controller_line_ID_mode);
            create_button<_controller_type_>("IDFullStop"   ,   "Full Stop", null, "Select", create_ID_mode_selector(false), is_grid_control_available         , create_ID_mode_indicator(false));
            create_button<_controller_type_>("IDCircularise", "Circularise", null, "Select", create_ID_mode_selector( true), is_grid_circularise_mode_available, create_ID_mode_indicator( true));

            IMyTerminalControlLabel controller_line_manoeuvre = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlLabel, _controller_type_>("TTDTWM_IDmanoeuvre");
            controller_line_manoeuvre.Label = MyStringId.GetOrCompute("Manoeuvres");
            MyAPIGateway.TerminalControls.AddControl<_controller_type_>(controller_line_manoeuvre);
            create_button<_controller_type_>("IDPrograde"  ,    "Prograde", null, "Start", create_manoeuvre_starter(engine_control_unit.ID_manoeuvres.burn_prograde  ), is_grid_circularise_mode_available, create_manoeuvre_indicator(engine_control_unit.ID_manoeuvres.burn_prograde  ));
            create_button<_controller_type_>("IDRetrograde",  "Retrograde", null, "Start", create_manoeuvre_starter(engine_control_unit.ID_manoeuvres.burn_retrograde), is_grid_circularise_mode_available, create_manoeuvre_indicator(engine_control_unit.ID_manoeuvres.burn_retrograde));
            create_button<_controller_type_>("IDNormal"    ,      "Normal", null, "Start", create_manoeuvre_starter(engine_control_unit.ID_manoeuvres.burn_normal    ), is_grid_circularise_mode_available, create_manoeuvre_indicator(engine_control_unit.ID_manoeuvres.burn_normal    ));
            create_button<_controller_type_>("IDAntiNormal", "Anti-normal", null, "Start", create_manoeuvre_starter(engine_control_unit.ID_manoeuvres.burn_antinormal), is_grid_circularise_mode_available, create_manoeuvre_indicator(engine_control_unit.ID_manoeuvres.burn_antinormal));
            create_button<_controller_type_>("IDOutward"   ,     "Outward", null, "Start", create_manoeuvre_starter(engine_control_unit.ID_manoeuvres.burn_outward   ), is_grid_circularise_mode_available, create_manoeuvre_indicator(engine_control_unit.ID_manoeuvres.burn_outward   ));
            create_button<_controller_type_>("IDInward"    ,      "Inward", null, "Start", create_manoeuvre_starter(engine_control_unit.ID_manoeuvres.burn_inward    ), is_grid_circularise_mode_available, create_manoeuvre_indicator(engine_control_unit.ID_manoeuvres.burn_inward    ));
        }

        private void try_register_handlers()
        {
            sync_helper.try_register_handlers();
            screen_info.try_register_handlers();
            if (!_entity_events_set && MyAPIGateway.Entities != null)
            {
                var existing_entities = new HashSet<IMyEntity>();
                MyAPIGateway.Entities.GetEntities(existing_entities);
                foreach (IMyEntity cur_entity in existing_entities)
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

                create_PB_property<Func<string, string, bool>, IMyProgrammableBlock>("ComputeOrbitElements", get_elements_calculator);
                create_PB_property<Action<Dictionary<string, Vector3D>>, IMyProgrammableBlock>("GetPrimaryVectors", get_vector_fetcher);
                create_PB_property<Action<Dictionary<string,   double>>, IMyProgrammableBlock>("GetPrimaryScalars", get_scalar_fetcher);
                create_PB_property<Action<Dictionary<string,   double>>, IMyProgrammableBlock>("GetDerivedElements", get_derived_fetcher);
                create_PB_property<Action<double?, Dictionary<string, double>>, IMyProgrammableBlock>("GetPositionalElements", get_positional_fetcher);

                create_PB_property<Func<double, double,   double>, IMyProgrammableBlock>("ConvertTrueAnomalyToMean", get_true_to_mean_converter );
                create_PB_property<Func<double, double,   double>, IMyProgrammableBlock>("ConvertMeanAnomalyToTrue", get_mean_to_true_converter );
                create_PB_property<Func<double, double, Vector3D>, IMyProgrammableBlock>(      "ComputeOrbitNormal", get_orbit_normal_calculator);

                IMyTerminalControlSeparator thruster_line = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlSeparator, IMyThrust>("TTDTWM_LINE1");
                MyAPIGateway.TerminalControls.AddControl<IMyThrust>(thruster_line);
                create_switch  <IMyThrust>(     "ActiveControl",             "Steering", null, "On", "Off", "On", "Off", thruster_tagger.is_under_active_control, thruster_tagger.set_active_control , thruster_tagger.is_active_control_available);
                create_switch  <IMyThrust>(          "AntiSlip",      "Thrust Trimming", null, "On", "Off", "On", "Off", thruster_tagger.is_anti_slip           , thruster_tagger.set_anti_slip      , thruster_tagger.is_anti_slip_available     );
                create_checkbox<IMyThrust>("DisableLinearInput", "Disable linear input", null, "On", "Off",              thruster_tagger.is_rotational_only     , thruster_tagger.toggle_linear_input, thruster_tagger.is_active_control_available);
                create_switch  <IMyThrust>(       "StaticLimit",       "Thrust Limiter", null, "On", "Off", "On", "Off", thruster_tagger.is_thrust_limiter_on   , thruster_tagger.set_thrust_limiter , thruster_tagger.is_thrust_limiter_available);

                IMyTerminalControlSlider manual_throttle = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlSlider, IMyThrust>("ManualThrottle");
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
                create_PB_property<float, IMyThrust>("BalancedLevel", thruster_tagger.get_thrust_limit);

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
                    gravity_and_physics.update_player_reference_bodies();
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

            if (_grids_handle_60Hz == null)
            {
                try_register_handlers();
                return;
            }

            screen_info.refresh_local_player_info();
            if (--_count15 <= 0)
            {
                _count15 = 15;
                screen_info.refresh_local_player_HUD();
                _last_grid = null;
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
            gravity_and_physics.apply_gravity_to_players();
        }

        public session_handler()
        {
            _session_ref = this;
            sync_helper.register_entity(this, 0);
        }

        protected override void UnloadData()
        {
            base.UnloadData();
            sync_helper.deregister_entity(0);
            sync_helper.deregister_handlers();
            screen_info.deregister_handlers();
            foreach (IMyCubeGrid leftover_grid in _grids.Keys.ToList())
                on_entity_removed(leftover_grid);
            MyAPIGateway.Entities.OnEntityAdd    -= on_entity_added;
            MyAPIGateway.Entities.OnEntityRemove -= on_entity_removed;
            _sample_controller  = null;
            _sample_thruster    = null;
            _panel_controls_set = false;
            _session_ref        = null;
        }
    }
}
