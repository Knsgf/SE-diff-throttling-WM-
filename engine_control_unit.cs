﻿using System;
using System.Collections.Generic;

using Sandbox.Game.Entities;
using Sandbox.ModAPI;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.Entity;
using VRage.Game.ModAPI;
using VRage.Utils;
using VRageMath;

namespace ttdtwm
{
    sealed class engine_control_unit
    {
        public enum ID_manoeuvres
        {
            manoeuvre_off  = 0,
            burn_prograde = 1, burn_retrograde = 2,
            burn_normal   = 3, burn_antinormal = 4,
            burn_outward  = 5, burn_inward     = 6
        };
        public const float MIN_CIRCULARISATION_SPEED = 10.0f;
        
        #region fields

        const int   NUM_ROTATION_SAMPLES = 6, PHYSICS_ENABLE_DELAY = 6;
        const float DESCENDING_SPEED     = 0.5f, MIN_OVERRIDE = 1.001f, LINEAR_INTEGRAL_CONSTANT = 0.05f, MIN_THROTTLE = 0.01f;
        const bool  DEBUG_THR_ALWAYS_ON  = false, DEBUG_DISABLE_ALT_HOLD = false;

        enum thrust_dir { fore = 0, aft = 3, starboard = 1, port = 4, dorsal = 2, ventral = 5 };
        sealed class thruster_info     // Technically a struct
        {
            public engine_control_unit host_ECU;
            public float               max_force, actual_max_force;
            public Vector3             torque_factor, grid_centre_pos, static_moment, actual_static_moment, CoM_offset, reference_vector;
            public thrust_dir          nozzle_direction;
            public float               current_setting, thrust_limit, prev_setting, manual_throttle;
            public bool                enable_limit, enable_rotation, active_control_on, is_RCS, disable_linear_input, skip, throttle_up, apply_limit;
            public thruster_info       next_tandem_thruster, prev_tandem_thruster, opposing_thruster;
            public string              throttle_setting;
            public int                 control_sector;
        };

        private static readonly Vector3I[] _thrust_forward_vectors;

        private readonly torque_and_orbit_control _grid_movement;
        private readonly solver_entry[]           _control_sectors;
        private readonly revised_simplex_solver[] _linear_solvers =
        {
            new revised_simplex_solver(),   // fore
            new revised_simplex_solver(),   // starboard
            new revised_simplex_solver(),   // dorsal
            new revised_simplex_solver(),   // aft
            new revised_simplex_solver(),   // port
            new revised_simplex_solver()    // ventral
        };
        private readonly bool[] _calibration_scheduled = new bool[6];

        private readonly Dictionary<IMyThrust, thruster_info> _thrusters_moved = new Dictionary<IMyThrust, thruster_info>();
        private readonly List<thruster_info>[] _thruster_infos =
        {
            new List<thruster_info>(),  // fore
            new List<thruster_info>(),  // starboard
            new List<thruster_info>(),  // dorsal
            new List<thruster_info>(),  // aft
            new List<thruster_info>(),  // port
            new List<thruster_info>()   // ventral
        };

        private static readonly Vector3 neg_override_bound = Vector3.One * (-MIN_THROTTLE), pos_override_bound = Vector3.One * MIN_THROTTLE;

        private static readonly float[] __control_vector  = new float[6], __manual_control_vector = new float[6];
        private static readonly float[] __braking_vector  = new float[6];
        private static readonly float[] __requested_force = new float[6];
        private static readonly float[] __actual_force    = new float[6];
        private static readonly float[] __non_THR_force   = new float[6];

        private static readonly float[][] __linear_component =
        {
            new float[6],   // fore
            new float[6],   // starboard
            new float[6],   // dorsal
            new float[6],   // aft
            new float[6],   // port
            new float[6],   // ventral
        };

        private static readonly float[] __steering_input          = new float[6];
        private static readonly float[] __angular_velocity        = new float[6];
        private static readonly float[] __target_angular_velocity = new float[6];
        private static readonly float[] __angular_velocity_diff   = new float[6];
        private static readonly float[] __angular_acceleration    = new float[6];
        private static readonly float[] __residual_torque         = new float[6];
        private static readonly float[] __gyro_override           = new float[6];
        private static readonly float[] __thrust_limits           = new float[6];

        private static readonly Vector3[] __new_static_moment = new Vector3[6];
        private static readonly float[] __new_total_force   = new   float[6];

        private static readonly float[] __local_gravity_inv         = new float[6];
        private static readonly float[] __local_linear_velocity_inv = new float[6];
        private static readonly bool[]  __is_controlled_side        = new bool[6];

        private readonly MyCubeGrid _grid;

        private readonly Dictionary<IMyThrust, thruster_info>[] _controlled_thrusters =
        {
            new Dictionary<IMyThrust, thruster_info>(),   // fore
            new Dictionary<IMyThrust, thruster_info>(),   // starboard
            new Dictionary<IMyThrust, thruster_info>(),   // dorsal
            new Dictionary<IMyThrust, thruster_info>(),   // aft
            new Dictionary<IMyThrust, thruster_info>(),   // port
            new Dictionary<IMyThrust, thruster_info>()    // ventral
        };
        private readonly Dictionary<Vector3I, List<thruster_info>>[] _tandem_thrusters =
        {
            new Dictionary<Vector3I, List<thruster_info>>(),  // fore
            new Dictionary<Vector3I, List<thruster_info>>(),  // starboard
            new Dictionary<Vector3I, List<thruster_info>>(),  // dorsal
            new Dictionary<Vector3I, List<thruster_info>>(),  // aft
            new Dictionary<Vector3I, List<thruster_info>>(),  // port
            new Dictionary<Vector3I, List<thruster_info>>()   // ventral
        };
        private readonly List<thruster_info>[] _collective_thrusters =
        {
            new List<thruster_info>(),  // fore
            new List<thruster_info>(),  // starboard
            new List<thruster_info>(),  // dorsal
            new List<thruster_info>(),  // aft
            new List<thruster_info>(),  // port
            new List<thruster_info>(),  // ventral
        };
        private readonly Dictionary<IMyThrust, thruster_info> _uncontrolled_thrusters = new Dictionary<IMyThrust, thruster_info>();
        private readonly float[] _max_force              = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        private readonly float[] _actual_max_force       = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        private readonly float[] _uncontrolled_max_force = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        private readonly float[] _thrust_override_vector = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

        private readonly HashSet<MyGyro> _gyroscopes = new HashSet<MyGyro>();

        private bool     _calibration_in_progress = false;
        private Vector3D _grid_CoM_location = Vector3D.Zero, _world_linear_velocity;
        private Vector3D _world_angular_velocity;
        private float    _spherical_moment_of_inertia = 1.0f, _grid_mass = 1.0f, _total_mass = 1.0f;

        private readonly  bool[]    _uncontrolled_override_checked = new bool[6];
        private readonly  bool[]    _is_solution_good  = { false, false, false, false, false, false };
        private readonly  bool[]    _active_control_on = { false, false, false, false, false, false };
        private readonly float[]    _smoothed_acceleration       = new float[6];
        private readonly float[]    _last_angular_velocity       = new float[6];
        private readonly float[]    _current_trim                = new float[6];
        private readonly float[]    _last_trim                   = new float[6];
        private readonly float[]    _aux_trim                    = new float[6];
        private readonly float[]    _angular_velocity_checkpoint = new float[6];
        private readonly float[]    _turn_sensitivity            = new float[6];
        private readonly Vector3?[] _active_CoT = new Vector3?[6];

        private Vector3 _local_angular_velocity, _prev_angular_velocity = Vector3.Zero, _torque, _manual_rotation, _target_velocity;
        private Vector3 _target_rotation, _gyro_override = Vector3.Zero, _dampers_axis_enable = new Vector3(1.0f);
        private bool    _CoM_shifted = false, _current_mode_is_CoT = false, _new_mode_is_CoT = false, _circularise_on = false;
        private bool    _is_gyro_override_active = false, _individual_calibration_on = false, _calibration_ready = false, _calibration_complete = false;
        private bool    _all_engines_off = false, _force_override_refresh = false;
        private float   _trim_fadeout = 1.0f;
        private bool    _integral_cleared = false, _is_thrust_override_active = false, _grid_is_movable = false;

        private readonly  bool[] _enable_linear_integral = { true, true, true, true, true, true };
        private readonly float[] _linear_integral        = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

        private float     _prev_air_density = float.MinValue, _counter_thrust_limit = 1.0f;
        private Vector3   _manual_thrust, _thrust_override = Vector3.Zero, _linear_control = Vector3.Zero;

        private readonly Vector3[] _rotation_samples = new Vector3[NUM_ROTATION_SAMPLES];
        private          Vector3   _sample_sum       = Vector3.Zero;
        private          int       _current_index    = 0, _physics_enable_delay = PHYSICS_ENABLE_DELAY, _num_subgrids = 1;

        private static readonly byte[] __message = new byte[1];

        private MyEntity _match_velocity_with = null;

        #endregion

        #region Properties

        public bool linear_dampers_on          { get; set; }
        public bool rotational_damping_on      { get; set; }
        public bool autopilot_on               { get; set; }
        public bool use_individual_calibration { get; set; }
        public bool secondary_ECU              { get; set; }

        public int  thrust_reduction      { get; private set; }

        public bool active_control_enabled
        {
            get
            {
                for (int dir_index = 0; dir_index < 3; ++dir_index)
                {
                    if (_active_control_on[dir_index])
                        return true;
                }
                return false;
            }
        }

        public float vertical_speed { get; private set; }
        
        public bool landing_mode_on { get; set; } = false;
        public bool     CoT_mode_on { get; set; } = false;

        public bool circularise_on
        {
            get
            {
                return _circularise_on && current_speed >= MIN_CIRCULARISATION_SPEED && gravity_and_physics.world_has_gravity;
            }
            set
            {
                _circularise_on = value && gravity_and_physics.world_has_gravity;
            }
        }

        public Vector3 current_trim
        {
            get
            {
                Vector3 result = recompose_vector(_current_trim);
                return result;
            }
            set
            {
                decompose_vector(value, _current_trim);
            }
        }

        public Vector3 last_trim
        {
            get
            {
                Vector3 result = recompose_vector(_last_trim);
                return result;
            }
            set
            {
                decompose_vector(value, _last_trim);
            }
        }

        public Vector3 linear_integral
        {
            get
            {
                Vector3 result = recompose_vector(_linear_integral);
                return result;
            }
            set
            {
                decompose_vector(value, _linear_integral);
            }
        }

        public float current_speed { get; private set; }

        public ID_manoeuvres current_manoeuvre { get; private set; } = ID_manoeuvres.manoeuvre_off;

        #endregion

        #region DEBUG

        public const bool CALIBRATION_DEBUG = false, FULL_CALIBRATION_DEBUG = false;

        private void log_ECU_action(string method_name, string message)
        {
            MyLog.Default.WriteLine(string.Format("TTDTWM\tengine_control_unit<{0} [{1}]>.{2}(): {3}", _grid.DisplayName, _grid.EntityId, method_name, message));
            int num_controlled_thrusters = 0;
            foreach (Dictionary<IMyThrust, thruster_info> cur_direction in _controlled_thrusters)
                num_controlled_thrusters += cur_direction.Count;
            MyLog.Default.WriteLine(string.Format("TTDTWM\ttotal thrusters: {0} ({1}/{2}/{3}/{4}/{5}/{6} controlled, {7} uncontrolled)", 
                _uncontrolled_thrusters.Count + num_controlled_thrusters,
                _controlled_thrusters[(int) thrust_dir.fore     ].Count,
                _controlled_thrusters[(int) thrust_dir.aft      ].Count,
                _controlled_thrusters[(int) thrust_dir.starboard].Count,
                _controlled_thrusters[(int) thrust_dir.port     ].Count,
                _controlled_thrusters[(int) thrust_dir.dorsal   ].Count,
                _controlled_thrusters[(int) thrust_dir.ventral  ].Count,
                _uncontrolled_thrusters.Count));
        }

        private void screen_vector<type>(string method_name, string vector_name, type[] vector, int display_time_ms)
        {
            screen_info.screen_text(_grid, method_name, string.Format("{0} = {1:F3}/{2:F3}/{3:F3}/{4:F3}/{5:F3}/{6:F3}", 
                vector_name,
                vector[(int) thrust_dir.fore     ],
                vector[(int) thrust_dir.aft      ],
                vector[(int) thrust_dir.starboard],
                vector[(int) thrust_dir.port     ],
                vector[(int) thrust_dir.dorsal   ],
                vector[(int) thrust_dir.ventral  ]), display_time_ms);
        }

        #endregion

        #region torque calculation

        private void refresh_thruster_info_for_single_direction(Dictionary<IMyThrust, thruster_info> thrusters)
        {
            IMyThrust     cur_thruster;
            thruster_info cur_thruster_info;

            foreach (KeyValuePair<IMyThrust, thruster_info> cur_thruster_entry in thrusters)
            {
                cur_thruster      = cur_thruster_entry.Key;
                cur_thruster_info = cur_thruster_entry.Value;
                cur_thruster_info.CoM_offset    = cur_thruster_info.grid_centre_pos - _grid_CoM_location;
                cur_thruster_info.torque_factor = Vector3.Cross(cur_thruster_info.CoM_offset, -_thrust_forward_vectors[(int) cur_thruster_info.nozzle_direction]);
            }
        }

        private void refresh_thruster_info()
        {
            refresh_thruster_info_for_single_direction(_uncontrolled_thrusters);
            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                refresh_thruster_info_for_single_direction(_controlled_thrusters[dir_index]);
                _calibration_scheduled[dir_index] = true;
            }
        }

        private void refresh_reference_vectors(bool CoM_shifted)
        {
            if (CoM_shifted)
                refresh_thruster_info();
            if (!_current_mode_is_CoT && CoM_shifted || _current_mode_is_CoT != _new_mode_is_CoT)
            {
                if (_new_mode_is_CoT)
                    update_reference_vectors_for_CoT_mode();
                else
                    update_reference_vectors_for_CoM_mode();
                _current_mode_is_CoT = _new_mode_is_CoT;
            }
        }

        private Vector3D get_world_combined_CoM()
        {
            List<IMyCubeGrid>      grid_list     = MyAPIGateway.GridGroups.GetGroup(_grid, GridLinkTypeEnum.Physical);
            Vector3D               static_moment = Vector3D.Zero;
            float                  cur_mass;
            MyPhysicsComponentBase grid_body;

            _total_mass   = 0.0f;
            _num_subgrids = grid_list.Count;
            foreach (IMyCubeGrid cur_grid in grid_list)
            {
                grid_body      = cur_grid.Physics;
                cur_mass       = grid_body.Mass;
                static_moment += cur_mass * grid_body.CenterOfMassWorld;
                _total_mass   += cur_mass;
            }
            if (_total_mass < 1.0f)
            {
                _total_mass   = _grid_mass;
                _num_subgrids = 1;
                return _grid.Physics.CenterOfMassWorld;
            }
            return static_moment / _total_mass;
        }

        private void calculate_and_apply_torque()
        {
            //if (MyAPIGateway.Multiplayer != null && !MyAPIGateway.Multiplayer.IsServer)
            //    return;

            IMyThrust     thruster;
            thruster_info cur_thruster_info;

            _torque = Vector3.Zero;
            foreach (Dictionary<IMyThrust, thruster_info> cur_direction in _controlled_thrusters)
            {
                foreach (KeyValuePair<IMyThrust, thruster_info> cur_thruster in cur_direction)
                {
                    cur_thruster_info = cur_thruster.Value;
                    if (cur_thruster.Key.IsWorking)
                        _torque += cur_thruster_info.torque_factor * cur_thruster_info.actual_max_force * cur_thruster_info.current_setting;
                }
            }
            foreach (KeyValuePair<IMyThrust, thruster_info> cur_thruster in _uncontrolled_thrusters)
            {
                thruster = cur_thruster.Key;
                if (thruster.IsWorking)
                    _torque += cur_thruster.Value.torque_factor * thruster.CurrentThrust;
            }

            /*
            if (!_stabilisation_off && _active_control_on && !autopilot_on && _max_gyro_torque >= 1.0f && _manual_rotation.LengthSquared() <= 0.0001f)
            {
                float   inverse_angular_acceleration = _spherical_moment_of_inertia / _max_gyro_torque;
                float   gyro_load                    = _local_angular_velocity.Length() * inverse_angular_acceleration;
                if (gyro_load > 1.0f)
                    gyro_load = 1.0f;
                Vector3 gyro_torque_dir = inverse_angular_acceleration * (desired_angular_velocity - _local_angular_velocity);
                if (gyro_torque_dir.LengthSquared() > 1.0f)
                    gyro_torque_dir.Normalize();
                _torque += gyro_torque_dir * _max_gyro_torque * (1.0f - gyro_load);
            }
            */

            if (_physics_enable_delay > 0)
                --_physics_enable_delay;
            else if (_torque.LengthSquared() >= 1.0f)
            {
                Vector3 world_torque = Vector3.Transform(_torque, _grid.WorldMatrix.GetOrientation());
                _grid_movement.apply_torque(world_torque);
            }
        }

        #endregion

        #region thrust calibration

        private void prepare_individual_calibration()
        {
            float x = 0.0f, y = 0.0f;

            foreach (thrust_dir cur_direction in Enum.GetValues(typeof(thrust_dir)))
            {
                if (!_calibration_scheduled[(int) cur_direction])
                    continue;

                _calibration_in_progress = true;
                List<thruster_info>    thruster_infos = _thruster_infos[(int) cur_direction];
                revised_simplex_solver linear_solver  = _linear_solvers[(int) cur_direction];

                if (CALIBRATION_DEBUG)
                    log_ECU_action("prepare_individual_calibration", "Preparing calibration on " + cur_direction.ToString() + " side");
                thruster_infos.Clear();
                foreach (thruster_info cur_thruster_info in _controlled_thrusters[(int) cur_direction].Values)
                {
                    if (!cur_thruster_info.disable_linear_input)
                        thruster_infos.Add(cur_thruster_info);
                    else
                    {
                        cur_thruster_info.enable_limit = true;
                        cur_thruster_info.thrust_limit = 0.0f;
                    }
                }
                for (int index = 0; index < thruster_infos.Count; ++index)
                {
                    switch (cur_direction)
                    {
                        case thrust_dir.fore:
                        case thrust_dir.aft:
                            x = thruster_infos[index].CoM_offset.X;
                            y = thruster_infos[index].CoM_offset.Y;
                            break;

                        case thrust_dir.starboard:
                        case thrust_dir.port:
                            x = thruster_infos[index].CoM_offset.Y;
                            y = thruster_infos[index].CoM_offset.Z;
                            break;

                        case thrust_dir.dorsal:
                        case thrust_dir.ventral:
                            x = thruster_infos[index].CoM_offset.X;
                            y = thruster_infos[index].CoM_offset.Z;
                            break;
                    }
                    if (index >= linear_solver.items.Count)
                        linear_solver.items.Add(new solver_entry());
                    linear_solver.items[index].x = x;
                    linear_solver.items[index].y = y;
                    linear_solver.items[index].max_value = thruster_infos[index].actual_max_force;
                }
            }
            _calibration_ready = true;
        }

        private void set_up_thrust_limits()
        {
            foreach (thrust_dir cur_direction in Enum.GetValues(typeof(thrust_dir)))
            {
                if (!_calibration_scheduled[(int) cur_direction])
                    continue;

                _calibration_scheduled[(int) cur_direction] = false;
                List<thruster_info>    thruster_infos = _thruster_infos[(int) cur_direction];
                revised_simplex_solver linear_solver  = _linear_solvers[(int) cur_direction];

                if (!_is_solution_good[(int) cur_direction])
                {
                    for (int index = 0; index < thruster_infos.Count; ++index)
                    {
                        thruster_infos[index].thrust_limit    = 0.0f;
                        thruster_infos[index].enable_rotation = true;
                    }
                }
                else
                {
                    for (int index = 0; index < thruster_infos.Count; ++index)
                    {
                        thruster_infos[index].thrust_limit = (linear_solver.items[index].max_value > 1.0f)
                            ? (linear_solver.items[index].result / linear_solver.items[index].max_value) : 1.0f;
                        thruster_infos[index].enable_rotation = thruster_infos[index].active_control_on;
                        if (CALIBRATION_DEBUG)
                            log_ECU_action("set_up_thrust_limits", string.Format("{0}/{1} kN ({2})", linear_solver.items[index].result / 1000.0f, linear_solver.items[index].max_value / 1000.0f, cur_direction));
                    }
                }
                foreach (IMyThrust cur_thruster in _controlled_thrusters[(int) cur_direction].Keys)
                    cur_thruster.RefreshCustomInfo();

                if (CALIBRATION_DEBUG)
                {
                    log_ECU_action("set_up_thrust_limits", _is_solution_good[(int) cur_direction]
                        ? string.Format("successfully calibrated {0} thrusters on {1} side", thruster_infos.Count, cur_direction)
                        : string.Format("calibration on {0} side failed", cur_direction));
                }
            }
        }

        private string get_sector_name(int sector_number)
        {
            string result = "";
            int    x = sector_number % 3, y = sector_number / 3;

            switch (y)
            {
                case 0:
                    result = "C";
                    break;

                case 1:
                    result = "U";
                    break;

                case 2:
                    result = "D";
                    break;
            }
            switch (x)
            {
                case 0:
                    result += "C";
                    break;

                case 1:
                    result += "L";
                    break;

                case 2:
                    result += "R";
                    break;
            }
            return result;
        }

        private void perform_quadrant_calibration()
        {
            const float NEUTRAL_OFFSET = 0.333f;

            float x = 0.0f, y = 0.0f;
            int   control_sector, active_sectors, sector_index;
            float neutral_offset = NEUTRAL_OFFSET * _grid.GridSize;

            foreach (thrust_dir cur_direction in Enum.GetValues(typeof(thrust_dir)))
            {
                if (!_calibration_scheduled[(int) cur_direction])
                    continue;

                Dictionary<IMyThrust, thruster_info> thruster_infos = _controlled_thrusters[(int) cur_direction];
                revised_simplex_solver               linear_solver  = _linear_solvers      [(int) cur_direction];

                for (sector_index = 0; sector_index < 3 * 3; ++sector_index)
                    _control_sectors[sector_index].x = _control_sectors[sector_index].y = _control_sectors[sector_index].max_value = 0.0f;
                foreach (thruster_info cur_thruster_info in thruster_infos.Values)
                {
                    if (cur_thruster_info.disable_linear_input)
                        continue;

                    switch (cur_direction)
                    {
                        case thrust_dir.fore:
                        case thrust_dir.aft:
                            x = cur_thruster_info.CoM_offset.X;
                            y = cur_thruster_info.CoM_offset.Y;
                            break;

                        case thrust_dir.starboard:
                        case thrust_dir.port:
                            x = cur_thruster_info.CoM_offset.Y;
                            y = cur_thruster_info.CoM_offset.Z;
                            break;

                        case thrust_dir.dorsal:
                        case thrust_dir.ventral:
                            x = cur_thruster_info.CoM_offset.X;
                            y = cur_thruster_info.CoM_offset.Z;
                            break;
                    }
                    control_sector = 0;
                    if (x < -neutral_offset)
                        control_sector += 1;
                    else if (x > neutral_offset)
                        control_sector += 2;
                    if (y < -neutral_offset)
                        control_sector += 1 * 3;
                    else if (y > neutral_offset)
                        control_sector += 2 * 3;
                    cur_thruster_info.control_sector = control_sector;
                    _control_sectors[control_sector].x         += x * cur_thruster_info.actual_max_force;
                    _control_sectors[control_sector].y         += y * cur_thruster_info.actual_max_force;
                    _control_sectors[control_sector].max_value += cur_thruster_info.actual_max_force;
                }

                active_sectors = 0;
                for (control_sector = 0; control_sector < 3 * 3; ++control_sector)
                {
                    if (_control_sectors[control_sector].max_value < 1.0f)
                        continue;

                    if (CALIBRATION_DEBUG)
                    {
                        log_ECU_action("perform_quadrant_calibration", string.Format("{4} sector {0} max {1} kN at {2}, {3}", 
                            get_sector_name(control_sector), 
                            _control_sectors[control_sector].max_value / 1000.0f, 
                            _control_sectors[control_sector].x / _control_sectors[control_sector].max_value, 
                            _control_sectors[control_sector].y / _control_sectors[control_sector].max_value,
                            cur_direction));
                    }
                    if (active_sectors >= linear_solver.items.Count)
                        linear_solver.items.Add(new solver_entry());
                    linear_solver.items[active_sectors  ].x         = _control_sectors[control_sector].x / _control_sectors[control_sector].max_value;
                    linear_solver.items[active_sectors  ].y         = _control_sectors[control_sector].y / _control_sectors[control_sector].max_value;
                    linear_solver.items[active_sectors++].max_value = _control_sectors[control_sector].max_value;
                }

                if (CALIBRATION_DEBUG)
                    log_ECU_action("perform_quadrant_calibration", "Starting calibration on " + cur_direction.ToString() + " side");
                _is_solution_good[(int) cur_direction] = linear_solver.calculate_solution(active_sectors);
                if (!_is_solution_good[(int) cur_direction])
                {
                    thruster_info cur_thruster_info;

                    if (CALIBRATION_DEBUG)
                        log_ECU_action("perform_quadrant_calibration", "Calibration on " + cur_direction.ToString() + " side failed");
                    foreach (KeyValuePair<IMyThrust, thruster_info> cur_thruster in thruster_infos)
                    {
                        cur_thruster_info = cur_thruster.Value;
                        cur_thruster_info.thrust_limit    = 0.0f;
                        cur_thruster_info.enable_rotation = true;
                        cur_thruster.Key.RefreshCustomInfo();
                    }
                }
                else
                {
                    thruster_info cur_thruster_info;

                    sector_index = 0;
                    for (control_sector = 0; control_sector < 3 * 3; ++control_sector)
                    {
                        if (_control_sectors[control_sector].max_value < 1.0f)
                            _control_sectors[control_sector].result = 0.0f;
                        else
                        {
                            _control_sectors[control_sector].result = linear_solver.items[sector_index].result / linear_solver.items[sector_index].max_value;
                            ++sector_index;
                            if (CALIBRATION_DEBUG)
                            {
                                log_ECU_action("perform_quadrant_calibration", string.Format("{3} sector {0} = {1} kN ({2} %)", 
                                    get_sector_name(control_sector), 
                                    _control_sectors[control_sector].result * _control_sectors[control_sector].max_value / 1000.0f, 
                                    _control_sectors[control_sector].result * 100.0f,
                                    cur_direction));
                            }
                        }
                    }
                    foreach (KeyValuePair<IMyThrust, thruster_info> cur_thruster in thruster_infos)
                    {
                        cur_thruster_info = cur_thruster.Value;
                        if (cur_thruster_info.disable_linear_input)
                        {
                            cur_thruster_info.enable_limit = true;
                            cur_thruster_info.thrust_limit = 0.0f;
                            continue;
                        }
                        cur_thruster_info.thrust_limit    = _control_sectors[cur_thruster_info.control_sector].result;
                        cur_thruster_info.enable_rotation = cur_thruster_info.active_control_on;
                        cur_thruster.Key.RefreshCustomInfo();
                    }
                }
                _calibration_scheduled[(int) cur_direction] = false;
            }
        }

        #endregion

        #region thrust control

        private static void decompose_vector(Vector3 source_vector, float[] decomposed_vector)
        {
            decomposed_vector[(int) thrust_dir.fore     ] = (source_vector.Z > 0.0f) ? ( source_vector.Z) : 0.0f;
            decomposed_vector[(int) thrust_dir.aft      ] = (source_vector.Z < 0.0f) ? (-source_vector.Z) : 0.0f;
            decomposed_vector[(int) thrust_dir.port     ] = (source_vector.X > 0.0f) ? ( source_vector.X) : 0.0f;
            decomposed_vector[(int) thrust_dir.starboard] = (source_vector.X < 0.0f) ? (-source_vector.X) : 0.0f;
            decomposed_vector[(int) thrust_dir.ventral  ] = (source_vector.Y > 0.0f) ? ( source_vector.Y) : 0.0f;
            decomposed_vector[(int) thrust_dir.dorsal   ] = (source_vector.Y < 0.0f) ? (-source_vector.Y) : 0.0f;
        }

        private static Vector3 recompose_vector(float[] decomposed_vector)
        {
            Vector3 result_vector;

            result_vector.Z = decomposed_vector[(int) thrust_dir.fore   ] - decomposed_vector[(int) thrust_dir.aft      ];
            result_vector.X = decomposed_vector[(int) thrust_dir.port   ] - decomposed_vector[(int) thrust_dir.starboard];
            result_vector.Y = decomposed_vector[(int) thrust_dir.ventral] - decomposed_vector[(int) thrust_dir.dorsal   ];
            return result_vector;
        }

        private void apply_thrust_settings(bool reset_all_thrusters)
        {
            const float MIN_THRUST_CHANGE = 1.0E+3f;

            float         setting;
            bool          dry_run = false;
            thruster_info cur_thruster_info;
            IMyThrust     thruster;

            if (reset_all_thrusters && _all_engines_off && !_force_override_refresh)
                return;

            if (!is_under_control_of(screen_info.local_controller) && MyAPIGateway.Multiplayer != null)
            {
                bool controls_active = _manual_rotation.LengthSquared() >= 0.0001f || _linear_control.LengthSquared() >= 0.0001f;
                if (MyAPIGateway.Multiplayer.IsServer)
                {
                    if (controls_active)
                        dry_run = true;

                }
                else if (!controls_active)
                    dry_run = true;
            }

            /*
            if (MyAPIGateway.Multiplayer == null || MyAPIGateway.Multiplayer.IsServer)
                dry_run = false;
            else
            {
                bool is_rotation_small = (_manual_rotation - _prev_rotation).LengthSquared() < 0.0001f;

                dry_run         = !_force_override_refresh || is_rotation_small && (_linear_control - _prev_control).LengthSquared() < 0.0001f;
                _prev_control   = _linear_control;
                _linear_control = Vector3.Zero;
                if (!is_rotation_small)
                    _prev_rotation = _manual_rotation;
            }
            */

            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                if (reset_all_thrusters)
                    _current_trim[dir_index] = _last_trim[dir_index] = 0.0f;
                __is_controlled_side[dir_index] = _controlled_thrusters[dir_index].Count > 0;

                foreach (KeyValuePair<IMyThrust, thruster_info> cur_thruster in _controlled_thrusters[dir_index])
                {
                    thruster          = cur_thruster.Key;
                    cur_thruster_info = cur_thruster.Value;
                    if (_force_override_refresh)
                        cur_thruster_info.prev_setting = cur_thruster.Key.CurrentThrust;

                    if (reset_all_thrusters || cur_thruster_info.actual_max_force < 1.0f || !cur_thruster.Key.IsWorking)
                    {
                        if (cur_thruster_info.prev_setting > 0.0f || reset_all_thrusters)
                        {
                            if (!dry_run)
                                cur_thruster.Key.ThrustOverride = 0.0f;
                            cur_thruster_info.current_setting = cur_thruster_info.prev_setting = 0.0f;
                        }
                        continue;
                    }

                    setting = cur_thruster_info.current_setting * cur_thruster_info.max_force;
                    if (setting < MIN_OVERRIDE)
                        setting = MIN_OVERRIDE;
                    if (   Math.Abs(setting - cur_thruster_info.prev_setting) >= MIN_THRUST_CHANGE 
                        || (setting >= cur_thruster_info.max_force) != (cur_thruster_info.prev_setting >= cur_thruster_info.max_force)
                        || (setting <=                MIN_OVERRIDE) != (cur_thruster_info.prev_setting <=                MIN_OVERRIDE))
                    {
                        if (!dry_run)
                            thruster.ThrustOverride = setting;
                        cur_thruster_info.prev_setting = setting;
                    }
                }
            }

            bool manoeuvre_active = linear_dampers_on && circularise_on || current_manoeuvre != ID_manoeuvres.manoeuvre_off;
            foreach (KeyValuePair<IMyThrust, thruster_info> cur_thruster in _uncontrolled_thrusters)
            {
                cur_thruster_info = cur_thruster.Value;
                if (_force_override_refresh)
                    cur_thruster_info.prev_setting = cur_thruster.Key.CurrentThrust;
                if (!manoeuvre_active && !__is_controlled_side[(int) cur_thruster_info.nozzle_direction] && cur_thruster_info.manual_throttle < 0.01f && cur_thruster_info.prev_setting < MIN_OVERRIDE)
                    continue;

                if (reset_all_thrusters || cur_thruster_info.actual_max_force < 1.0f || !cur_thruster.Key.IsWorking)
                {
                    if (cur_thruster_info.prev_setting > 0.0f || reset_all_thrusters)
                    {
                        if (!dry_run)
                            cur_thruster.Key.ThrustOverride = 0.0f;
                        cur_thruster_info.current_setting = cur_thruster_info.prev_setting = 0.0f;
                    }
                    continue;
                }

                if (cur_thruster_info.manual_throttle >= 0.01f)
                    cur_thruster_info.current_setting = cur_thruster_info.manual_throttle;
                else if (__is_controlled_side[(int) cur_thruster_info.nozzle_direction] || manoeuvre_active)
                    cur_thruster_info.current_setting = __control_vector[(int) cur_thruster_info.nozzle_direction];
                else
                    cur_thruster_info.current_setting = 0.0f;
                if (cur_thruster_info.current_setting * cur_thruster_info.actual_max_force < MIN_OVERRIDE)
                    cur_thruster_info.current_setting = 0.0f;
                setting = cur_thruster_info.current_setting * cur_thruster_info.max_force;
                __actual_force[(int) cur_thruster_info.nozzle_direction] += setting;
                if (   Math.Abs(setting - cur_thruster_info.prev_setting) >= MIN_THRUST_CHANGE
                    || (setting >                         0.0f) != (cur_thruster_info.prev_setting >                         0.0f)
                    || (setting >= cur_thruster_info.max_force) != (cur_thruster_info.prev_setting >= cur_thruster_info.max_force))
                {
                    if (!dry_run)
                        cur_thruster.Key.ThrustOverride = setting;
                    cur_thruster_info.prev_setting = setting;
                }
            }

            _all_engines_off        = reset_all_thrusters;
            _force_override_refresh = false;
        }

        private void initialise_linear_controls(Vector3 local_linear_velocity_vector, Vector3 local_gravity_vector)
        {
            const float DAMPING_CONSTANT = -2.0f;

            _linear_control = Vector3.Clamp(_manual_thrust + _thrust_override, -Vector3.One, Vector3.One);
            decompose_vector(_linear_control, __control_vector);
            for (int dir_index = 0, opposite_dir = 3; dir_index < 3; ++dir_index, ++opposite_dir)
            {
                if (_actual_max_force[dir_index] + _uncontrolled_max_force[dir_index] < 1.0f && _actual_max_force[opposite_dir] + _uncontrolled_max_force[dir_index] < 1.0f)
                    __control_vector[dir_index] = __control_vector[opposite_dir] = 0.0f;
            }
            float gravity_magnitude = local_gravity_vector.Length();
            bool  controls_active   = _linear_control.LengthSquared() > 0.0001f;

            _trim_fadeout = 1.0f;

            if (gravity_magnitude < 0.01f)
                _counter_thrust_limit = 1.0f;
            if (!linear_dampers_on && current_manoeuvre == ID_manoeuvres.manoeuvre_off || controls_active && circularise_on && _match_velocity_with == null)
            {
                _counter_thrust_limit = 1.0f;
                if (!_integral_cleared)
                {
                    for (int dir_index = 0; dir_index < 6; ++dir_index)
                    {
                        _enable_linear_integral[dir_index] = false;
                        _linear_integral[dir_index]        = __braking_vector[dir_index] = 0.0f;
                    }
                    _integral_cleared = true;
                }
                if (!controls_active && gravity_magnitude > 0.1f && vertical_speed > -DESCENDING_SPEED * 0.5f && vertical_speed < DESCENDING_SPEED * 0.5f)
                    _trim_fadeout = Math.Abs(vertical_speed / (DESCENDING_SPEED * 0.5f));
            }
            else
            {
                _integral_cleared = false;
                if (!landing_mode_on)
                {
                    if (vertical_speed >= 0.1f)
                        _counter_thrust_limit *= 0.95f;     // Quickly reduce upwards rebound after descending
                    else if (vertical_speed <= -0.1f)
                        _counter_thrust_limit = 0.95f * _counter_thrust_limit + 0.05f; // When descending, gradually bring counter thrust level back to normal
                }
                else
                {
                    if (vertical_speed >= -DESCENDING_SPEED + 0.05f)
                        _counter_thrust_limit *= 0.99f;
                    else if (vertical_speed <= -DESCENDING_SPEED - 0.05f)
                        _counter_thrust_limit = 0.99f * _counter_thrust_limit + 0.01f;
                    if (!controls_active && vertical_speed >= -DESCENDING_SPEED * 0.5f && vertical_speed < 0.0f)
                        _trim_fadeout = vertical_speed / (-DESCENDING_SPEED * 0.5f);
                }
                Vector3 linear_damping    = local_linear_velocity_vector * DAMPING_CONSTANT - local_gravity_vector * _counter_thrust_limit;
                float   average_grid_mass = _total_mass / _num_subgrids;
                linear_damping           *= (average_grid_mass >= _grid_mass) ? average_grid_mass : _grid_mass;
                decompose_vector(               linear_damping,            __braking_vector);
                decompose_vector(        -local_gravity_vector,         __local_gravity_inv);
                decompose_vector(-local_linear_velocity_vector, __local_linear_velocity_inv);
                Array.Copy(__control_vector, __manual_control_vector, 6);

                for (int dir_index = 0, opposite_dir = 3; dir_index < 3; ++dir_index, ++opposite_dir)
                {
                    _enable_linear_integral[dir_index] = _enable_linear_integral[opposite_dir] = !DEBUG_DISABLE_ALT_HOLD 
                        && (    __local_gravity_inv[dir_index] >         0.0f  ||     __local_gravity_inv[opposite_dir] >  0.0f)
                        &&  __manual_control_vector[dir_index] <  MIN_THROTTLE && __manual_control_vector[opposite_dir] <  MIN_THROTTLE
                        && (      _actual_max_force[dir_index] >=        1.0f  ||       _actual_max_force[opposite_dir] >= 1.0f);

                    set_brake(   dir_index, opposite_dir);
                    set_brake(opposite_dir,    dir_index);
                    if (__control_vector[dir_index] > __control_vector[opposite_dir])
                    {
                        __control_vector[   dir_index] -= __control_vector[opposite_dir];
                        __control_vector[opposite_dir]  = 0.0f;
                    }
                    else
                    { 
                        __control_vector[opposite_dir] -= __control_vector[dir_index];
                        __control_vector[   dir_index]  = 0.0f;
                    }

                    float gravity_ratio          = (gravity_magnitude < 0.01f) ? 1.0f : ((__local_gravity_inv[dir_index] + __local_gravity_inv[opposite_dir]) / gravity_magnitude),
                          axis_speed             = __local_linear_velocity_inv[dir_index] + __local_linear_velocity_inv[opposite_dir],
                          linear_integral_change = LINEAR_INTEGRAL_CONSTANT * (__local_linear_velocity_inv[dir_index] - __local_linear_velocity_inv[opposite_dir]);
                    if (linear_integral_change > 0.0f)
                        set_linear_integral(   dir_index, opposite_dir,  linear_integral_change, gravity_ratio, axis_speed);
                    else if (linear_integral_change < 0.0f)
                        set_linear_integral(opposite_dir,    dir_index, -linear_integral_change, gravity_ratio, axis_speed);

                    if (landing_mode_on)
                    {
                        _linear_integral[dir_index] -= LINEAR_INTEGRAL_CONSTANT * DESCENDING_SPEED * gravity_ratio;
                        if (_linear_integral[dir_index] < 0.0f)
                            _linear_integral[dir_index] = 0.0f;
                        _linear_integral[opposite_dir] -= LINEAR_INTEGRAL_CONSTANT * DESCENDING_SPEED * gravity_ratio;
                        if (_linear_integral[opposite_dir] < 0.0f)
                            _linear_integral[opposite_dir] = 0.0f;
                    }
                }
                normalise_control(current_manoeuvre == ID_manoeuvres.manoeuvre_off);
            }
        }

        private void set_linear_integral(int dir_index, int opposite_dir, float linear_integral_change, float gravity_ratio, float axis_speed)
        {
            if (_linear_integral[opposite_dir] <= 0.0f)
            {
                if (_enable_linear_integral[dir_index])
                {
                    if (linear_integral_change > LINEAR_INTEGRAL_CONSTANT)
                        linear_integral_change = LINEAR_INTEGRAL_CONSTANT;
                    
                    // Prevent integral from winding up excessively in the direction perpendicular to gravity
                    if (axis_speed < 1.0f)
                        linear_integral_change *= axis_speed * (gravity_ratio - 1.0f) + 1.0f;
                    else
                        linear_integral_change *= gravity_ratio;

                    _linear_integral[dir_index] += linear_integral_change;
                }
                _linear_integral[opposite_dir] = 0.0f;
            }
            else
            {
                _linear_integral[opposite_dir] -= linear_integral_change;
                if (_linear_integral[opposite_dir] >= 0.0f)
                    _linear_integral[dir_index] = 0.0f;
                else
                {
                    float max_increase = LINEAR_INTEGRAL_CONSTANT + _linear_integral[opposite_dir];

                    if (max_increase < 0.0f)
                        max_increase = 0.0f;
                    _linear_integral[dir_index] = _enable_linear_integral[dir_index] ? (-_linear_integral[opposite_dir]) : 0.0f;
                    if (_linear_integral[dir_index] > max_increase && !secondary_ECU)
                        _linear_integral[dir_index] = max_increase;
                    _linear_integral[opposite_dir] = 0.0f;
                }
            }
        }

        private void set_brake(int dir_index, int opposite_dir)
        {
            if (__manual_control_vector[opposite_dir] < MIN_THROTTLE && _actual_max_force[dir_index] + _uncontrolled_max_force[dir_index] >= 1.0f)
            {
                float braking_force = __braking_vector[dir_index], mass, average_grid_mass = _total_mass / _num_subgrids;

                mass = (average_grid_mass >= _grid_mass) ? average_grid_mass : _grid_mass;
                if (_enable_linear_integral[dir_index])
                    braking_force += mass * _linear_integral[dir_index];
                if (_actual_max_force[opposite_dir] + _uncontrolled_max_force[opposite_dir] < 1.0f && _enable_linear_integral[opposite_dir])
                    braking_force -= mass * _linear_integral[opposite_dir];
                __control_vector[dir_index] += braking_force / (_actual_max_force[dir_index] + _uncontrolled_max_force[dir_index]);
                if (__control_vector[dir_index] < 0.0f)
                    __control_vector[dir_index] = 0.0f;
                else if (__control_vector[dir_index] >= 1.0f)
                    _enable_linear_integral[dir_index] = _enable_linear_integral[opposite_dir] = false;
            }
        }

        private void normalise_control(bool allow_dogleg_braking)
        {
            float divider;

            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                if (__control_vector[dir_index] > 1.0f)
                {
                    if (allow_dogleg_braking || __manual_control_vector[dir_index] >= MIN_THROTTLE)
                        __control_vector[dir_index] = 1.0f;
                    else
                    {
                        divider = __control_vector[dir_index];
                        for (int dir_index2 = 0; dir_index2 < 6; ++dir_index2)
                        {
                            if (__manual_control_vector[dir_index2] < MIN_THROTTLE)
                                __control_vector[dir_index2] /= divider;
                        }
                    }
                }
            }
        }

        private Vector3 get_reference_vector(thruster_info thruster, Vector3 reference_point, Vector3 thruster_dir)
        {
            Vector3 reference_vector = thruster.grid_centre_pos - reference_point;

            reference_vector -= Vector3.Dot(reference_vector, thruster_dir) * thruster_dir;
            if (!Vector3.IsZero(reference_vector))
                reference_vector.Normalize();
            return reference_vector;
        }

        private void adjust_thrust_for_steering(int cur_dir, int opposite_dir, Vector3 desired_angular_velocity)
        {
            const float DAMPING_CONSTANT = 5.0f, MIN_LINEAR_OPPOSITION = 0.05f, MAX_LINEAR_OPPOSITION = 1.0f, MAX_CONTROL = 0.2f, MIN_COT_SETTING = 0.1f;

            if (_actual_max_force[cur_dir] <= 1.0f)
            {
                __new_static_moment[cur_dir] = Vector3.Zero;
                return;
            }

            float average_grid_mass = _total_mass / _num_subgrids;

            Vector3 angular_velocity_diff = desired_angular_velocity - _local_angular_velocity, total_static_moment = Vector3.Zero;
            float   max_linear_opposition, damping = DAMPING_CONSTANT * ((average_grid_mass >= _grid_mass) ? average_grid_mass : _grid_mass) / _actual_max_force[cur_dir],
                    current_limit = __thrust_limits[cur_dir], total_force = 0.0f, 
                    linear_opposition = Math.Min(MAX_CONTROL, Math.Max(__control_vector[opposite_dir], _thrust_override_vector[opposite_dir])) / MAX_CONTROL,
                    CoT_setting;
            float[] linear_component = __linear_component[cur_dir];
            bool    enforce_thrust_limit = !_current_mode_is_CoT && linear_opposition >= MIN_LINEAR_OPPOSITION, THR_mode_used;

            max_linear_opposition = MAX_LINEAR_OPPOSITION * (1.0f - linear_opposition) + MIN_LINEAR_OPPOSITION * linear_opposition;
            if (_actual_max_force[opposite_dir] < _actual_max_force[cur_dir])
                max_linear_opposition *= _actual_max_force[opposite_dir] / _actual_max_force[cur_dir];
            if (max_linear_opposition > MAX_LINEAR_OPPOSITION)
                max_linear_opposition = MAX_LINEAR_OPPOSITION;

            foreach (thruster_info cur_thruster_info in _controlled_thrusters[cur_dir].Values)
            {
                if (cur_thruster_info.skip)
                {
                    cur_thruster_info.apply_limit = false;
                    continue;
                }
                cur_thruster_info.apply_limit = Vector3.Dot(angular_velocity_diff, cur_thruster_info.torque_factor) < 0.0f;

                decompose_vector(Vector3.Cross(angular_velocity_diff, cur_thruster_info.reference_vector), linear_component);
                if (linear_component[cur_dir] > 0.0f)
                {
                    cur_thruster_info.current_setting += damping * linear_component[cur_dir];
                    if (cur_thruster_info.active_control_on && !cur_thruster_info.is_RCS)
                        cur_thruster_info.current_setting += max_linear_opposition * (1.0f - __thrust_limits[cur_dir]);
                    cur_thruster_info.throttle_up = true;
                    if (enforce_thrust_limit && cur_thruster_info.active_control_on && !cur_thruster_info.is_RCS)
                    {
                        // Limit thrusters opposing player/ID linear input
                        if (cur_thruster_info.current_setting > max_linear_opposition)
                            cur_thruster_info.current_setting = max_linear_opposition;
                    }
                    if (cur_thruster_info.apply_limit && cur_thruster_info.current_setting > current_limit)
                        cur_thruster_info.current_setting = current_limit;
                    else if (cur_thruster_info.current_setting > 1.0f)
                        cur_thruster_info.current_setting = 1.0f;

                    CoT_setting = cur_thruster_info.current_setting;
                    if (CoT_setting < MIN_COT_SETTING)
                        CoT_setting = MIN_COT_SETTING;
                    total_static_moment += CoT_setting * cur_thruster_info.actual_static_moment;
                    total_force         += CoT_setting * cur_thruster_info.actual_max_force;
                }
                else if (linear_component[opposite_dir] > 0.0f)
                {
                    THR_mode_used = cur_thruster_info.active_control_on && !cur_thruster_info.is_RCS;
                    cur_thruster_info.current_setting -= damping * linear_component[opposite_dir];
                    cur_thruster_info.throttle_up      = false;
                    if (cur_thruster_info.current_setting < 0.0f)
                        cur_thruster_info.current_setting = 0.0f;
                    if (THR_mode_used)
                        cur_thruster_info.current_setting += 0.5f * cur_thruster_info.thrust_limit * __control_vector[cur_dir] * (1.0f - cur_thruster_info.current_setting);
                    if (enforce_thrust_limit && THR_mode_used && cur_thruster_info.current_setting > max_linear_opposition)
                    {
                        // Limit thrusters opposing player/ID linear input
                        cur_thruster_info.current_setting = max_linear_opposition;
                    }
                    if (cur_thruster_info.apply_limit && cur_thruster_info.current_setting > current_limit)
                        cur_thruster_info.current_setting = current_limit;

                    total_static_moment += cur_thruster_info.current_setting * cur_thruster_info.actual_static_moment;
                    total_force         += cur_thruster_info.current_setting * cur_thruster_info.actual_max_force;
                }
                else
                    cur_thruster_info.throttle_up = false;
            }

            __new_static_moment[cur_dir] = total_static_moment;
            __new_total_force  [cur_dir] = total_force;

            // Handle triangular thruster arrangements
            if (CoT_mode_on && _active_CoT[opposite_dir] != null)
            {
                Vector3 effective_CoT = (Vector3) _active_CoT[opposite_dir], thruster_dir = _thrust_forward_vectors[cur_dir];

                foreach (thruster_info cur_thruster_info in _controlled_thrusters[cur_dir].Values)
                {
                    if (cur_thruster_info.skip || cur_thruster_info.throttle_up)
                        continue;

                    decompose_vector(Vector3.Cross(angular_velocity_diff, get_reference_vector(cur_thruster_info, effective_CoT, thruster_dir)), linear_component);
                    if (linear_component[cur_dir] > 0.0f)
                    {
                        cur_thruster_info.current_setting += damping * linear_component[cur_dir];
                        if (cur_thruster_info.current_setting > 1.0f)
                            cur_thruster_info.current_setting = 1.0f;
                    }
                }
            }
        }

        // Ensures that resulting linear force doesn't exceed player/ID input (to prevent undesired drift when turning)
        void normalise_thrust()
        {
            const float MAX_NORMALISATION = 10.0f;

            float linear_force = 0.0f, requested_force = 0.0f, max_setting = 0.0f, max_control = 0.0f, dir_force1, dir_force2;
            bool  zero_thrust_reduction = true;

            Action<int> eliminate_direct_opposition = delegate (int dir_index)
            {
                thruster_info first_opposite_trhuster, cur_opposite_thruster;
                float         cur_thruster_force, opposite_thruster_force;
                bool          active_control_off, opposite_control_off;

                foreach (thruster_info cur_thruster_info in _controlled_thrusters[dir_index].Values)
                {
                    first_opposite_trhuster = cur_thruster_info.opposing_thruster;
                    if (first_opposite_trhuster == null || cur_thruster_info.actual_max_force < 1.0f)
                        continue;

                    cur_thruster_force    = cur_thruster_info.current_setting * cur_thruster_info.actual_max_force;
                    active_control_off    = cur_thruster_info.enable_limit && !cur_thruster_info.active_control_on;
                    cur_opposite_thruster = first_opposite_trhuster;
                    do
                    {
                        if (cur_opposite_thruster.actual_max_force >= 1.0f)
                        {
                            opposite_thruster_force = cur_opposite_thruster.current_setting * cur_opposite_thruster.actual_max_force;
                            opposite_control_off    = cur_opposite_thruster.enable_limit && !cur_opposite_thruster.active_control_on;
                            if (opposite_control_off && cur_opposite_thruster.current_setting >= 0.1f || cur_thruster_force <= opposite_thruster_force)
                            {
                                if (!opposite_control_off)
                                    cur_opposite_thruster.current_setting = (opposite_thruster_force - cur_thruster_force) / cur_opposite_thruster.actual_max_force;
                                cur_thruster_force = 0.0f;
                                break;
                            }
                            if (!active_control_off)
                                cur_thruster_force -= opposite_thruster_force;
                            cur_opposite_thruster.current_setting = 0.0f;
                        }
                        cur_opposite_thruster = cur_opposite_thruster.next_tandem_thruster;
                    }
                    while (cur_opposite_thruster != first_opposite_trhuster);
                    if (!active_control_off)
                        cur_thruster_info.current_setting = cur_thruster_force / cur_thruster_info.actual_max_force;
                }
            };
            if (!session_handler.SINGLE_THREADED_EXEC)
                MyAPIGateway.Parallel.For(0, 3, eliminate_direct_opposition);
            {
                for (int dir_index = 0; dir_index < 3; ++dir_index)
                    eliminate_direct_opposition(dir_index);
            }

            for (int dir_index = 0, opposite_dir = 3; dir_index < 3; ++dir_index, ++opposite_dir)
            { 
                foreach (thruster_info cur_thruster_info in _controlled_thrusters[dir_index].Values)
                {
                    if (max_setting < cur_thruster_info.current_setting)
                        max_setting = cur_thruster_info.current_setting;
                }
                foreach (thruster_info cur_thruster_info in _controlled_thrusters[opposite_dir].Values)
                {
                    if (max_setting < cur_thruster_info.current_setting)
                        max_setting = cur_thruster_info.current_setting;
                }
                if (max_control < __control_vector[   dir_index])
                    max_control = __control_vector[   dir_index];
                if (max_control < __control_vector[opposite_dir])
                    max_control = __control_vector[opposite_dir];
                if (max_control < _thrust_override_vector[dir_index])
                    max_control = _thrust_override_vector[dir_index];
                if (max_control < _thrust_override_vector[opposite_dir])
                    max_control = _thrust_override_vector[opposite_dir];
            }

            if (max_setting > 0.0f)
            {
                float max_normalisation_multiplier = 1.0f / max_setting;

                if (max_normalisation_multiplier > MAX_NORMALISATION)
                    max_normalisation_multiplier = MAX_NORMALISATION;
                max_normalisation_multiplier = 1.0f + max_control * (max_normalisation_multiplier - 1.0f);

                Action<int> normalise_direction = delegate (int dir_index)
                {
                    __actual_force[dir_index] = __non_THR_force[dir_index] = 0.0f;
                    foreach (thruster_info cur_thruster_info in _controlled_thrusters[dir_index].Values)
                    {
                        cur_thruster_info.current_setting *= max_normalisation_multiplier;
                        if (!cur_thruster_info.active_control_on || cur_thruster_info.is_RCS)
                            __non_THR_force[dir_index] += cur_thruster_info.current_setting * cur_thruster_info.actual_max_force;
                        else
                            __actual_force [dir_index] += cur_thruster_info.current_setting * cur_thruster_info.actual_max_force;
                    }
                };
                if (!session_handler.SINGLE_THREADED_EXEC)
                    MyAPIGateway.Parallel.For(0, 6, normalise_direction);
                else
                {
                    for (int dir_index = 0; dir_index < 6; ++dir_index)
                        normalise_direction(dir_index);
                }
            }

            Action<int> equalise_2_directions = delegate (int dir_index)
            {
                int   opposite_dir     = dir_index + 3;
                float force1           = __actual_force[   dir_index] + __non_THR_force[   dir_index];
                float force2           = __actual_force[opposite_dir] + __non_THR_force[opposite_dir];
                float new_force_ratio1 = 1.0f, new_force_ratio2 = 1.0f;

                if (__actual_force[dir_index] >= 1.0f && force1 - __requested_force[dir_index] > force2)
                {
                    new_force_ratio1 = (force2 + __requested_force[dir_index] - __non_THR_force[dir_index]) / __actual_force[dir_index];
                    if (new_force_ratio1 < 0.0f)
                        new_force_ratio1 = 0.0f;
                    else if (new_force_ratio1 > 1.0f)
                        new_force_ratio1 = 1.0f;

                    foreach (thruster_info cur_thruster_info in _controlled_thrusters[dir_index].Values)
                    {
                        if (cur_thruster_info.active_control_on && !cur_thruster_info.is_RCS)
                            cur_thruster_info.current_setting *= new_force_ratio1;
                    }
                }
                __actual_force[dir_index] *= new_force_ratio1;
                force1 = __actual_force[dir_index] + __non_THR_force[dir_index];

                if (__actual_force[opposite_dir] >= 1.0f && force2 - __requested_force[opposite_dir] > force1)
                {
                    new_force_ratio2 = (force1 + __requested_force[opposite_dir] - __non_THR_force[opposite_dir]) / __actual_force[opposite_dir];
                    if (new_force_ratio2 < 0.0f)
                        new_force_ratio2 = 0.0f;
                    else if (new_force_ratio2 > 1.0f)
                        new_force_ratio2 = 1.0f;

                    foreach (thruster_info cur_thruster_info in _controlled_thrusters[opposite_dir].Values)
                    {
                        if (cur_thruster_info.active_control_on && !cur_thruster_info.is_RCS)
                            cur_thruster_info.current_setting *= new_force_ratio2;
                    }
                }
                __actual_force[opposite_dir] *= new_force_ratio2;
            };
            if (!session_handler.SINGLE_THREADED_EXEC)
                MyAPIGateway.Parallel.For(0, 3, equalise_2_directions);
            else
            {
                for (int dir_index = 0; dir_index < 3; ++dir_index)
                    equalise_2_directions(dir_index);
            }

            for (int dir_index = 0, opposite_dir = 3; dir_index < 3; ++dir_index, ++opposite_dir)
            { 
                if (__control_vector[dir_index] >= 0.25f || __control_vector[opposite_dir] >= 0.25f)
                {
                    zero_thrust_reduction = false;
                    dir_force1 = Math.Abs((__actual_force[dir_index] + __non_THR_force[dir_index]) - (__actual_force[opposite_dir] + __non_THR_force[opposite_dir]));
                    dir_force2 = __requested_force[dir_index] + __requested_force[opposite_dir];
                    if (dir_force2 < dir_force1)
                        dir_force2 = dir_force1;
                    linear_force    += dir_force1 * dir_force1;
                    requested_force += dir_force2 * dir_force2;
                }
            }

            if (MyAPIGateway.Multiplayer == null || MyAPIGateway.Multiplayer.IsServer)
            { 
                if (zero_thrust_reduction)
                    thrust_reduction = 0;
                else
                {
                    linear_force     = (float) Math.Sqrt(   linear_force);
                    requested_force  = (float) Math.Sqrt(requested_force);
                    thrust_reduction = (requested_force < 1.0f) ? 0 : ((int) ((1.0f - linear_force / requested_force) * 100.0f + 0.5f));

                    if (thrust_reduction < 0)
                        thrust_reduction = 0;
                }
            }
        }

        private void change_trim(float[] trim, int dir_index, int opposite_dir, float trim_change)
        {
            const float MAX_TRIM = 5.0f;
            
            if (trim[opposite_dir] > 0.0f)
            {
                trim[opposite_dir] -= trim_change;
                if (trim[opposite_dir] < 0.0f)
                {
                    trim[   dir_index] = -trim[opposite_dir];
                    trim[opposite_dir] = 0.0f;
                }
            }
            else if (trim[dir_index] < MAX_TRIM)
                trim[dir_index] += trim_change;
        }

        private void adjust_trim_setting(out Vector3 desired_angular_velocity)
        {
            const float ANGULAR_INTEGRAL_COEFF = 0.4f, ANGULAR_DERIVATIVE_COEFF = 0.05f, MAX_TRIM = 5.0f, THRUST_CUTOFF_TRIM = 4.0f, CHECKPOINT_FADE = 0.75f, 
                ANGULAR_ACCELERATION_SMOOTHING = 0.5f;

            bool    rotational_damping_enabled = rotational_damping_on;
            float   trim_change, thrust_limit_pitch, thrust_limit_yaw, thrust_limit_roll;
            Vector3 local_angular_velocity      = rotational_damping_enabled ? _local_angular_velocity : Vector3.Zero;
            Vector3 local_angular_acceleration  = (local_angular_velocity - _prev_angular_velocity) * MyEngineConstants.UPDATE_STEPS_PER_SECOND;
            _prev_angular_velocity = local_angular_velocity;

            Vector3 steering_input = _manual_rotation;
            steering_input.X /= 0.2f;
            steering_input.Y /= 0.45f;
            steering_input.Z /= 0.45f;
            decompose_vector(            steering_input,       __steering_input);
            decompose_vector(    local_angular_velocity,     __angular_velocity);
            decompose_vector(local_angular_acceleration, __angular_acceleration);
            Vector3 residual_torque_vector = _torque / _spherical_moment_of_inertia;
            if (residual_torque_vector.LengthSquared() > 1.0f)
                residual_torque_vector.Normalize();
            //residual_torque_vector *= 0.01f;
            decompose_vector(residual_torque_vector, __residual_torque);
            decompose_vector(_is_gyro_override_active ? _gyro_override : Vector3.Zero, __gyro_override);

            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                __target_angular_velocity[dir_index] = __steering_input[dir_index] * _turn_sensitivity[dir_index];
                if (!rotational_damping_enabled)
                    __target_angular_velocity[dir_index] += __angular_velocity[dir_index];
                else if (_is_gyro_override_active)
                    __target_angular_velocity[dir_index] += __gyro_override   [dir_index];
                else if (__steering_input[dir_index] > 0.01f)
                    __target_angular_velocity[dir_index] += __angular_velocity[dir_index];
            }
            decompose_vector(recompose_vector(__target_angular_velocity) - local_angular_velocity, __angular_velocity_diff);

            int opposite_dir;
            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                if (!rotational_damping_enabled || !_active_control_on[dir_index])
                {
                    _last_angular_velocity[dir_index] = -1.0f;
                    _current_trim[dir_index] = _aux_trim[dir_index] = _angular_velocity_checkpoint[dir_index] = _smoothed_acceleration[dir_index] = 0.0f;
                }

                opposite_dir = (dir_index + 3) % 6;
                if (_is_gyro_override_active && __gyro_override[dir_index] > 0.01f && __angular_velocity_diff[dir_index] < 0.01f)
                    __angular_velocity_diff[dir_index] = 0.0f;
                if (__target_angular_velocity[dir_index] != _last_angular_velocity[dir_index])
                {
                    if (_angular_velocity_checkpoint[dir_index] < __angular_velocity_diff[dir_index] * CHECKPOINT_FADE)
                        _angular_velocity_checkpoint[dir_index] = __angular_velocity_diff[dir_index] * CHECKPOINT_FADE;
                    if (_angular_velocity_checkpoint[opposite_dir] < __angular_velocity_diff[opposite_dir] * CHECKPOINT_FADE)
                        _angular_velocity_checkpoint[opposite_dir] = __angular_velocity_diff[opposite_dir] * CHECKPOINT_FADE;
                    _last_angular_velocity[dir_index] = __target_angular_velocity[dir_index];
                }

                trim_change = ANGULAR_INTEGRAL_COEFF * __angular_velocity_diff[dir_index];
                if (__angular_velocity_diff[dir_index] < 0.0001f && __angular_velocity_diff[opposite_dir] < 0.0001f)
                    trim_change += ANGULAR_DERIVATIVE_COEFF * __residual_torque[opposite_dir];
                change_trim(    _aux_trim, dir_index, opposite_dir, trim_change);
                change_trim(_current_trim, dir_index, opposite_dir, trim_change);
                if (_aux_trim[dir_index] > 0.0 && _current_trim[opposite_dir] > 0.0f)
                    _aux_trim[dir_index] = 0.0f;
                else if (_aux_trim[opposite_dir] > 0.0 && _current_trim[dir_index] > 0.0f)
                    _aux_trim[opposite_dir] = 0.0f;
            }

            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                _current_trim[dir_index] *= _trim_fadeout;
                _aux_trim    [dir_index] *= _trim_fadeout;
                if (_current_trim[dir_index] < _aux_trim[dir_index])
                    _current_trim[dir_index] = _aux_trim[dir_index];
                if (__angular_velocity_diff[dir_index] < _angular_velocity_checkpoint[dir_index])
                {
                    _angular_velocity_checkpoint[dir_index] *= CHECKPOINT_FADE;
                    if (_angular_velocity_checkpoint[dir_index] < 0.01f)
                        _angular_velocity_checkpoint[dir_index] = 0.0f;
                    _current_trim[dir_index] = 0.5f * (_current_trim[dir_index] + _aux_trim[dir_index]);
                    _aux_trim    [dir_index] = 0.0f;
                }
                _smoothed_acceleration[dir_index] = _smoothed_acceleration[dir_index] * ANGULAR_ACCELERATION_SMOOTHING + __angular_acceleration[dir_index] * (1.0f - ANGULAR_ACCELERATION_SMOOTHING);
            }

            Vector3 trim_vector = recompose_vector(_current_trim);
            thrust_limit_pitch = 1.0f - (1.0f / THRUST_CUTOFF_TRIM) * Math.Abs(trim_vector.X);
            thrust_limit_yaw   = 1.0f - (1.0f / THRUST_CUTOFF_TRIM) * Math.Abs(trim_vector.Y);
            thrust_limit_roll  = 1.0f - (1.0f / THRUST_CUTOFF_TRIM) * Math.Abs(trim_vector.Z);
            if (thrust_limit_pitch < 0.0f)
                thrust_limit_pitch = 0.0f;
            if (thrust_limit_yaw < 0.0f)
                thrust_limit_yaw = 0.0f;
            if (thrust_limit_roll < 0.0f)
                thrust_limit_roll = 0.0f;
            __thrust_limits[(int) thrust_dir.fore     ] = __thrust_limits[(int) thrust_dir.aft    ] = (thrust_limit_pitch < thrust_limit_yaw ) ? thrust_limit_pitch : thrust_limit_yaw;
            __thrust_limits[(int) thrust_dir.starboard] = __thrust_limits[(int) thrust_dir.port   ] = (thrust_limit_yaw   < thrust_limit_roll) ? thrust_limit_yaw   : thrust_limit_roll;
            __thrust_limits[(int) thrust_dir.dorsal   ] = __thrust_limits[(int) thrust_dir.ventral] = (thrust_limit_pitch < thrust_limit_roll) ? thrust_limit_pitch : thrust_limit_roll;

            desired_angular_velocity = recompose_vector(__angular_velocity_diff) + recompose_vector(_current_trim) - ANGULAR_DERIVATIVE_COEFF * recompose_vector(_smoothed_acceleration);
        }

        private void handle_thrust_control(Vector3 world_linear_velocity, Vector3 target_linear_velocity, Vector3 world_angular_velocity, bool sleep_mode_on)
        {
            Matrix  inverse_world_rotation = _grid.PositionComp.WorldMatrixNormalizedInv.GetOrientation();
            Vector3 local_linear_velocity  = Vector3.Transform(world_linear_velocity - target_linear_velocity, inverse_world_rotation);
            Vector3 local_gravity          = Vector3.Transform(_grid.Physics.Gravity, inverse_world_rotation);
            float   gravity_magnitude      = local_gravity.Length();
            vertical_speed = (gravity_magnitude < 0.1f) ? 0.0f : (Vector3.Dot(world_linear_velocity, _grid.Physics.Gravity) / (-gravity_magnitude));

            if (sleep_mode_on)
            {
                thrust_reduction = 0;
                apply_thrust_settings(reset_all_thrusters: true);
                return;
            }

            _local_angular_velocity = Vector3.Transform(world_angular_velocity, inverse_world_rotation);
            Vector3 desired_angular_velocity;
            initialise_linear_controls(local_linear_velocity * _dampers_axis_enable, local_gravity * _dampers_axis_enable);
            adjust_trim_setting(out desired_angular_velocity);
            _new_mode_is_CoT = CoT_mode_on;

            Action<int> set_up_thrusters = delegate (int dir_index)
            {
                thruster_info cur_thruster_info;
                float         control = __control_vector[dir_index], min_collective_throttle = 1.0f;

                __requested_force[dir_index] = 0.0f;
                foreach (thruster_info cur_thruster in _collective_thrusters[dir_index])
                {
                    if (min_collective_throttle > cur_thruster.manual_throttle)
                        min_collective_throttle = cur_thruster.manual_throttle;
                }
                foreach (KeyValuePair<IMyThrust, thruster_info> cur_thruster in _controlled_thrusters[dir_index])
                {
                    cur_thruster_info = cur_thruster.Value;
                    if (!cur_thruster.Key.IsWorking || cur_thruster_info.actual_max_force < 1.0f)
                    { 
                        cur_thruster_info.current_setting = 0.0f;
                        cur_thruster_info.skip            = true;
                    }
                    else
                    {
                        if (cur_thruster_info.manual_throttle < 0.01f)
                            cur_thruster_info.current_setting = cur_thruster_info.disable_linear_input ? 0.0f : control;
                        else if (cur_thruster_info.enable_limit && !cur_thruster_info.active_control_on && _is_solution_good[dir_index])
                            cur_thruster_info.current_setting = min_collective_throttle;
                        else
                            cur_thruster_info.current_setting = cur_thruster_info.manual_throttle;
                        __requested_force[dir_index] += cur_thruster_info.current_setting * cur_thruster_info.actual_max_force;
                        if (cur_thruster_info.enable_limit && (_is_solution_good[dir_index] || cur_thruster_info.disable_linear_input))
                            cur_thruster_info.current_setting *= cur_thruster_info.thrust_limit;
                        cur_thruster_info.skip = !cur_thruster_info.enable_rotation;
                    }
                }
                adjust_thrust_for_steering(dir_index, (dir_index < 3) ? (dir_index + 3) : (dir_index - 3), desired_angular_velocity);
            };

            if (!session_handler.SINGLE_THREADED_EXEC)
                MyAPIGateway.Parallel.For(0, 6, set_up_thrusters);
            else
            { 
                for (int dir_index = 0; dir_index < 6; ++dir_index)
                    set_up_thrusters(dir_index);
            }

            for (int dir_index = 0, opposite_dir = 3; dir_index < 3; ++dir_index, ++opposite_dir)
            {
                float total_force      = __new_total_force[dir_index] + __new_total_force[opposite_dir];
                _active_CoT[dir_index] = _active_CoT[opposite_dir] = (total_force < 1.0f) ? null : ((Vector3?) ((__new_static_moment[dir_index] + __new_static_moment[opposite_dir]) / total_force));

                if (__requested_force[dir_index] > __requested_force[opposite_dir])
                {
                    __requested_force[   dir_index] -= __requested_force[opposite_dir];
                    __requested_force[opposite_dir]  = 0.0f;
                }
                else
                {
                    __requested_force[opposite_dir] -= __requested_force[dir_index];
                    __requested_force[   dir_index] = 0.0f;
                }
            }
            normalise_thrust();
            apply_thrust_settings(reset_all_thrusters: false);
        }

        private void update_reference_vectors_for_CoM_mode()
        {
            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                Dictionary<IMyThrust, thruster_info> cur_direction = _controlled_thrusters  [dir_index];
                Vector3                              thruster_dir  = _thrust_forward_vectors[dir_index];

                foreach (thruster_info cur_thruster_info in cur_direction.Values)
                    cur_thruster_info.reference_vector = get_reference_vector(cur_thruster_info, _grid_CoM_location, thruster_dir);
            }
        }

        private void update_reference_vectors_for_CoT_mode()
        {
            Vector3                              total_static_moment, CoT_location, thruster_dir;
            Dictionary<IMyThrust, thruster_info> cur_direction;

            for (int dir_index = 0, opposite_dir = 3; dir_index < 3; ++dir_index, ++opposite_dir)
            {
                if (_actual_max_force[dir_index] < 1.0f || _actual_max_force[opposite_dir] < 1.0f)
                {
                    cur_direction = _controlled_thrusters  [dir_index];
                    thruster_dir  = _thrust_forward_vectors[dir_index];
                    foreach (thruster_info cur_thruster_info in cur_direction.Values)
                        cur_thruster_info.reference_vector = get_reference_vector(cur_thruster_info, _grid_CoM_location, thruster_dir);
                    cur_direction = _controlled_thrusters  [opposite_dir];
                    thruster_dir  = _thrust_forward_vectors[opposite_dir];
                    foreach (thruster_info cur_thruster_info in cur_direction.Values)
                        cur_thruster_info.reference_vector = get_reference_vector(cur_thruster_info, _grid_CoM_location, thruster_dir);
                }
                else
                {
                    total_static_moment = Vector3.Zero;
                    cur_direction = _controlled_thrusters[dir_index];
                    foreach (thruster_info cur_thruster_info in cur_direction.Values)
                        total_static_moment += cur_thruster_info.actual_static_moment;
                    cur_direction = _controlled_thrusters[opposite_dir];
                    foreach (thruster_info cur_thruster_info in cur_direction.Values)
                        total_static_moment += cur_thruster_info.actual_static_moment;
                    CoT_location = total_static_moment / (_actual_max_force[dir_index] + _actual_max_force[opposite_dir]);

                    cur_direction = _controlled_thrusters  [dir_index];
                    thruster_dir  = _thrust_forward_vectors[dir_index];
                    foreach (thruster_info cur_thruster_info in cur_direction.Values)
                        cur_thruster_info.reference_vector = get_reference_vector(cur_thruster_info, CoT_location, thruster_dir);
                    cur_direction = _controlled_thrusters  [opposite_dir];
                    thruster_dir  = _thrust_forward_vectors[opposite_dir];
                    foreach (thruster_info cur_thruster_info in cur_direction.Values)
                        cur_thruster_info.reference_vector = get_reference_vector(cur_thruster_info, CoT_location, thruster_dir);
                }
            }
        }

        #endregion

        #region thruster manager

        private static thrust_dir get_nozzle_orientation(IMyThrust thruster)
        {
            Vector3I dir_vector = ((MyThrust) thruster).ThrustForwardVector;

            foreach (thrust_dir cur_direction in Enum.GetValues(typeof(thrust_dir)))
            {
                if (_thrust_forward_vectors[(int) cur_direction] == dir_vector)
                    return cur_direction;
            }
            throw new ArgumentException(string.Format("Thruster {0}  is not grid-aligned (direction = {1})", thruster.CustomName, dir_vector));
        }

        private void find_tandem_and_opposite_thrusters(IMyThrust thruster, thruster_info examined_thruster_info)
        {
            Dictionary<Vector3I, List<thruster_info>> cur_direction =       _tandem_thrusters[(int) examined_thruster_info.nozzle_direction];
            Vector3I                                  filter_vector = _thrust_forward_vectors[(int) examined_thruster_info.nozzle_direction], filtered_location;

            filter_vector     = Vector3I.One - filter_vector / (filter_vector.X + filter_vector.Y + filter_vector.Z);
            filtered_location = (thruster.Min + thruster.Max) * filter_vector;
            if (!cur_direction.ContainsKey(filtered_location))
            {
                cur_direction.Add(filtered_location, new List<thruster_info>());
                examined_thruster_info.next_tandem_thruster = examined_thruster_info.prev_tandem_thruster = examined_thruster_info;
                cur_direction[filtered_location].Add(examined_thruster_info);
            }
            else
            {
                List<thruster_info> tandem_thrusters      = cur_direction[filtered_location];
                thruster_info       first_tandem_thruster = tandem_thrusters[0];

                examined_thruster_info.next_tandem_thruster = first_tandem_thruster.next_tandem_thruster;
                examined_thruster_info.prev_tandem_thruster = first_tandem_thruster;
                first_tandem_thruster.next_tandem_thruster  = examined_thruster_info.next_tandem_thruster.prev_tandem_thruster = examined_thruster_info;
                tandem_thrusters.Add(examined_thruster_info);
            }

            if (examined_thruster_info.next_tandem_thruster != examined_thruster_info)
                examined_thruster_info.opposing_thruster = examined_thruster_info.next_tandem_thruster.opposing_thruster;
            else
            {
                Dictionary<Vector3I, List<thruster_info>> opposite_direction = _tandem_thrusters[((int) examined_thruster_info.nozzle_direction + 3) % 6];

                if (!opposite_direction.ContainsKey(filtered_location))
                    examined_thruster_info.opposing_thruster = null;
                else
                {
                    thruster_info first_opposing_thruster = opposite_direction[filtered_location][0], current_thruster_info;

                    examined_thruster_info.opposing_thruster = current_thruster_info = first_opposing_thruster;
                    do
                    {
                        current_thruster_info.opposing_thruster = examined_thruster_info;
                        current_thruster_info                   = current_thruster_info.next_tandem_thruster;
                    }
                    while (current_thruster_info != first_opposing_thruster);
                }
            }
        }

        private void remove_thruster_from_lists(IMyThrust thruster, thruster_info removed_thruster_info)
        {
            if (removed_thruster_info.opposing_thruster != null)
            {
                thruster_info first_opposing_thruster = removed_thruster_info.opposing_thruster, current_thruster_info;

                for (current_thruster_info  = removed_thruster_info.next_tandem_thruster;
                     current_thruster_info != removed_thruster_info;
                     current_thruster_info  = current_thruster_info.next_tandem_thruster)
                {
                    current_thruster_info.opposing_thruster = first_opposing_thruster;
                }

                if (first_opposing_thruster != null)
                {
                    thruster_info next_tandem_thruster = (removed_thruster_info.next_tandem_thruster == removed_thruster_info) ? null : removed_thruster_info.next_tandem_thruster;
                    current_thruster_info = first_opposing_thruster;
                    do
                    {
                        current_thruster_info.opposing_thruster = next_tandem_thruster;
                        current_thruster_info                   = current_thruster_info.next_tandem_thruster;
                    }
                    while (current_thruster_info != first_opposing_thruster);
                }

                removed_thruster_info.opposing_thruster = null;
            }

            removed_thruster_info.next_tandem_thruster.prev_tandem_thruster = removed_thruster_info.prev_tandem_thruster;
            removed_thruster_info.prev_tandem_thruster.next_tandem_thruster = removed_thruster_info.next_tandem_thruster;
            removed_thruster_info.next_tandem_thruster = removed_thruster_info.prev_tandem_thruster = removed_thruster_info;

            Dictionary<Vector3I, List<thruster_info>> cur_direction =       _tandem_thrusters[(int) removed_thruster_info.nozzle_direction];
            Vector3I                                  filter_vector = _thrust_forward_vectors[(int) removed_thruster_info.nozzle_direction], filtered_location;

            filter_vector     = Vector3I.One - filter_vector / (filter_vector.X + filter_vector.Y + filter_vector.Z);
            filtered_location = (thruster.Min + thruster.Max) * filter_vector;
            List<thruster_info> tandem_thrusters = cur_direction[filtered_location];
            tandem_thrusters.Remove(removed_thruster_info);
            if (tandem_thrusters.Count == 0)
                cur_direction.Remove(filtered_location);
        }

        private void check_thruster_control_changed()
        {
            List<thruster_info> collective_thrusters;
            IMyThrust           thruster;
            thruster_info       cur_thruster_info;
            bool                changes_made = false, contains_THR, contains_RCS, contains_STAT, use_active_control, active_control_on, disable_linear_input;
            int                 dir_index;
            string              thruster_data;

            if (_calibration_in_progress)
            {
                if (!_calibration_complete)
                    return;
                set_up_thrust_limits();
            }

            lock (_uncontrolled_thrusters)
            { 
                _thrusters_moved.Clear();
                foreach (KeyValuePair<IMyThrust, thruster_info> cur_thruster in _uncontrolled_thrusters)
                {
                    cur_thruster_info = cur_thruster.Value;
                    thruster          = cur_thruster.Key;
                    thruster_data     = thruster.CustomData;
                    contains_THR      = thruster_data.ContainsTHRTag() || DEBUG_THR_ALWAYS_ON;
                    contains_RCS      = thruster_data.ContainsRCSTag();
                    contains_STAT     = thruster_data.ContainsSTATTag();
                    if ((contains_THR || contains_RCS || contains_STAT) && cur_thruster_info.actual_max_force > 0.01f * cur_thruster_info.max_force && thruster.IsWorking)
                    {
                        _thrusters_moved.Add(thruster, cur_thruster_info);
                        cur_thruster_info.enable_rotation = cur_thruster_info.active_control_on = contains_THR || contains_RCS;
                        changes_made = true;
                    }
                }
                foreach (KeyValuePair<IMyThrust, thruster_info> cur_thruster in _thrusters_moved)
                    enable_control(cur_thruster.Key, cur_thruster.Value);

                Array.Clear(_active_control_on, 0, 6);
                for (dir_index = 0; dir_index < 6; ++dir_index)
                {
                    Dictionary<IMyThrust, thruster_info> cur_direction = _controlled_thrusters[dir_index];

                    _thrusters_moved.Clear();
                    collective_thrusters = _collective_thrusters[dir_index];
                    active_control_on = false;
                    foreach (KeyValuePair<IMyThrust, thruster_info> cur_thruster in _controlled_thrusters[dir_index])
                    {
                        cur_thruster_info = cur_thruster.Value;
                        thruster          = cur_thruster.Key;
                        thruster_data     = thruster.CustomData;
                        contains_THR      = thruster_data.ContainsTHRTag() || DEBUG_THR_ALWAYS_ON;
                        contains_RCS      = thruster_data.ContainsRCSTag();
                        contains_STAT     = !contains_RCS && thruster_data.ContainsSTATTag();
                        if (!contains_THR && !contains_RCS && !contains_STAT || cur_thruster_info.actual_max_force < 0.01f * cur_thruster_info.max_force || !thruster.IsWorking)
                        {
                            _thrusters_moved.Add(thruster, cur_thruster_info);
                            changes_made = true;
                        }
                        else
                        {
                            use_active_control = contains_THR || contains_RCS;
                            if (cur_thruster_info.active_control_on != use_active_control)
                                cur_thruster_info.enable_rotation = cur_thruster_info.active_control_on = use_active_control;
                            active_control_on       |= use_active_control || cur_thruster_info.enable_rotation;
                            cur_thruster_info.is_RCS = contains_RCS;
                            disable_linear_input     = use_active_control && thruster_data.ContainsNLTag();
                            if (cur_thruster_info.disable_linear_input != disable_linear_input)
                            {
                                _calibration_scheduled[dir_index]      = true;
                                cur_thruster_info.disable_linear_input = disable_linear_input;
                            }
                            cur_thruster_info.enable_limit = contains_STAT || disable_linear_input;
                            if (contains_STAT)
                            {
                                if (!use_active_control && _is_solution_good[dir_index])
                                {
                                    if (!collective_thrusters.Contains(cur_thruster_info))
                                        collective_thrusters.Add(cur_thruster_info);
                                }
                                else if (collective_thrusters.Contains(cur_thruster_info))
                                    collective_thrusters.Remove(cur_thruster_info);
                            }
                        }
                    }
                    foreach (KeyValuePair<IMyThrust, thruster_info> cur_thruster in _thrusters_moved)
                        disable_control(cur_thruster.Key, cur_thruster.Value);

                    switch ((thrust_dir) dir_index)
                    {
                        case thrust_dir.fore:
                        case thrust_dir.aft:
                            _active_control_on[(int) thrust_dir.port     ] |= active_control_on;
                            _active_control_on[(int) thrust_dir.starboard] |= active_control_on;
                            _active_control_on[(int) thrust_dir.dorsal   ] |= active_control_on;
                            _active_control_on[(int) thrust_dir.ventral  ] |= active_control_on;
                            break;

                        case thrust_dir.port:
                        case thrust_dir.starboard:
                            _active_control_on[(int) thrust_dir.dorsal ] |= active_control_on;
                            _active_control_on[(int) thrust_dir.ventral] |= active_control_on;
                            _active_control_on[(int) thrust_dir.fore   ] |= active_control_on;
                            _active_control_on[(int) thrust_dir.aft    ] |= active_control_on;
                            break;

                        case thrust_dir.dorsal:
                        case thrust_dir.ventral:
                            _active_control_on[(int) thrust_dir.port     ] |= active_control_on;
                            _active_control_on[(int) thrust_dir.starboard] |= active_control_on;
                            _active_control_on[(int) thrust_dir.fore     ] |= active_control_on;
                            _active_control_on[(int) thrust_dir.aft      ] |= active_control_on;
                            break;
                    }
                }
            }

            if (changes_made)
            {
                _prev_air_density = float.MinValue;
                if (!_current_mode_is_CoT)
                    update_reference_vectors_for_CoM_mode();
                /*
                log_ECU_action("check_thruster_control_changed", string.Format("{0}/{1}/{2}/{3}/{4}/{5} kN",
                    _max_force[(int) thrust_dir.fore     ] / 1000.0f,
                    _max_force[(int) thrust_dir.aft      ] / 1000.0f,
                    _max_force[(int) thrust_dir.starboard] / 1000.0f,
                    _max_force[(int) thrust_dir.port     ] / 1000.0f,
                    _max_force[(int) thrust_dir.dorsal   ] / 1000.0f,
                    _max_force[(int) thrust_dir.ventral  ] / 1000.0f));
                */
            }

            if (_individual_calibration_on && !_calibration_in_progress)
                prepare_individual_calibration();
            else
                _calibration_in_progress = _calibration_complete = false;
        }

        private void enable_control(IMyThrust cur_thruster, thruster_info cur_thruster_info)
        {
            int dir_index = (int) cur_thruster_info.nozzle_direction;

            _controlled_thrusters[dir_index].Add(cur_thruster, cur_thruster_info);
            _uncontrolled_thrusters.Remove(cur_thruster);
            _max_force[dir_index] += cur_thruster_info.max_force;
            find_tandem_and_opposite_thrusters(cur_thruster, cur_thruster_info);
            cur_thruster_info.thrust_limit    = 0.0f;
            _calibration_scheduled[dir_index] = true;
        }

        private void disable_control(IMyThrust cur_thruster, thruster_info cur_thruster_info)
        {
            int dir_index = (int) cur_thruster_info.nozzle_direction;

            remove_thruster_from_lists(cur_thruster, cur_thruster_info);
            cur_thruster_info.thrust_limit = 0.0f;
            _max_force[dir_index]         -= cur_thruster_info.max_force;
            _uncontrolled_thrusters.Add(cur_thruster, cur_thruster_info);
            _controlled_thrusters[dir_index].Remove(cur_thruster);
            _calibration_scheduled[dir_index] = true;
            cur_thruster.RefreshCustomInfo();
        }

        private float refresh_real_max_forces_for_single_direction(Dictionary<IMyThrust, thruster_info> thrusters/*, bool atmosphere_present, float air_density*/)
        {
            thruster_info cur_thruster_info;
            float         actual_max_force = 0.0f;

            foreach (KeyValuePair<IMyThrust, thruster_info> cur_thruster in thrusters)
            {
                cur_thruster_info                      = cur_thruster.Value;
                cur_thruster_info.actual_max_force     = cur_thruster.Key.MaxEffectiveThrust;
                cur_thruster_info.actual_static_moment = cur_thruster_info.static_moment * (cur_thruster_info.actual_max_force / cur_thruster_info.max_force);
                actual_max_force                      += cur_thruster_info.actual_max_force;
            }
            return actual_max_force;
        }

        private void refresh_real_max_forces_for_uncontrolled_thrusters()
        {
            thruster_info      cur_thruster_info;
            Array.Clear(_uncontrolled_max_force, 0, 6);
            foreach (KeyValuePair<IMyThrust, thruster_info> cur_thruster in _uncontrolled_thrusters)
            {
                cur_thruster_info                      = cur_thruster.Value;
                cur_thruster_info.actual_max_force     = cur_thruster.Key.MaxEffectiveThrust;
                cur_thruster_info.actual_static_moment = cur_thruster_info.static_moment * (cur_thruster_info.actual_max_force / cur_thruster_info.max_force);
                _uncontrolled_max_force[(int) cur_thruster_info.nozzle_direction] += cur_thruster_info.actual_max_force;
            }
        }

        private void refresh_real_max_forces()
        {
            BoundingBoxD grid_bounding_box = _grid.PositionComp.WorldAABB;
            MyPlanet     closest_planetoid = MyGamePruningStructure.GetClosestPlanet(ref grid_bounding_box);
            float        air_density       = (closest_planetoid == null) ? 0.0f : closest_planetoid.GetAirDensity(grid_bounding_box.Center);

            if (Math.Abs(air_density - _prev_air_density) < 0.005f)
                return;
            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                _actual_max_force[dir_index] = refresh_real_max_forces_for_single_direction(_controlled_thrusters[dir_index]);
                if (_prev_air_density >= 0.0f)
                    _calibration_scheduled[dir_index] = true;
            }
            /*
            log_ECU_action("refresh_real_max_forces", string.Format("actual forces = {0}/{1}/{2}/{3}/{4}/{5} kN",
                _actual_max_force[(int) thrust_dir.fore     ] / 1000.0f,
                _actual_max_force[(int) thrust_dir.aft      ] / 1000.0f,
                _actual_max_force[(int) thrust_dir.starboard] / 1000.0f,
                _actual_max_force[(int) thrust_dir.port     ] / 1000.0f,
                _actual_max_force[(int) thrust_dir.dorsal   ] / 1000.0f,
                _actual_max_force[(int) thrust_dir.ventral  ] / 1000.0f));
            */

            refresh_real_max_forces_for_uncontrolled_thrusters();
            _prev_air_density = air_density;

            if (_current_mode_is_CoT)
                update_reference_vectors_for_CoT_mode();
        }

        public void assign_thruster(IMyThrust thruster)
        {
            var new_thruster = new thruster_info();
            new_thruster.host_ECU             = this;
            new_thruster.grid_centre_pos      = (thruster.Min + thruster.Max) * (_grid.GridSize / 2.0f);
            new_thruster.max_force            = new_thruster.actual_max_force = thruster.MaxThrust;
            new_thruster.CoM_offset           = new_thruster.grid_centre_pos - _grid_CoM_location;
            new_thruster.static_moment        = new_thruster.actual_static_moment = new_thruster.grid_centre_pos * new_thruster.max_force;
            new_thruster.nozzle_direction     = get_nozzle_orientation(thruster);
            new_thruster.torque_factor        = Vector3.Cross(new_thruster.CoM_offset, -_thrust_forward_vectors[(int) new_thruster.nozzle_direction]);
            new_thruster.reference_vector     = get_reference_vector(new_thruster, _grid_CoM_location, _thrust_forward_vectors[(int) new_thruster.nozzle_direction]);
            new_thruster.thrust_limit         = 1.0f;
            new_thruster.enable_limit         = new_thruster.enable_rotation = new_thruster.active_control_on = false;
            new_thruster.opposing_thruster    = null;
            new_thruster.next_tandem_thruster = new_thruster.prev_tandem_thruster = new_thruster;
            new_thruster.throttle_setting     = "TTDTWM_MT_" + thruster.EntityId.ToString();

            uint manual_throttle;
            if (MyAPIGateway.Utilities.GetVariable(new_thruster.throttle_setting, out manual_throttle))
                new_thruster.manual_throttle = manual_throttle / 100.0f;
            else
                new_thruster.manual_throttle = 0.0f;

            lock (_uncontrolled_thrusters)
            {
                if (MyAPIGateway.Multiplayer == null || MyAPIGateway.Multiplayer.IsServer)
                    thruster.ThrustOverride = 0.0f;
                _uncontrolled_thrusters.Add(thruster, new_thruster);
                _prev_air_density = float.MinValue;
            }
            sync_helper.register_entity(new_thruster, thruster.EntityId);
        }

        public void dispose_thruster(IMyThrust thruster)
        {
            lock (_uncontrolled_thrusters)
            { 
                if (_uncontrolled_thrusters.ContainsKey(thruster))
                {
                    MyAPIGateway.Utilities.RemoveVariable(_uncontrolled_thrusters[thruster].throttle_setting);
                    sync_helper.deregister_entity(thruster.EntityId);
                    _uncontrolled_thrusters.Remove(thruster);
                }
                else
                {
                    for (int dir_index = 0; dir_index < 6; ++dir_index)
                    {
                        if (_controlled_thrusters[dir_index].ContainsKey(thruster))
                        {
                            _calibration_scheduled[dir_index] = true;
                            remove_thruster_from_lists(thruster, _controlled_thrusters[dir_index][thruster]);
                            MyAPIGateway.Utilities.RemoveVariable(_controlled_thrusters[dir_index][thruster].throttle_setting);
                            _max_force[dir_index] -= _controlled_thrusters[dir_index][thruster].max_force;
                            sync_helper.deregister_entity(thruster.EntityId);
                            _controlled_thrusters[dir_index].Remove(thruster);
                            break;
                        }
                    }
                }
            }
        }

        static engine_control_unit()
        {
            _thrust_forward_vectors = new Vector3I[6];
            _thrust_forward_vectors[(int) thrust_dir.fore     ] = Vector3I.Forward;
            _thrust_forward_vectors[(int) thrust_dir.aft      ] = Vector3I.Backward;
            _thrust_forward_vectors[(int) thrust_dir.port     ] = Vector3I.Left;
            _thrust_forward_vectors[(int) thrust_dir.starboard] = Vector3I.Right;
            _thrust_forward_vectors[(int) thrust_dir.dorsal   ] = Vector3I.Up;
            _thrust_forward_vectors[(int) thrust_dir.ventral  ] = Vector3I.Down;
        }

        public engine_control_unit(IMyCubeGrid grid_ref, torque_and_orbit_control grid_movement)
        {
            _grid = (MyCubeGrid) grid_ref;
            _grid_movement = grid_movement;
            refresh_turn_sensitivity();

            _control_sectors = new solver_entry[3 * 3];
            for (int index = 0; index < 3 * 3; ++index)
                _control_sectors[index] = new solver_entry();
        }

        #endregion

        #region Gyroscope handling

        private void calc_spherical_moment_of_inertia()
        {
            Vector3I grid_dim = _grid.Max - _grid.Min + Vector3I.One;
            int      low_dim  = grid_dim.X, med_dim = grid_dim.Y, high_dim = grid_dim.Z, temp;

            if (low_dim < 0)
                low_dim = -low_dim;
            if (med_dim < 0)
                med_dim = -med_dim;
            if (high_dim < 0)
                high_dim = -high_dim;
            do
            {
                temp = -1;
                if (low_dim > med_dim)
                {
                    temp    = low_dim;
                    low_dim = med_dim;
                    med_dim = temp;
                }
                if (med_dim > high_dim)
                {
                    temp     = med_dim;
                    med_dim  = high_dim;
                    high_dim = temp;
                }
            }
            while (temp >= 0);
            if (low_dim <= 0)
                low_dim = 1;
            if (med_dim <= 0)
                med_dim = 1;
            float smallest_area          = low_dim * med_dim * _grid.GridSize * _grid.GridSize;
            float reference_radius       = (float) Math.Sqrt(smallest_area / Math.PI);
            _spherical_moment_of_inertia = 0.4f * ((_grid_mass >= 1.0f) ? _grid_mass : 1.0f) * reference_radius * reference_radius;
        }

        private void refresh_gyro_info()
        {
            uint num_overriden_gyroscopes = 0;

            _gyro_override   = Vector3.Zero;
            foreach (MyGyro cur_gyroscope in _gyroscopes)
            {
                if (cur_gyroscope.IsWorking)
                {
                    if (cur_gyroscope.GyroOverride)
                    {
                        _gyro_override += cur_gyroscope.GyroOverrideVelocityGrid;
                        ++num_overriden_gyroscopes;
                    }
                }
            }

            if (autopilot_on)
            {
                _gyro_override           = Vector3.Zero;
                _is_gyro_override_active = true;
            }
            else if (num_overriden_gyroscopes > 0)
            {
                _gyro_override          /= num_overriden_gyroscopes;
                _is_gyro_override_active = true;
            }
            else if (_is_gyro_override_active)
            {
                reset_ECU();
                reset_user_input();
                _is_gyro_override_active = false;
            }
        }

        public void assign_gyroscope(IMyGyro new_gyroscope)
        {
            _gyroscopes.Add((MyGyro) new_gyroscope);
        }

        public void dispose_gyroscope(IMyGyro gyroscope_to_remove)
        {
            _gyroscopes.Remove((MyGyro) gyroscope_to_remove);
        }

        #endregion

        #region Flight controls handling

        internal static void extract_thrust_limit(object entity, byte[] argument, int length)
        {
            var thruster_entry = entity as thruster_info;

            if (thruster_entry != null)
            {
                engine_control_unit host = thruster_entry.host_ECU;

                if (host._uncontrolled_thrusters.ContainsValue(thruster_entry))
                    thruster_tagger.displayed_thrust_limit = -2;
                else if (!host._is_solution_good[(int) thruster_entry.nozzle_direction])
                    thruster_tagger.displayed_thrust_limit = -1;
                else
                    thruster_tagger.displayed_thrust_limit = (int) (thruster_entry.thrust_limit * 100.0f + 0.5f);
            }
        }

        private static void send_manual_throttle(thruster_info thruster_entry, uint manual_throttle)
        {
            __message[0] = (byte) manual_throttle;
            sync_helper.send_message_to_others(sync_helper.message_types.MANUAL_THROTTLE, thruster_entry, __message, 1);
        }

        internal static void on_manual_throttle_changed(object entity, byte[] argument, int length)
        {
            var thruster_entry = entity as thruster_info;

            if (length == 1 && thruster_entry != null)
            {
                uint new_throttle = argument[0] & 0x7FU;

                thruster_entry.manual_throttle = new_throttle / 100.0f;
                MyAPIGateway.Utilities.SetVariable(thruster_entry.throttle_setting, new_throttle);
                if ((argument[0] & 0x80) != 0)
                    send_manual_throttle(thruster_entry, new_throttle);
            }
        }

        private void check_manual_override()
        {
            thruster_info cur_thruster_info;

            _is_thrust_override_active = false;
            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                _uncontrolled_override_checked[dir_index] = false;
                _thrust_override_vector       [dir_index] = 0.0f;
            }
            foreach (KeyValuePair<IMyThrust, thruster_info> cur_thruster in _uncontrolled_thrusters)
            {
                cur_thruster_info = cur_thruster.Value;
                if (!_uncontrolled_override_checked[(int) cur_thruster_info.nozzle_direction] && cur_thruster.Key.IsWorking)
                {
                    if (cur_thruster_info.manual_throttle >= 0.01f)
                    { 
                        _thrust_override_vector[(int) cur_thruster_info.nozzle_direction] = MIN_THROTTLE;
                        _uncontrolled_override_checked[(int) cur_thruster_info.nozzle_direction] = _is_thrust_override_active = true;
                    }
                }
            }
            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                foreach (KeyValuePair<IMyThrust, thruster_info> cur_thruster in _controlled_thrusters[dir_index])
                {
                    cur_thruster_info = cur_thruster.Value;
                    if (cur_thruster_info.manual_throttle > _thrust_override_vector[dir_index] && cur_thruster.Key.IsWorking)
                    {
                        _thrust_override_vector[dir_index] = cur_thruster_info.manual_throttle;
                        _is_thrust_override_active         = true;
                    }
                }
            }
            _thrust_override = Vector3.Clamp(recompose_vector(_thrust_override_vector), neg_override_bound, pos_override_bound);
        }

        public bool is_under_control_of(VRage.Game.ModAPI.Interfaces.IMyControllableEntity current_controller)
        {
            var    controller = current_controller as MyShipController;
            return controller != null && controller.CubeGrid == _grid;
        }

        public void check_autopilot(IMyRemoteControl RC_block)
        {
            autopilot_on |= RC_block.IsAutoPilotEnabled;
        }

        public void reset_user_input()
        {
            _manual_thrust = _manual_rotation = _target_rotation = Vector3.Zero;
        }

        public void translate_player_input(Vector3 input_thrust, Vector3 input_rotation, VRage.Game.ModAPI.Interfaces.IMyControllableEntity current_controller)
        {
            _match_velocity_with = ((Sandbox.Game.Entities.IMyControllableEntity) current_controller).RelativeDampeningEntity;

            if (secondary_ECU)
                return;
            var controller = current_controller as IMyShipController;
            if (controller == null)
                return;
            if (controller.CubeGrid != _grid)
            {
                reset_user_input();
                return;
            }
            if (!controller.ControlThrusters || _grid.HasMainCockpit() && !controller.IsMainCockpit)
                return;

            Matrix cockpit_matrix;
            controller.Orientation.GetMatrix(out cockpit_matrix);
            _manual_thrust     = Vector3.Clamp(Vector3.Transform(input_thrust, cockpit_matrix), -Vector3.One, Vector3.One);
            _target_rotation.X = input_rotation.X * (-0.05f);
            _target_rotation.Y = input_rotation.Y * (-0.05f);
            _target_rotation.Z = input_rotation.Z * (-0.2f);
            _target_rotation   = Vector3.Transform(_target_rotation, cockpit_matrix);
        }

        public void get_primary_control_parameters(out Vector3D world_linear_velocity, out Vector3 target_linear_velocity, out Vector3D world_angular_velocity, 
            out Vector3 linear_output, out Vector3 rotational_output, out bool main_gyro_override_active, out Vector3 main_gyro_override, 
            out bool circularisation_on, out ID_manoeuvres current_manoeuvre)
        {
            if (secondary_ECU)
                throw new Exception("Attempt to obtain control parameters from secondary grid");

            world_linear_velocity     = _world_linear_velocity;
            target_linear_velocity    = _target_velocity;
            world_angular_velocity    = _world_angular_velocity;
            MatrixD grid_orientation  = _grid.WorldMatrix.GetOrientation();
            linear_output             = Vector3.Transform(_manual_thrust  , grid_orientation);
            rotational_output         = Vector3.Transform(_manual_rotation, grid_orientation);
            main_gyro_override        = Vector3.Transform(_gyro_override  , grid_orientation);
            main_gyro_override_active = _is_gyro_override_active;
            circularisation_on        = _circularise_on;
            current_manoeuvre         = this.current_manoeuvre;
        }

        public void set_secondary_control_parameters(Vector3D world_linear_velocity, Vector3 target_linear_velocity, Vector3D world_angular_velocity, 
            Vector3 linear_input, Vector3 rotational_input, bool main_gyro_override_active, Vector3 main_gyro_override,
            bool circularisation_on, ID_manoeuvres current_manoeuvre)
        {
            if (!secondary_ECU)
                throw new Exception("Attempt to set external control parameters to primary grid");

            _world_linear_velocity       = world_linear_velocity;
            _target_velocity             = target_linear_velocity;
            _world_angular_velocity      = world_angular_velocity;
            MatrixD grid_orientation_inv = _grid.PositionComp.WorldMatrixNormalizedInv.GetOrientation();
            _manual_thrust               = Vector3.Transform(linear_input      , grid_orientation_inv);
            _manual_rotation             = Vector3.Transform(rotational_input  , grid_orientation_inv);
            _gyro_override               = Vector3.Transform(main_gyro_override, grid_orientation_inv);
            _is_gyro_override_active     = main_gyro_override_active;
            _circularise_on              = circularisation_on;
            this.current_manoeuvre       = current_manoeuvre;
        }

        public void translate_damper_override(Vector3 input_override, IMyTerminalBlock controller)
        {
            Matrix cockpit_matrix;
            controller.Orientation.GetMatrix(out cockpit_matrix);

            Vector3 cockpit_damper_axis_disable = Vector3.Transform(input_override, cockpit_matrix);
            _dampers_axis_enable.X = (Math.Abs(cockpit_damper_axis_disable.X) >= 0.5f) ? 0.0f : 1.0f;
            _dampers_axis_enable.Y = (Math.Abs(cockpit_damper_axis_disable.Y) >= 0.5f) ? 0.0f : 1.0f;
            _dampers_axis_enable.Z = (Math.Abs(cockpit_damper_axis_disable.Z) >= 0.5f) ? 0.0f : 1.0f;
        }

        public Vector3 get_damper_override_for_cockpit(IMyTerminalBlock controller)
        {
            Matrix inv_cockpit_matrix;
            controller.Orientation.GetMatrix(out inv_cockpit_matrix);
            inv_cockpit_matrix = Matrix.Invert(inv_cockpit_matrix);

            Vector3 result = Vector3.Transform(_dampers_axis_enable, inv_cockpit_matrix);
            result.X = (Math.Abs(result.X) >= 0.5f) ? 0.0f : 1.0f;
            result.Y = (Math.Abs(result.Y) >= 0.5f) ? 0.0f : 1.0f;
            result.Z = (Math.Abs(result.Z) >= 0.5f) ? 0.0f : 1.0f;
            return result;
        }

        public void begin_manoeuvre(ID_manoeuvres selection)
        {
            current_manoeuvre = (current_manoeuvre == selection) ? ID_manoeuvres.manoeuvre_off : selection;
        }

        #endregion

        public void reset_ECU()
        {
            _prev_angular_velocity = Vector3.Zero;
            for (int dir_index = 0; dir_index < 6; ++dir_index)
                _current_trim[dir_index] = _last_trim[dir_index] = _linear_integral[dir_index] = 0.0f;
        }

        private void refresh_turn_sensitivity()
        {
            const float SENSITIVITY_MULT = 0.5f;
            Vector3 ship_size         = (_grid.Max - _grid.Min) * _grid.GridSize;
            float   average_grid_mass = _total_mass / _num_subgrids;

            if (average_grid_mass > _grid_mass)
                ship_size *= average_grid_mass / _grid_mass;
            _turn_sensitivity[(int) thrust_dir.port  ] = _turn_sensitivity[(int) thrust_dir.starboard] = Math.Max(ship_size.Y, ship_size.Z) * SENSITIVITY_MULT;  // pitch
            _turn_sensitivity[(int) thrust_dir.dorsal] = _turn_sensitivity[(int) thrust_dir.ventral  ] = Math.Max(ship_size.X, ship_size.Z) * SENSITIVITY_MULT;  // yaw
            _turn_sensitivity[(int) thrust_dir.fore  ] = _turn_sensitivity[(int) thrust_dir.aft      ] = Math.Max(ship_size.X, ship_size.Y) * SENSITIVITY_MULT;  // roll
        }

        public void handle_60Hz()
        {
            if (!_grid_is_movable)
            {
                _physics_enable_delay = PHYSICS_ENABLE_DELAY;
                current_speed         = 0.0f;
                return;
            }

            bool is_secondary = secondary_ECU;
            if (is_secondary)
            {
                _circularise_on  = false;
                _target_velocity = Vector3.Zero;
                current_speed    = (float) _world_linear_velocity.Length();
            }
            else
            {
                // Suppress input noise caused by analog controls
                _sample_sum += _target_rotation - _rotation_samples[_current_index];
                _rotation_samples[_current_index] = _target_rotation;
                if (++_current_index >= NUM_ROTATION_SAMPLES)
                    _current_index = 0;
                _manual_rotation = _sample_sum / NUM_ROTATION_SAMPLES;

                if (current_speed < MIN_CIRCULARISATION_SPEED || _linear_control.LengthSquared() >= 0.0001f)
                    current_manoeuvre = ID_manoeuvres.manoeuvre_off;
                _grid_movement.get_linear_and_angular_velocities(out _world_linear_velocity, out _world_angular_velocity);
                current_speed    = (float) _world_linear_velocity.Length();
                _circularise_on &= current_speed >= MIN_CIRCULARISATION_SPEED;
                if (_match_velocity_with?.Physics != null)
                    _target_velocity = _match_velocity_with.Physics.LinearVelocity;
                else if (current_manoeuvre != ID_manoeuvres.manoeuvre_off)
                    _target_velocity = _world_linear_velocity + _grid_movement.get_manoeuvre_direction(current_manoeuvre) * current_speed;
                else if (_circularise_on)
                    _target_velocity = _grid_movement.get_circular_orbit_velocity(!linear_dampers_on || _linear_control.LengthSquared() > 0.0001f);
                else
                    _target_velocity = Vector3.Zero;
            }

            _grid_mass = _grid.Physics.Mass;
            if (_grid_mass < 1.0f)
                _grid_mass = 1.0f;

            if (!is_secondary)
                refresh_gyro_info();
            else
            {
                Vector3D current_grid_CoM = Vector3D.Transform(get_world_combined_CoM(), _grid.PositionComp.WorldMatrixNormalizedInv);
                bool     CoM_shifted      = (current_grid_CoM - _grid_CoM_location).LengthSquared() > 0.01f;

                if (CoM_shifted)
                {
                    _grid_CoM_location = current_grid_CoM;
                    refresh_reference_vectors(CoM_shifted: true);
                }
            }

            if (  autopilot_on || !_is_thrust_override_active && !_is_gyro_override_active 
                && _manual_rotation.LengthSquared() < 0.0001f && _manual_thrust.LengthSquared() < 0.0001f && current_manoeuvre == ID_manoeuvres.manoeuvre_off
                && (!rotational_damping_on || _world_angular_velocity.LengthSquared() < 0.0001f)
                && (!linear_dampers_on && !secondary_ECU || _grid.Physics.Gravity.LengthSquared() < 0.01f && current_speed < 0.1f))
            {
                handle_thrust_control(_world_linear_velocity, _target_velocity, _world_angular_velocity, sleep_mode_on: true);
                if (autopilot_on)
                    calculate_and_apply_torque();
            }
            else
            {
                handle_thrust_control(_world_linear_velocity, _target_velocity, _world_angular_velocity, sleep_mode_on: false);
                calculate_and_apply_torque();
            }
        }

        public void handle_4Hz_foreground()
        {
            if (!_grid_is_movable)
                reset_ECU();
            else
            {
                Vector3D current_grid_CoM = Vector3D.Transform(get_world_combined_CoM(), _grid.PositionComp.WorldMatrixNormalizedInv);
                lock (_uncontrolled_thrusters)
                {
                    _CoM_shifted |= (current_grid_CoM - _grid_CoM_location).LengthSquared() > 0.01f;
                    if (_CoM_shifted)
                        _grid_CoM_location = current_grid_CoM;
                }
            }
        }

        public void handle_4Hz_background()
        {
            if (!_grid_is_movable)
                return;

            if (!_calibration_in_progress)
            {
                bool CoM_shifted;

                lock (_uncontrolled_thrusters)
                {
                    CoM_shifted  = _CoM_shifted;
                    _CoM_shifted = false;
                }
                refresh_reference_vectors(CoM_shifted);
                if (use_individual_calibration != _individual_calibration_on)
                {
                    _individual_calibration_on = use_individual_calibration;
                    for (int dir_index = 0; dir_index < 6; ++dir_index)
                        _calibration_scheduled[dir_index] = true;
                }
                refresh_real_max_forces();
                calc_spherical_moment_of_inertia();
                check_manual_override();
                if (!_individual_calibration_on)
                    perform_quadrant_calibration();
            }
        }

        public void handle_2s_period_foreground()
        {
            _grid_is_movable = !_grid.IsStatic && _grid.Physics != null && _grid.Physics.Enabled;
            if (!_grid_is_movable)
                return;

            thruster_info cur_thrust_info;

            foreach (KeyValuePair<IMyThrust, thruster_info> cur_thruster in _uncontrolled_thrusters)
            {
                cur_thrust_info = cur_thruster.Value;

                if (cur_thrust_info.enable_rotation || cur_thrust_info.enable_limit)
                {
                    if (MyAPIGateway.Multiplayer == null || MyAPIGateway.Multiplayer.IsServer)
                        cur_thruster.Key.ThrustOverride = 0.0f;
                    cur_thrust_info.enable_limit = cur_thrust_info.enable_rotation = cur_thrust_info.active_control_on = cur_thrust_info.is_RCS = false;
                }
            }

            _force_override_refresh = true;
            refresh_turn_sensitivity();
        }

        public void handle_2s_period_background()
        {
            if (_grid_is_movable)
                check_thruster_control_changed();
        }

        public void perform_individual_calibration()
        {
            if (!_individual_calibration_on || !_calibration_in_progress || !_calibration_ready || _calibration_complete)
                return;

            _calibration_ready = false;
            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                if (_calibration_scheduled[dir_index])
                {
                    if (CALIBRATION_DEBUG)
                        log_ECU_action("perform_individual_calibration", "Starting calibration on " + ((thrust_dir) dir_index).ToString() + " side");
                    _is_solution_good[dir_index] = _linear_solvers[dir_index].calculate_solution(_thruster_infos[dir_index].Count);
                }
            }
            _calibration_complete = true;
        }
    }
}
