using System;
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
        #region fields

        const int   NUM_ROTATION_SAMPLES = 6, PHYSICS_ENABLE_DELAY = 6;
        const float DESCENDING_SPEED     = 0.5f, MIN_OVERRIDE = 1.001f, LINEAR_INTEGRAL_CONSTANT = 0.05f, MIN_THROTTLE = 0.01f;
        const bool  DEBUG_THR_ALWAYS_ON  = false, DEBUG_DISABLE_ALT_HOLD = false;

        enum thrust_dir { fore = 0, aft = 3, starboard = 1, port = 4, dorsal = 2, ventral = 5 };
        sealed class thruster_info     // Technically a struct
        {
            public IMyThrust     host_thruster;
            public float         max_force, actual_max_force;
            public Vector3I      grid_cells;
            public Vector3       torque_factor, grid_centre_pos, static_moment, actual_static_moment, CoM_offset, reference_vector;
            public thrust_dir    nozzle_direction;
            public float         current_setting, thrust_limit, prev_setting, manual_throttle;
            public bool          enable_limit, enable_rotation, steering_on, is_RCS, disable_linear_input, throttle_up, apply_limit, collective_control_on, operational;
            public thruster_info next_tandem_thruster, prev_tandem_thruster, opposing_thruster;
            public int           control_sector;
        };

        private static readonly Vector3I[] _thrust_forward_vectors;

        private readonly torque_control           _grid_movement;
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

        private static readonly float[] __steering_input          = new float[6];
        private static readonly float[] __angular_velocity        = new float[6];
        private static readonly float[] __target_angular_velocity = new float[6];
        private static readonly float[] __angular_velocity_diff   = new float[6];
        private static readonly float[] __angular_acceleration    = new float[6];
        private static readonly float[] __residual_torque         = new float[6];
        private static readonly float[] __gyro_override           = new float[6];
        private static readonly float[] __thrust_limits           = new float[6];

        private static readonly Vector3[] __new_static_moment = new Vector3[6];
        private static readonly float[]   __new_total_force   = new   float[6];

        private static readonly float[] __local_gravity_inv         = new float[6];
        private static readonly float[] __local_linear_velocity_inv = new float[6];

        private readonly MyCubeGrid _grid;

        private readonly HashSet<HashSet<thruster_info>> _thruster_groups_to_reset = new HashSet<HashSet<thruster_info>>();
        private readonly HashSet<thruster_info>[] _controlled_thrusters =
        {
            new HashSet<thruster_info>(),   // fore
            new HashSet<thruster_info>(),   // starboard
            new HashSet<thruster_info>(),   // dorsal
            new HashSet<thruster_info>(),   // aft
            new HashSet<thruster_info>(),   // port
            new HashSet<thruster_info>()    // ventral
        };
        private readonly HashSet<thruster_info>[] _steering_thrusters =
        {
            new HashSet<thruster_info>(),   // fore
            new HashSet<thruster_info>(),   // starboard
            new HashSet<thruster_info>(),   // dorsal
            new HashSet<thruster_info>(),   // aft
            new HashSet<thruster_info>(),   // port
            new HashSet<thruster_info>()    // ventral
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
        private readonly HashSet<thruster_info>[] _collective_thrusters =
        {
            new HashSet<thruster_info>(),  // fore
            new HashSet<thruster_info>(),  // starboard
            new HashSet<thruster_info>(),  // dorsal
            new HashSet<thruster_info>(),  // aft
            new HashSet<thruster_info>(),  // port
            new HashSet<thruster_info>(),  // ventral
        };
        private readonly Dictionary<long, thruster_info> _all_thrusters = new Dictionary<long, thruster_info>();
        private readonly HashSet<thruster_info> _uncontrolled_thrusters   = new HashSet<thruster_info>();
        private readonly HashSet<thruster_info> _thrusters_reset_override = new HashSet<thruster_info>();
        private readonly HashSet<thruster_info> _changed_thrusters1       = new HashSet<thruster_info>();
        private readonly HashSet<thruster_info> _changed_thrusters2       = new HashSet<thruster_info>();
        private          HashSet<thruster_info> _changed_thrusters;
        private readonly float[] _max_force              = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        private readonly float[] _actual_max_force       = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        private readonly float[] _uncontrolled_max_force = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        private readonly float[] _thrust_override_vector = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        private readonly float[] _total_force            = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        private readonly float[] _surface_radii          = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };

        private readonly HashSet<MyGyro> _gyroscopes = new HashSet<MyGyro>();

        private bool     _calibration_in_progress = false;
        private Vector3D _grid_CoM_location = Vector3D.Zero, _world_linear_velocity;
        private Vector3D _world_angular_velocity;
        private float    _spherical_moment_of_inertia = 1.0f, _grid_mass = 1.0f;

        private readonly  bool[]    _uncontrolled_override_checked = new bool[6];
        private readonly  bool[]    _is_solution_good = { false, false, false, false, false, false };
        private readonly  bool[]    _steering_enabled = { false, false, false, false, false, false };
        private readonly float[]    _smoothed_acceleration       = new float[6];
        private readonly float[]    _last_angular_velocity       = new float[6];
        private readonly float[]    _current_trim                = new float[6];
        private readonly float[]    _aux_trim                    = new float[6];
        private readonly float[]    _angular_velocity_checkpoint = new float[6];
        private readonly Vector3?[] _active_CoT = new Vector3?[6];

        private Vector3 _local_angular_velocity, _prev_angular_velocity = Vector3.Zero, _torque, _manual_rotation, _target_velocity;
        private Vector3 _target_rotation, _gyro_override = Vector3.Zero, _dampers_axes_enabled = new Vector3(1.0f);
        private bool    _CoM_shifted = false, _current_mode_is_CoT = false, _new_mode_is_CoT = false;
        private bool    _is_gyro_override_active = false, _individual_calibration_on = false, _calibration_ready = false, _calibration_complete = false, _calibration_interrupted = false;
        private bool    _all_engines_off = false, _force_override_refresh = false, _dry_run = false;
        private float   _trim_fadeout = 1.0f;
        private bool    _integral_cleared = false, _is_thrust_override_active = false, _thruster_check_in_progress;

        private readonly  bool[] _enable_linear_integral = { true, true, true, true, true, true };
        private readonly float[] _linear_integral        = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

        private float     _air_density = float.MinValue, _counter_thrust_limit = 1.0f;
        private Vector3   _manual_thrust, _thrust_override = Vector3.Zero, _linear_control = Vector3.Zero;

        private readonly Vector3[] _rotation_samples = new Vector3[NUM_ROTATION_SAMPLES];
        private          Vector3   _sample_sum       = Vector3.Zero;
        private          int       _current_index    = 0, _physics_enable_delay = PHYSICS_ENABLE_DELAY;

        private MyEntity _match_velocity_with = null;

        #endregion

        #region Properties

        public bool linear_dampers_on          { get; set; }
        public bool rotational_damping_on      { get; set; } = true;
        public bool autopilot_on               { get; set; }
        public bool use_individual_calibration { get; set; }
        public bool secondary_ECU              { get; set; }

        public Vector3D new_grid_CoM      { get; set; }
        public float    average_grid_mass { get; set; } = 1.0f;
        
        public int  thrust_reduction      { get; private set; }

        public bool active_control_enabled
        {
            get
            {
                for (int dir_index = 0; dir_index < 3; ++dir_index)
                {
                    if (_steering_enabled[dir_index])
                        return true;
                }
                return false;
            }
        }

        public float vertical_speed { get; private set; }
        
        public bool touchdown_mode_on { get; set; } = false;
        public bool       CoT_mode_on { get; set; } = false;

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

        public Vector3 aux_trim
        {
            get
            {
                Vector3 result = recompose_vector(_aux_trim);
                return result;
            }
            set
            {
                decompose_vector(value, _aux_trim);
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

        public Vector3 dampers_axes_enabled => _dampers_axes_enabled;

        #endregion

        #region DEBUG

        public const bool CALIBRATION_DEBUG = false, FULL_CALIBRATION_DEBUG = false;

        private void log_ECU_action(string method_name, string message)
        {
            MyLog.Default.WriteLine(string.Format("TTDTWM\tengine_control_unit<{0} [{1}]>.{2}(): {3}", _grid.DisplayName, _grid.EntityId, method_name, message));
            int num_controlled_thrusters = 0;
            foreach (HashSet<thruster_info> cur_direction in _controlled_thrusters)
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
            screen_info.screen_text(_grid, method_name, string.Format("{0} = {1:F5}/{2:F5}/{3:F5}/{4:F5}/{5:F5}/{6:F5}", 
                vector_name,
                vector[(int) thrust_dir.fore     ],
                vector[(int) thrust_dir.aft      ],
                vector[(int) thrust_dir.starboard],
                vector[(int) thrust_dir.port     ],
                vector[(int) thrust_dir.dorsal   ],
                vector[(int) thrust_dir.ventral  ]), display_time_ms);
        }

        private void dual_screen_vector<type>(bool global_msg, string method_name, string vector_name, type[] vector, int display_time_ms)
        {
            screen_info.dual_screen_text(global_msg ? null : _grid, vector_name, method_name, string.Format("{0} = {1:F5}/{2:F5}/{3:F5}/{4:F5}/{5:F5}/{6:F5}",
                vector_name,
                vector[(int) thrust_dir.fore],
                vector[(int) thrust_dir.aft],
                vector[(int) thrust_dir.starboard],
                vector[(int) thrust_dir.port],
                vector[(int) thrust_dir.dorsal],
                vector[(int) thrust_dir.ventral]), display_time_ms);
        }

        #endregion

        #region torque calculation

        private void refresh_thruster_info_for_single_direction(HashSet<thruster_info> thrusters)
        {
            IMyThrust cur_thruster;

            foreach (thruster_info cur_thruster_info in thrusters)
            {
                cur_thruster = cur_thruster_info.host_thruster;
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

        private void calculate_and_apply_torque()
        {
            //if (MyAPIGateway.Multiplayer != null && !MyAPIGateway.Multiplayer.IsServer)
            //    return;

            Vector3 torque = Vector3.Zero;
            foreach (HashSet<thruster_info> cur_direction in _controlled_thrusters)
            {
                foreach (thruster_info cur_thruster_info in cur_direction)
                {
                    if (cur_thruster_info.operational)
                        torque += cur_thruster_info.torque_factor * cur_thruster_info.host_thruster.CurrentThrust;
                }
            }
           
            foreach (thruster_info cur_thruster_info in _uncontrolled_thrusters)
            {
                if (cur_thruster_info.operational)
                    torque += cur_thruster_info.torque_factor * cur_thruster_info.host_thruster.CurrentThrust;
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

            _torque = torque;
            if (_physics_enable_delay > 0)
                --_physics_enable_delay;
            else if (torque.LengthSquared() >= 1.0f)
            {
                Vector3 world_torque = Vector3.Transform(torque, _grid.WorldMatrix.GetOrientation());
                _grid_movement.apply_torque(world_torque);
            }
        }

        #endregion

        #region thrust calibration

        private void prepare_individual_calibration()
        {
            float x = 0.0f, y = 0.0f;

            _calibration_interrupted = false;
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
                foreach (thruster_info cur_thruster_info in _controlled_thrusters[(int) cur_direction])
                {
                    if (!cur_thruster_info.disable_linear_input)
                        thruster_infos.Add(cur_thruster_info);
                    else
                    {
                        cur_thruster_info.enable_limit = true;
                        cur_thruster_info.thrust_limit = 0.0f;
                    }
                }

                List<solver_entry> items = linear_solver.items;
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
                        items.Add(new solver_entry());
                    items[index].x = x;
                    items[index].y = y;
                    items[index].max_value = thruster_infos[index].actual_max_force;
                }
            }
            _calibration_ready = _calibration_in_progress;
        }

        private void set_up_thrust_limits()
        {
            foreach (thrust_dir cur_direction in Enum.GetValues(typeof(thrust_dir)))
            {
                if (_calibration_interrupted || !_calibration_scheduled[(int) cur_direction])
                    continue;

                _calibration_scheduled[(int) cur_direction] = false;
                List<thruster_info>    thruster_infos = _thruster_infos[(int) cur_direction];
                revised_simplex_solver linear_solver  = _linear_solvers[(int) cur_direction];

                if (!_is_solution_good[(int) cur_direction])
                {
                    for (int index = 0; index < thruster_infos.Count; ++index)
                    {
                        thruster_infos[index].thrust_limit    = 1.0f;
                        thruster_infos[index].enable_rotation = true;
                    }
                }
                else
                {
                    List<solver_entry> items = linear_solver.items;
                    for (int index = 0; index < thruster_infos.Count; ++index)
                    {
                        thruster_infos[index].thrust_limit = (items[index].max_value > 1.0f)
                            ? (items[index].result / items[index].max_value) : 1.0f;
                        thruster_infos[index].enable_rotation = thruster_infos[index].steering_on;
                        if (CALIBRATION_DEBUG)
                            log_ECU_action("set_up_thrust_limits", string.Format("{0}/{1} kN ({2})", items[index].result / 1000.0f, items[index].max_value / 1000.0f, cur_direction));
                    }
                }

                if (CALIBRATION_DEBUG)
                {
                    log_ECU_action("set_up_thrust_limits", _is_solution_good[(int) cur_direction]
                        ? string.Format("successfully calibrated {0} thrusters on {1} side", thruster_infos.Count, cur_direction)
                        : string.Format("calibration on {0} side failed", cur_direction));
                }
            }
            _calibration_in_progress = _calibration_complete = _calibration_interrupted = false;
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
            solver_entry[] control_sectors = _control_sectors;

            foreach (thrust_dir cur_direction in Enum.GetValues(typeof(thrust_dir)))
            {
                if (!_calibration_scheduled[(int) cur_direction])
                    continue;

                HashSet<thruster_info> thruster_infos = _controlled_thrusters[(int) cur_direction];
                revised_simplex_solver linear_solver  = _linear_solvers      [(int) cur_direction];

                for (sector_index = 0; sector_index < 3 * 3; ++sector_index)
                    control_sectors[sector_index].x = control_sectors[sector_index].y = control_sectors[sector_index].max_value = 0.0f;
                foreach (thruster_info cur_thruster_info in thruster_infos)
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
                    control_sectors[control_sector].x         += x * cur_thruster_info.actual_max_force;
                    control_sectors[control_sector].y         += y * cur_thruster_info.actual_max_force;
                    control_sectors[control_sector].max_value += cur_thruster_info.actual_max_force;
                }

                active_sectors = 0;
                List<solver_entry> items = linear_solver.items;
                for (control_sector = 0; control_sector < 3 * 3; ++control_sector)
                {
                    if (control_sectors[control_sector].max_value < 1.0f)
                        continue;

                    if (CALIBRATION_DEBUG)
                    {
                        log_ECU_action("perform_quadrant_calibration", string.Format("{4} sector {0} max {1} kN at {2}, {3}", 
                            get_sector_name(control_sector), 
                            control_sectors[control_sector].max_value / 1000.0f, 
                            control_sectors[control_sector].x / control_sectors[control_sector].max_value, 
                            control_sectors[control_sector].y / control_sectors[control_sector].max_value,
                            cur_direction));
                    }
                    if (active_sectors >= linear_solver.items.Count)
                        items.Add(new solver_entry());
                    items[active_sectors  ].x         = control_sectors[control_sector].x / control_sectors[control_sector].max_value;
                    items[active_sectors  ].y         = control_sectors[control_sector].y / control_sectors[control_sector].max_value;
                    items[active_sectors++].max_value = control_sectors[control_sector].max_value;
                }

                if (CALIBRATION_DEBUG)
                    log_ECU_action("perform_quadrant_calibration", "Starting calibration on " + cur_direction.ToString() + " side");
                _is_solution_good[(int) cur_direction] = linear_solver.calculate_solution(active_sectors);
                if (!_is_solution_good[(int) cur_direction])
                {
                    if (CALIBRATION_DEBUG)
                        log_ECU_action("perform_quadrant_calibration", "Calibration on " + cur_direction.ToString() + " side failed");
                    foreach (thruster_info cur_thruster_info in thruster_infos)
                    {
                        cur_thruster_info.thrust_limit    = 1.0f;
                        cur_thruster_info.enable_rotation = true;
                    }
                }
                else
                {
                    sector_index = 0;
                    for (control_sector = 0; control_sector < 3 * 3; ++control_sector)
                    {
                        if (control_sectors[control_sector].max_value < 1.0f)
                            control_sectors[control_sector].result = 0.0f;
                        else
                        {
                            control_sectors[control_sector].result = items[sector_index].result / items[sector_index].max_value;
                            ++sector_index;
                            if (CALIBRATION_DEBUG)
                            {
                                log_ECU_action("perform_quadrant_calibration", string.Format("{3} sector {0} = {1} kN ({2} %)", 
                                    get_sector_name(control_sector), 
                                    control_sectors[control_sector].result * control_sectors[control_sector].max_value / 1000.0f, 
                                    control_sectors[control_sector].result * 100.0f,
                                    cur_direction));
                            }
                        }
                    }
                    foreach (thruster_info cur_thruster_info in thruster_infos)
                    {
                        if (cur_thruster_info.disable_linear_input)
                        {
                            cur_thruster_info.enable_limit = true;
                            cur_thruster_info.thrust_limit = 0.0f;
                            continue;
                        }
                        cur_thruster_info.thrust_limit    = control_sectors[cur_thruster_info.control_sector].result;
                        cur_thruster_info.enable_rotation = cur_thruster_info.steering_on;
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

        private void reset_thrusters(HashSet<thruster_info> thrusters)
        {
            IMyThrust thruster;

            if (thrusters != null)
            {
                _thruster_groups_to_reset.Add(thrusters);
                foreach (thruster_info cur_thruster_info in thrusters)
                {
                    thruster = cur_thruster_info.host_thruster;
                    if (thruster.ThrustOverride > MIN_OVERRIDE)
                        thruster.ThrustOverride = MIN_OVERRIDE;
                    cur_thruster_info.current_setting = cur_thruster_info.prev_setting = 0.01f;
                }
            }
            else
            {
                foreach (HashSet<thruster_info> cur_group in _thruster_groups_to_reset)
                {
                    foreach (thruster_info cur_thruster_info in cur_group)
                    {
                        thruster = cur_thruster_info.host_thruster;
                        if (thruster.ThrustOverride > 0.0f)
                            thruster.ThrustOverride = 0.0f;
                        cur_thruster_info.current_setting = cur_thruster_info.prev_setting = 0.0f;
                    }
                }
                _thruster_groups_to_reset.Clear();
            }
        }

        private void apply_thrust_settings(bool reset_all_thrusters)
        {
            const float MIN_THRUST_CHANGE = 1.0E+3f;

            float     setting;
            bool      dry_run, force_override_refresh = _force_override_refresh;
            IMyThrust thruster;

            if (reset_all_thrusters && _all_engines_off && !force_override_refresh)
                return;

            IMyMultiplayer network_info = MyAPIGateway.Multiplayer;
            if (network_info == null)
                dry_run = false;
            else
            {
                IMyPlayer controlling_player = network_info.Players.GetPlayerControllingEntity(_grid);
                //dry_run = (controlling_player == null) ? !network_info.IsServer : (controlling_player != screen_info.local_player);
                bool controls_active = _manual_rotation.LengthSquared() >= 0.0001f || _linear_control.LengthSquared() >= 0.0001f;
                if (sync_helper.running_on_server)
                    _dry_run = dry_run = controls_active && controlling_player != null && controlling_player != screen_info.local_player;
                else
                    _dry_run = dry_run = !controls_active || controlling_player == null || controlling_player != screen_info.local_player;
            }

            if (reset_all_thrusters)
            {
                for (int dir_index = 0; dir_index < 6; ++dir_index)
                {
                    _current_trim[dir_index] = 0.0f;
                    reset_thrusters(_controlled_thrusters[dir_index]);
                }
            }
            else
            {
                if (force_override_refresh)
                {
                    for (int dir_index = 0; dir_index < 6; ++dir_index)
                    {
                        foreach (thruster_info cur_thruster_info in _controlled_thrusters[dir_index])
                            cur_thruster_info.prev_setting = cur_thruster_info.host_thruster.CurrentThrust;
                    }
                }

                if (dry_run)
                {
                    for (int dir_index = 0; dir_index < 6; ++dir_index)
                    {
                        foreach (thruster_info cur_thruster_info in _controlled_thrusters[dir_index])
                        {
                            if ((cur_thruster_info.actual_max_force < 1.0f || !cur_thruster_info.operational) && cur_thruster_info.prev_setting > 0.0f)
                            {
                                cur_thruster_info.current_setting = cur_thruster_info.prev_setting = 0.0f;
                                continue;
                            }

                            setting = cur_thruster_info.current_setting * cur_thruster_info.max_force;
                            if (setting < MIN_OVERRIDE)
                                setting = MIN_OVERRIDE;
                            if (Math.Abs(setting - cur_thruster_info.prev_setting) >= MIN_THRUST_CHANGE
                                || (setting >= cur_thruster_info.max_force) != (cur_thruster_info.prev_setting >= cur_thruster_info.max_force)
                                || (setting <=                MIN_OVERRIDE) != (cur_thruster_info.prev_setting <=                MIN_OVERRIDE))
                            {
                                cur_thruster_info.prev_setting = setting;
                            }
                        }
                    }
                }
                else
                {
                    for (int dir_index = 0; dir_index < 6; ++dir_index)
                    {
                        foreach (thruster_info cur_thruster_info in _controlled_thrusters[dir_index])
                        {
                            thruster = cur_thruster_info.host_thruster;
                            if ((cur_thruster_info.actual_max_force < 1.0f || !cur_thruster_info.operational) && cur_thruster_info.prev_setting > 0.0f)
                            {
                                thruster.ThrustOverride = 0.0f;
                                cur_thruster_info.current_setting = cur_thruster_info.prev_setting = 0.0f;
                                continue;
                            }

                            setting = cur_thruster_info.current_setting * cur_thruster_info.max_force;
                            if (setting < MIN_OVERRIDE)
                                setting = MIN_OVERRIDE;
                            if (Math.Abs(setting - cur_thruster_info.prev_setting) >= MIN_THRUST_CHANGE
                                || (setting >= cur_thruster_info.max_force) != (cur_thruster_info.prev_setting >= cur_thruster_info.max_force)
                                || (setting <=                MIN_OVERRIDE) != (cur_thruster_info.prev_setting <=                MIN_OVERRIDE))
                            {
                                thruster.ThrustOverride = setting;
                                cur_thruster_info.prev_setting = setting;
                            }
                        }
                    }
                }
            }

            _all_engines_off        = reset_all_thrusters;
            _force_override_refresh = false;
        }

        private void initialise_linear_controls(Vector3 local_linear_velocity_vector, Vector3 local_gravity_vector)
        {
            const float DAMPING_CONSTANT = -2.0f;

            float[] control_vector    =    __control_vector, manual_control_vector     = __manual_control_vector, 
                    local_gravity_inv = __local_gravity_inv, local_linear_velocity_inv = __local_linear_velocity_inv, 
                    actual_max_force  =   _actual_max_force, total_force = _total_force, linear_integral = _linear_integral;
            bool[]  enable_linear_integral = _enable_linear_integral;
            float   vertical_speed = this.vertical_speed;

            _linear_control = Vector3.Clamp(_manual_thrust + _thrust_override, -Vector3.One, Vector3.One);
            decompose_vector(_linear_control, control_vector);
            for (int dir_index = 0, opposite_dir = 3; dir_index < 3; ++dir_index, ++opposite_dir)
            {
                if (total_force[dir_index] < 1.0f && total_force[opposite_dir] < 1.0f)
                    control_vector[dir_index] = control_vector[opposite_dir] = 0.0f;
            }
            float gravity_magnitude = local_gravity_vector.Length();
            bool  controls_active   = _linear_control.LengthSquared() > 0.0001f;

            _trim_fadeout = 1.0f;

            if (gravity_magnitude < 0.01f)
                _counter_thrust_limit = 1.0f;
            if (!linear_dampers_on)
            {
                _counter_thrust_limit = 1.0f;
                if (!_integral_cleared)
                {
                    for (int dir_index = 0; dir_index < 6; ++dir_index)
                    {
                        enable_linear_integral[dir_index] = false;
                        linear_integral[dir_index]        = __braking_vector[dir_index] = 0.0f;
                    }
                    _integral_cleared = true;
                }
                if (!controls_active && gravity_magnitude > 0.1f && vertical_speed > -DESCENDING_SPEED * 0.5f && vertical_speed < DESCENDING_SPEED * 0.5f)
                    _trim_fadeout = Math.Abs(vertical_speed / (DESCENDING_SPEED * 0.5f));
            }
            else
            {
                _integral_cleared = false;
                if (!touchdown_mode_on)
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
                float   average_grid_mass = this.average_grid_mass;
                linear_damping           *= (average_grid_mass >= _grid_mass) ? average_grid_mass : _grid_mass;
                decompose_vector(               linear_damping,          __braking_vector);
                decompose_vector(        -local_gravity_vector,         local_gravity_inv);
                decompose_vector(-local_linear_velocity_vector, local_linear_velocity_inv);
                Array.Copy(control_vector, manual_control_vector, 6);

                for (int dir_index = 0, opposite_dir = 3; dir_index < 3; ++dir_index, ++opposite_dir)
                {
                    enable_linear_integral[dir_index] = enable_linear_integral[opposite_dir] = !DEBUG_DISABLE_ALT_HOLD 
                        && (    local_gravity_inv[dir_index] >        0.0f  ||     local_gravity_inv[opposite_dir] > 0.0f)
                        &&  manual_control_vector[dir_index] < MIN_THROTTLE && manual_control_vector[opposite_dir] < MIN_THROTTLE
                        && (     actual_max_force[dir_index] >        1.0f  ||      actual_max_force[opposite_dir] > 1.0f);

                    set_brake(   dir_index, opposite_dir);
                    set_brake(opposite_dir,    dir_index);
                    if (control_vector[dir_index] > control_vector[opposite_dir])
                    {
                        control_vector[   dir_index] -= control_vector[opposite_dir];
                        control_vector[opposite_dir]  = 0.0f;
                    }
                    else
                    { 
                        control_vector[opposite_dir] -= control_vector[dir_index];
                        control_vector[   dir_index]  = 0.0f;
                    }

                    float gravity_ratio          = (gravity_magnitude < 0.01f) ? 1.0f : ((local_gravity_inv[dir_index] + local_gravity_inv[opposite_dir]) / gravity_magnitude),
                          axis_speed             = local_linear_velocity_inv[dir_index] + local_linear_velocity_inv[opposite_dir],
                          linear_integral_change = LINEAR_INTEGRAL_CONSTANT * (local_linear_velocity_inv[dir_index] - local_linear_velocity_inv[opposite_dir]);
                    if (linear_integral_change > 0.0f)
                        set_linear_integral(   dir_index, opposite_dir,  linear_integral_change, gravity_ratio, axis_speed);
                    else if (linear_integral_change < 0.0f)
                        set_linear_integral(opposite_dir,    dir_index, -linear_integral_change, gravity_ratio, axis_speed);

                    if (touchdown_mode_on)
                    {
                        linear_integral[dir_index] -= LINEAR_INTEGRAL_CONSTANT * DESCENDING_SPEED * gravity_ratio;
                        if (linear_integral[dir_index] < 0.0f)
                            linear_integral[dir_index] = 0.0f;
                        linear_integral[opposite_dir] -= LINEAR_INTEGRAL_CONSTANT * DESCENDING_SPEED * gravity_ratio;
                        if (linear_integral[opposite_dir] < 0.0f)
                            linear_integral[opposite_dir] = 0.0f;
                    }
                }
                normalise_control();
            }
        }

        private void set_linear_integral(int dir_index, int opposite_dir, float linear_integral_change, float gravity_ratio, float axis_speed)
        {
            float[] linear_integral = _linear_integral;

            if (linear_integral[opposite_dir] <= 0.0f)
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

                    linear_integral[dir_index] += linear_integral_change;
                }
                linear_integral[opposite_dir] = 0.0f;
            }
            else
            {
                linear_integral[opposite_dir] -= linear_integral_change;
                if (linear_integral[opposite_dir] >= 0.0f)
                    linear_integral[dir_index] = 0.0f;
                else
                {
                    float max_increase = LINEAR_INTEGRAL_CONSTANT + linear_integral[opposite_dir];

                    if (max_increase < 0.0f)
                        max_increase = 0.0f;
                    linear_integral[dir_index] = _enable_linear_integral[dir_index] ? (-linear_integral[opposite_dir]) : 0.0f;
                    if (linear_integral[dir_index] > max_increase && !secondary_ECU)
                        linear_integral[dir_index] = max_increase;
                    linear_integral[opposite_dir] = 0.0f;
                }
            }
        }

        private void set_brake(int dir_index, int opposite_dir)
        {
            float[] control_vector = __control_vector, total_force = _total_force;
            bool[]  enable_linear_integral = _enable_linear_integral;

            if (__manual_control_vector[opposite_dir] < MIN_THROTTLE && total_force[dir_index] >= 1.0f)
            {
                float braking_force = __braking_vector[dir_index], mass, average_grid_mass = this.average_grid_mass;

                mass = (average_grid_mass >= _grid_mass) ? average_grid_mass : _grid_mass;
                if (enable_linear_integral[dir_index])
                    braking_force += mass * _linear_integral[dir_index];
                if (total_force[opposite_dir] < 1.0f && enable_linear_integral[opposite_dir])
                    braking_force -= mass * _linear_integral[opposite_dir];
                control_vector[dir_index] += braking_force / (total_force[dir_index]);
                if (control_vector[dir_index] < 0.0f)
                    control_vector[dir_index] = 0.0f;
                else if (control_vector[dir_index] >= 1.0f)
                    enable_linear_integral[dir_index] = enable_linear_integral[opposite_dir] = false;
            }
        }

        private void normalise_control()
        {
            float[] control_vector = __control_vector;

            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                if (control_vector[dir_index] > 1.0f)
                    control_vector[dir_index] = 1.0f;
            }
        }

        private Vector3 get_reference_vector(thruster_info thruster, Vector3 reference_point, Vector3 thruster_dir)
        {
            Vector3 offset    = thruster.grid_centre_pos - reference_point;
            Vector3 reference = Vector3.Cross(thruster_dir, offset);
            float   length    = reference.Length();

            if (length > 1.0f)
                return reference / length;
            return reference;
        }

        private void adjust_thrust_for_steering(int cur_dir, int opposite_dir, Vector3 desired_angular_velocity)
        {
            const float DAMPING_CONSTANT = 0.5f, MIN_LINEAR_OPPOSITION = 0.05f, MAX_LINEAR_OPPOSITION = 1.0f, MAX_CONTROL = 0.2f, MIN_COT_SETTING = 0.1f;
            const float ALIGMENT_DEAD_ZONE = 0.5f, ANGULAR_VELOCITY_DEAD_ZONE = 0.01f;

            if (_actual_max_force[cur_dir] <= 1.0f)
            {
                __new_static_moment[cur_dir] = Vector3.Zero;
                return;
            }

            float average_grid_mass = this.average_grid_mass;

            Vector3 angular_velocity_diff = desired_angular_velocity - _local_angular_velocity, total_static_moment = Vector3.Zero;
            float   max_linear_opposition, damping = DAMPING_CONSTANT * _surface_radii[cur_dir] * ((average_grid_mass >= _grid_mass) ? average_grid_mass : _grid_mass) / _actual_max_force[cur_dir],
                    current_limit = __thrust_limits[cur_dir], total_force = 0.0f, adjusted_damping,
                    linear_opposition = Math.Min(MAX_CONTROL, Math.Max(__control_vector[opposite_dir], _thrust_override_vector[opposite_dir])) / MAX_CONTROL,
                    CoT_setting, torque_alignment;
            bool    enforce_thrust_limit = !_current_mode_is_CoT && linear_opposition >= MIN_LINEAR_OPPOSITION, THR_mode_used;

            max_linear_opposition = MAX_LINEAR_OPPOSITION * (1.0f - linear_opposition) + MIN_LINEAR_OPPOSITION * linear_opposition;
            if (_actual_max_force[opposite_dir] < _actual_max_force[cur_dir])
                max_linear_opposition *= _actual_max_force[opposite_dir] / _actual_max_force[cur_dir];
            if (max_linear_opposition > MAX_LINEAR_OPPOSITION)
                max_linear_opposition = MAX_LINEAR_OPPOSITION;

            float angular_velocity_delta = angular_velocity_diff.Length();
            damping                     *= angular_velocity_delta;
            angular_velocity_diff       /= (angular_velocity_delta > ANGULAR_VELOCITY_DEAD_ZONE) ? angular_velocity_delta : ANGULAR_VELOCITY_DEAD_ZONE;
            foreach (thruster_info cur_thruster_info in _steering_thrusters[cur_dir])
            {
                cur_thruster_info.apply_limit = Vector3.Dot(angular_velocity_diff, cur_thruster_info.torque_factor) < 0.0f;

                torque_alignment = Vector3.Dot(angular_velocity_diff, cur_thruster_info.reference_vector);
                if (torque_alignment > 0.0f)
                {
                    adjusted_damping = damping;
                    if (torque_alignment < ALIGMENT_DEAD_ZONE)
                        adjusted_damping *= torque_alignment / ALIGMENT_DEAD_ZONE;
                    cur_thruster_info.current_setting += adjusted_damping;
                    if (cur_thruster_info.steering_on && !cur_thruster_info.is_RCS)
                        cur_thruster_info.current_setting += max_linear_opposition * (1.0f - __thrust_limits[cur_dir]);
                    cur_thruster_info.throttle_up = true;
                    if (enforce_thrust_limit && cur_thruster_info.steering_on && !cur_thruster_info.is_RCS)
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
                else if (torque_alignment < 0.0f)
                {
                    THR_mode_used = cur_thruster_info.steering_on && !cur_thruster_info.is_RCS;
                    adjusted_damping = damping;
                    if (torque_alignment > ALIGMENT_DEAD_ZONE)
                        adjusted_damping *= torque_alignment / (-ALIGMENT_DEAD_ZONE);
                    cur_thruster_info.current_setting -= damping;
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

                foreach (thruster_info cur_thruster_info in _controlled_thrusters[cur_dir])
                {
                    if (cur_thruster_info.throttle_up)
                        continue;

                    torque_alignment = Vector3.Dot(angular_velocity_diff, get_reference_vector(cur_thruster_info, effective_CoT, thruster_dir));
                    if (torque_alignment > 0.0f)
                    {
                        adjusted_damping = damping;
                        if (torque_alignment < ALIGMENT_DEAD_ZONE)
                            adjusted_damping *= torque_alignment / ALIGMENT_DEAD_ZONE;
                        cur_thruster_info.current_setting += adjusted_damping;
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

            HashSet<thruster_info>[] controlled_thrusters = _controlled_thrusters;
            float[] control_vector = __control_vector, thrust_override_vector = _thrust_override_vector;

            float linear_force = 0.0f, requested_force = 0.0f, max_setting = 0.0f, max_control = 0.0f, dir_force1, dir_force2;
            bool  zero_thrust_reduction = true;

            Action<int> eliminate_direct_opposition = delegate (int dir_index)
            {
                thruster_info first_opposite_trhuster, cur_opposite_thruster;
                float         cur_thruster_force, opposite_thruster_force;
                bool          active_control_off, opposite_control_off;

                foreach (thruster_info cur_thruster_info in _controlled_thrusters[dir_index])
                {
                    first_opposite_trhuster = cur_thruster_info.opposing_thruster;
                    if (first_opposite_trhuster == null || cur_thruster_info.actual_max_force < 1.0f)
                        continue;

                    cur_thruster_force    = cur_thruster_info.current_setting * cur_thruster_info.actual_max_force;
                    active_control_off    = cur_thruster_info.enable_limit && !cur_thruster_info.steering_on;
                    cur_opposite_thruster = first_opposite_trhuster;
                    do
                    {
                        if (cur_opposite_thruster.actual_max_force >= 1.0f)
                        {
                            opposite_thruster_force = cur_opposite_thruster.current_setting * cur_opposite_thruster.actual_max_force;
                            opposite_control_off    = cur_opposite_thruster.enable_limit && !cur_opposite_thruster.steering_on;
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
            else
            {
                for (int dir_index = 0; dir_index < 3; ++dir_index)
                    eliminate_direct_opposition(dir_index);
            }

            for (int dir_index = 0, opposite_dir = 3; dir_index < 3; ++dir_index, ++opposite_dir)
            { 
                foreach (thruster_info cur_thruster_info in controlled_thrusters[dir_index])
                {
                    if (max_setting < cur_thruster_info.current_setting)
                        max_setting = cur_thruster_info.current_setting;
                }
                foreach (thruster_info cur_thruster_info in controlled_thrusters[opposite_dir])
                {
                    if (max_setting < cur_thruster_info.current_setting)
                        max_setting = cur_thruster_info.current_setting;
                }
                if (max_control < control_vector[   dir_index])
                    max_control = control_vector[   dir_index];
                if (max_control < control_vector[opposite_dir])
                    max_control = control_vector[opposite_dir];
                if (max_control < thrust_override_vector[dir_index])
                    max_control = thrust_override_vector[dir_index];
                if (max_control < thrust_override_vector[opposite_dir])
                    max_control = thrust_override_vector[opposite_dir];
            }

            if (max_setting > 0.0f)
            {
                float max_normalisation_multiplier = 1.0f / max_setting;

                if (max_normalisation_multiplier > MAX_NORMALISATION)
                    max_normalisation_multiplier = MAX_NORMALISATION;
                max_normalisation_multiplier = 1.0f + max_control * (max_normalisation_multiplier - 1.0f);

                Action<int> normalise_direction = delegate (int dir_index)
                {
                    float[] actual_force = __actual_force, non_THR_force = __non_THR_force;
                    
                    actual_force[dir_index] = non_THR_force[dir_index] = 0.0f;
                    foreach (thruster_info cur_thruster_info in controlled_thrusters[dir_index])
                    {
                        cur_thruster_info.current_setting *= max_normalisation_multiplier;
                        if (!cur_thruster_info.steering_on || cur_thruster_info.is_RCS)
                            non_THR_force[dir_index] += cur_thruster_info.current_setting * cur_thruster_info.actual_max_force;
                        else
                            actual_force [dir_index] += cur_thruster_info.current_setting * cur_thruster_info.actual_max_force;
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
                float[] actual_force = __actual_force, non_THR_force = __non_THR_force;
                
                int   opposite_dir     = dir_index + 3;
                float force1           = actual_force[   dir_index] + non_THR_force[   dir_index];
                float force2           = actual_force[opposite_dir] + non_THR_force[opposite_dir];
                float new_force_ratio1 = 1.0f, new_force_ratio2 = 1.0f;

                if (actual_force[dir_index] >= 1.0f && force1 - __requested_force[dir_index] > force2)
                {
                    new_force_ratio1 = (force2 + __requested_force[dir_index] - non_THR_force[dir_index]) / actual_force[dir_index];
                    if (new_force_ratio1 < 0.0f)
                        new_force_ratio1 = 0.0f;
                    else if (new_force_ratio1 > 1.0f)
                        new_force_ratio1 = 1.0f;

                    foreach (thruster_info cur_thruster_info in controlled_thrusters[dir_index])
                    {
                        if (cur_thruster_info.steering_on && !cur_thruster_info.is_RCS)
                            cur_thruster_info.current_setting *= new_force_ratio1;
                    }
                }
                actual_force[dir_index] *= new_force_ratio1;
                force1 = actual_force[dir_index] + non_THR_force[dir_index];

                if (actual_force[opposite_dir] >= 1.0f && force2 - __requested_force[opposite_dir] > force1)
                {
                    new_force_ratio2 = (force1 + __requested_force[opposite_dir] - non_THR_force[opposite_dir]) / actual_force[opposite_dir];
                    if (new_force_ratio2 < 0.0f)
                        new_force_ratio2 = 0.0f;
                    else if (new_force_ratio2 > 1.0f)
                        new_force_ratio2 = 1.0f;

                    foreach (thruster_info cur_thruster_info in controlled_thrusters[opposite_dir])
                    {
                        if (cur_thruster_info.steering_on && !cur_thruster_info.is_RCS)
                            cur_thruster_info.current_setting *= new_force_ratio2;
                    }
                }
                actual_force[opposite_dir] *= new_force_ratio2;
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
                if (control_vector[dir_index] >= 0.25f || control_vector[opposite_dir] >= 0.25f)
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

            if (sync_helper.running_on_server)
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
            const float ANGULAR_INTEGRAL_COEFF = 0.1f, ANGULAR_DERIVATIVE_COEFF = 0.1f, MAX_TRIM = 5.0f, THRUST_CUTOFF_TRIM = 4.0f, CHECKPOINT_FADE = 0.75f, 
                ANGULAR_ACCELERATION_SMOOTHING = 0.5f;

            bool    rotational_damping_enabled = rotational_damping_on;
            float   trim_change, thrust_limit_pitch, thrust_limit_yaw, thrust_limit_roll;
            Vector3 local_angular_velocity      = rotational_damping_enabled ? _local_angular_velocity : Vector3.Zero;
            Vector3 local_angular_acceleration  = (local_angular_velocity - _prev_angular_velocity) * MyEngineConstants.UPDATE_STEPS_PER_SECOND;
            _prev_angular_velocity = local_angular_velocity;

            Vector3 steering_input_vector = _manual_rotation;
            steering_input_vector.X /= 0.2f;
            steering_input_vector.Y /= 0.45f;
            steering_input_vector.Z /= 0.45f;
            float[] steering_input              =             __steering_input, angular_velocity        = __angular_velocity, 
                    angular_velocity_diff       =      __angular_velocity_diff, target_angular_velocity = __target_angular_velocity, 
                    angular_velocity_checkpoint = _angular_velocity_checkpoint, last_angular_velocity   =  _last_angular_velocity, 
                    current_trim                =                _current_trim, aux_trim                = _aux_trim,
                    angular_acceleration        =       __angular_acceleration, smoothed_acceleration   = _smoothed_acceleration, 
                    gyro_override               =              __gyro_override,
                    residual_torque             =            __residual_torque, thrust_limits           = __thrust_limits;
            bool[] active_control_on       = _steering_enabled;
            bool   is_gyro_override_active = _is_gyro_override_active;
            decompose_vector(recompose_vector(target_angular_velocity) - local_angular_velocity, angular_velocity_diff);
            decompose_vector(     steering_input_vector,       steering_input);
            decompose_vector(    local_angular_velocity,     angular_velocity);
            decompose_vector(local_angular_acceleration, angular_acceleration);
            Vector3 residual_torque_vector = _torque / _spherical_moment_of_inertia;
            if (residual_torque_vector.LengthSquared() > 1.0f)
                residual_torque_vector.Normalize();
            residual_torque_vector *= 0.01f;
            decompose_vector(residual_torque_vector, residual_torque);
            decompose_vector(is_gyro_override_active ? _gyro_override : Vector3.Zero, gyro_override);

            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                target_angular_velocity[dir_index] = steering_input[dir_index] * 2.0f;
                if (steering_input[dir_index] > 0.01f)
                    target_angular_velocity[dir_index] += angular_velocity[dir_index];
                if (!rotational_damping_enabled)
                    target_angular_velocity[dir_index] += angular_velocity[dir_index];
                else if (is_gyro_override_active)
                    target_angular_velocity[dir_index] += gyro_override [dir_index];
                else if (steering_input[dir_index] > 0.01f)
                    target_angular_velocity[dir_index] += angular_velocity[dir_index];
            }
            decompose_vector(recompose_vector(target_angular_velocity) - local_angular_velocity, angular_velocity_diff);

            int opposite_dir;
            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                if (!rotational_damping_enabled || !active_control_on[dir_index])
                {
                    last_angular_velocity[dir_index] = -1.0f;
                    current_trim[dir_index] = aux_trim[dir_index] = angular_velocity_checkpoint[dir_index] = smoothed_acceleration[dir_index] = 0.0f;
                }

                opposite_dir = (dir_index + 3) % 6;
                if (is_gyro_override_active && gyro_override[dir_index] > 0.01f && angular_velocity_diff[dir_index] < 0.01f)
                    angular_velocity_diff[dir_index] = 0.0f;
                if (target_angular_velocity[dir_index] != last_angular_velocity[dir_index])
                {
                    if (angular_velocity_checkpoint[dir_index] < angular_velocity_diff[dir_index] * CHECKPOINT_FADE)
                        angular_velocity_checkpoint[dir_index] = angular_velocity_diff[dir_index] * CHECKPOINT_FADE;
                    if (angular_velocity_checkpoint[opposite_dir] < angular_velocity_diff[opposite_dir] * CHECKPOINT_FADE)
                        angular_velocity_checkpoint[opposite_dir] = angular_velocity_diff[opposite_dir] * CHECKPOINT_FADE;
                    last_angular_velocity[dir_index] = target_angular_velocity[dir_index];
                }

                trim_change = ANGULAR_INTEGRAL_COEFF * angular_velocity_diff[dir_index];
                if (angular_velocity_diff[dir_index] < 0.0001f && angular_velocity_diff[opposite_dir] < 0.0001f)
                    trim_change += ANGULAR_DERIVATIVE_COEFF * residual_torque[opposite_dir];
                change_trim(    aux_trim, dir_index, opposite_dir, trim_change);
                change_trim(current_trim, dir_index, opposite_dir, trim_change);
                if (aux_trim[dir_index] > 0.0 && current_trim[opposite_dir] > 0.0f)
                    aux_trim[dir_index] = 0.0f;
                else if (aux_trim[opposite_dir] > 0.0 && current_trim[dir_index] > 0.0f)
                    aux_trim[opposite_dir] = 0.0f;
            }

            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                current_trim[dir_index] *= _trim_fadeout;
                aux_trim    [dir_index] *= _trim_fadeout;
                if (current_trim[dir_index] < aux_trim[dir_index])
                    current_trim[dir_index] = aux_trim[dir_index];
                if (angular_velocity_diff[dir_index] < angular_velocity_checkpoint[dir_index])
                {
                    angular_velocity_checkpoint[dir_index] *= CHECKPOINT_FADE;
                    if (angular_velocity_checkpoint[dir_index] < 0.01f)
                        angular_velocity_checkpoint[dir_index] = 0.0f;
                    current_trim[dir_index] = 0.5f * (current_trim[dir_index] + aux_trim[dir_index]);
                    aux_trim    [dir_index] = 0.0f;
                }
                smoothed_acceleration[dir_index] = smoothed_acceleration[dir_index] * ANGULAR_ACCELERATION_SMOOTHING + angular_acceleration[dir_index] * (1.0f - ANGULAR_ACCELERATION_SMOOTHING);
            }

            Vector3 trim_vector = recompose_vector(current_trim);
            thrust_limit_pitch = 1.0f - (1.0f / THRUST_CUTOFF_TRIM) * Math.Abs(trim_vector.X);
            thrust_limit_yaw   = 1.0f - (1.0f / THRUST_CUTOFF_TRIM) * Math.Abs(trim_vector.Y);
            thrust_limit_roll  = 1.0f - (1.0f / THRUST_CUTOFF_TRIM) * Math.Abs(trim_vector.Z);
            if (thrust_limit_pitch < 0.0f)
                thrust_limit_pitch = 0.0f;
            if (thrust_limit_yaw < 0.0f)
                thrust_limit_yaw = 0.0f;
            if (thrust_limit_roll < 0.0f)
                thrust_limit_roll = 0.0f;
            thrust_limits[(int) thrust_dir.fore     ] = thrust_limits[(int) thrust_dir.aft    ] = (thrust_limit_pitch < thrust_limit_yaw ) ? thrust_limit_pitch : thrust_limit_yaw;
            thrust_limits[(int) thrust_dir.starboard] = thrust_limits[(int) thrust_dir.port   ] = (thrust_limit_yaw   < thrust_limit_roll) ? thrust_limit_yaw   : thrust_limit_roll;
            thrust_limits[(int) thrust_dir.dorsal   ] = thrust_limits[(int) thrust_dir.ventral] = (thrust_limit_pitch < thrust_limit_roll) ? thrust_limit_pitch : thrust_limit_roll;

            desired_angular_velocity = recompose_vector(angular_velocity_diff) + recompose_vector(current_trim) - ANGULAR_DERIVATIVE_COEFF * recompose_vector(smoothed_acceleration);
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
                _linear_control  = Vector3.Zero;
                apply_thrust_settings(reset_all_thrusters: true);
                return;
            }

            _local_angular_velocity = Vector3.Transform(world_angular_velocity, inverse_world_rotation);
            Vector3 desired_angular_velocity;
            initialise_linear_controls(local_linear_velocity * _dampers_axes_enabled, local_gravity * _dampers_axes_enabled);
            adjust_trim_setting(out desired_angular_velocity);
            _new_mode_is_CoT = CoT_mode_on;

            Action<int> set_up_thrusters = delegate (int dir_index)
            {
                float   control         = __control_vector[dir_index], min_collective_throttle = 1.0f;
                float[] requested_force = __requested_force;

                requested_force[dir_index] = 0.0f;
                foreach (thruster_info cur_thruster_info in _collective_thrusters[dir_index])
                {
                    if (min_collective_throttle > cur_thruster_info.manual_throttle)
                        min_collective_throttle = cur_thruster_info.manual_throttle;
                }
                foreach (thruster_info cur_thruster_info in _controlled_thrusters[dir_index])
                {
                    if (cur_thruster_info.manual_throttle < 0.01f)
                        cur_thruster_info.current_setting = cur_thruster_info.disable_linear_input ? 0.0f : control;
                    else if (cur_thruster_info.enable_limit && !cur_thruster_info.steering_on && _is_solution_good[dir_index])
                        cur_thruster_info.current_setting = min_collective_throttle;
                    else
                        cur_thruster_info.current_setting = cur_thruster_info.manual_throttle;
                    requested_force[dir_index] += cur_thruster_info.current_setting * cur_thruster_info.actual_max_force;
                    if (cur_thruster_info.enable_limit && (_is_solution_good[dir_index] || cur_thruster_info.disable_linear_input))
                        cur_thruster_info.current_setting *= cur_thruster_info.thrust_limit;
                }
                adjust_thrust_for_steering(dir_index, (dir_index < 3) ? (dir_index + 3) : (dir_index - 3), desired_angular_velocity);
            };

            lock (_changed_thrusters)
            {
                if (!session_handler.SINGLE_THREADED_EXEC)
                    MyAPIGateway.Parallel.For(0, 6, set_up_thrusters);
                else
                {
                    for (int dir_index = 0; dir_index < 6; ++dir_index)
                        set_up_thrusters(dir_index);
                }

                float[]    new_total_force   = __new_total_force;
                Vector3[]  new_static_moment = __new_static_moment;
                Vector3?[] active_CoT        = _active_CoT;
                for (int dir_index = 0, opposite_dir = 3; dir_index < 3; ++dir_index, ++opposite_dir)
                {
                    float total_force      = new_total_force[dir_index] + new_total_force[opposite_dir];
                    active_CoT[dir_index] = active_CoT[opposite_dir] = (total_force < 1.0f) ? null : ((Vector3?) ((new_static_moment[dir_index] + new_static_moment[opposite_dir]) / total_force));

                    if (__requested_force[dir_index] > __requested_force[opposite_dir])
                    {
                        __requested_force[dir_index] -= __requested_force[opposite_dir];
                        __requested_force[opposite_dir]  = 0.0f;
                    }
                    else
                    {
                        __requested_force[opposite_dir] -= __requested_force[dir_index];
                        __requested_force[dir_index] = 0.0f;
                    }
                }
                normalise_thrust();
                apply_thrust_settings(reset_all_thrusters: false);
            }
        }

        private void update_reference_vectors_for_CoM_mode()
        {
            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                HashSet<thruster_info> cur_direction = _controlled_thrusters  [dir_index];
                Vector3                thruster_dir  = _thrust_forward_vectors[dir_index];

                foreach (thruster_info cur_thruster_info in cur_direction)
                    cur_thruster_info.reference_vector = get_reference_vector(cur_thruster_info, _grid_CoM_location, thruster_dir);
            }
        }

        private void update_reference_vectors_for_CoT_mode()
        {
            Vector3                  total_static_moment, CoT_location, thruster_dir;
            HashSet<thruster_info>   cur_direction;
            HashSet<thruster_info>[] controlled_thrusters = _controlled_thrusters;

            for (int dir_index = 0, opposite_dir = 3; dir_index < 3; ++dir_index, ++opposite_dir)
            {
                if (_actual_max_force[dir_index] < 1.0f || _actual_max_force[opposite_dir] < 1.0f)
                {
                    Vector3D grid_CoM_location = _grid_CoM_location;
                    
                    cur_direction = controlled_thrusters   [dir_index];
                    thruster_dir  = _thrust_forward_vectors[dir_index];
                    foreach (thruster_info cur_thruster_info in cur_direction)
                        cur_thruster_info.reference_vector = get_reference_vector(cur_thruster_info, grid_CoM_location, thruster_dir);
                    cur_direction = controlled_thrusters   [opposite_dir];
                    thruster_dir  = _thrust_forward_vectors[opposite_dir];
                    foreach (thruster_info cur_thruster_info in cur_direction)
                        cur_thruster_info.reference_vector = get_reference_vector(cur_thruster_info, grid_CoM_location, thruster_dir);
                }
                else
                {
                    total_static_moment = Vector3.Zero;
                    cur_direction = controlled_thrusters[dir_index];
                    foreach (thruster_info cur_thruster_info in cur_direction)
                        total_static_moment += cur_thruster_info.actual_static_moment;
                    cur_direction = controlled_thrusters[opposite_dir];
                    foreach (thruster_info cur_thruster_info in cur_direction)
                        total_static_moment += cur_thruster_info.actual_static_moment;
                    CoT_location = total_static_moment / (_actual_max_force[dir_index] + _actual_max_force[opposite_dir]);

                    cur_direction = controlled_thrusters  [dir_index];
                    thruster_dir  = _thrust_forward_vectors[dir_index];
                    foreach (thruster_info cur_thruster_info in cur_direction)
                        cur_thruster_info.reference_vector = get_reference_vector(cur_thruster_info, CoT_location, thruster_dir);
                    cur_direction = controlled_thrusters  [opposite_dir];
                    thruster_dir  = _thrust_forward_vectors[opposite_dir];
                    foreach (thruster_info cur_thruster_info in cur_direction)
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

        private void find_tandem_and_opposite_thrusters(thruster_info examined_thruster_info)
        {
            Dictionary<Vector3I, List<thruster_info>> cur_direction =       _tandem_thrusters[(int) examined_thruster_info.nozzle_direction];
            Vector3I                                  filter_vector = _thrust_forward_vectors[(int) examined_thruster_info.nozzle_direction], filtered_location;

            filter_vector     = Vector3I.One - filter_vector / (filter_vector.X + filter_vector.Y + filter_vector.Z);
            filtered_location = examined_thruster_info.grid_cells * filter_vector;
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

        private void remove_thruster_from_lists(thruster_info removed_thruster_info)
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
            filtered_location = removed_thruster_info.grid_cells * filter_vector;
            List<thruster_info> tandem_thrusters = cur_direction[filtered_location];
            tandem_thrusters.Remove(removed_thruster_info);
            if (tandem_thrusters.Count == 0)
                cur_direction.Remove(filtered_location);
        }

        private void check_changed_thrusters()
        {
            bool                   is_controlled, is_steering, is_limited, currently_uncontrolled, currently_steering, thrusters_moved = false;
            int                    dir_index;
            HashSet<thruster_info> steering_thrusters, collective_thrusters, uncontrolled_thrusters = _uncontrolled_thrusters, working_set = _changed_thrusters;
            bool[]                 is_solution_good = _is_solution_good;

            if (_changed_thrusters.Count == 0)
                return;
           
            lock (_changed_thrusters)
            {
                if (_thruster_check_in_progress)
                    return;
                _thruster_check_in_progress = true;
                _changed_thrusters = (_changed_thrusters == _changed_thrusters1) ? _changed_thrusters2 : _changed_thrusters1;
            }
            foreach (thruster_info cur_thruster_info in working_set)
            {
                dir_index            = (int) cur_thruster_info.nozzle_direction;
                steering_thrusters   =   _steering_thrusters[dir_index];
                collective_thrusters = _collective_thrusters[dir_index];
                is_steering          = cur_thruster_info.steering_on || cur_thruster_info.enable_rotation;
                is_limited           = cur_thruster_info.enable_limit;
                is_controlled        = (is_steering || is_limited) && cur_thruster_info.operational && cur_thruster_info.actual_max_force > 1.0f;
                is_steering         &= is_controlled;

                currently_uncontrolled = uncontrolled_thrusters.Contains(cur_thruster_info);
                currently_steering     =     steering_thrusters.Contains(cur_thruster_info);
                if (is_controlled && currently_uncontrolled)
                {
                    enable_control(cur_thruster_info);
                    thrusters_moved = true;
                }
                else if (!is_controlled && !currently_uncontrolled)
                {
                    disable_control(cur_thruster_info);
                    thrusters_moved = true;
                }
                if (is_steering && !currently_steering)
                    steering_thrusters.Add(cur_thruster_info);
                else if (!is_steering && currently_steering)
                    steering_thrusters.Remove(cur_thruster_info);

                if (is_limited)
                {
                    if (!is_steering && is_solution_good[dir_index])
                    {
                        if (!cur_thruster_info.collective_control_on)
                        {
                            collective_thrusters.Add(cur_thruster_info);
                            cur_thruster_info.collective_control_on = true;
                        }
                    }
                    else if (cur_thruster_info.collective_control_on)
                    {
                        collective_thrusters.Remove(cur_thruster_info);
                        cur_thruster_info.collective_control_on = false;
                    }
                }
            }
            working_set.Clear();

            if (thrusters_moved)
            {
                _air_density = float.MinValue;
                if (_current_mode_is_CoT)
                    update_reference_vectors_for_CoT_mode();
                else
                    update_reference_vectors_for_CoM_mode();
                /*
                log_ECU_action("check_changed_thrusters", string.Format("{0}/{1}/{2}/{3}/{4}/{5} kN",
                    _max_force[(int) thrust_dir.fore     ] / 1000.0f,
                    _max_force[(int) thrust_dir.aft      ] / 1000.0f,
                    _max_force[(int) thrust_dir.starboard] / 1000.0f,
                    _max_force[(int) thrust_dir.port     ] / 1000.0f,
                    _max_force[(int) thrust_dir.dorsal   ] / 1000.0f,
                    _max_force[(int) thrust_dir.ventral  ] / 1000.0f));
                */
            }

            bool[] steering_enabled = _steering_enabled;
            Array.Clear(steering_enabled, 0, 6);
            for (dir_index = 0; dir_index < 3; ++dir_index)
            {
                is_steering = _steering_thrusters[dir_index].Count > 0 || _steering_thrusters[dir_index + 3].Count > 0;
                switch ((thrust_dir) dir_index)
                {
                    case thrust_dir.fore:
                    case thrust_dir.aft:
                        steering_enabled[(int) thrust_dir.port     ] |= is_steering;
                        steering_enabled[(int) thrust_dir.starboard] |= is_steering;
                        steering_enabled[(int) thrust_dir.dorsal   ] |= is_steering;
                        steering_enabled[(int) thrust_dir.ventral  ] |= is_steering;
                        break;

                    case thrust_dir.port:
                    case thrust_dir.starboard:
                        steering_enabled[(int) thrust_dir.dorsal ] |= is_steering;
                        steering_enabled[(int) thrust_dir.ventral] |= is_steering;
                        steering_enabled[(int) thrust_dir.fore   ] |= is_steering;
                        steering_enabled[(int) thrust_dir.aft    ] |= is_steering;
                        break;

                    case thrust_dir.dorsal:
                    case thrust_dir.ventral:
                        steering_enabled[(int) thrust_dir.port     ] |= is_steering;
                        steering_enabled[(int) thrust_dir.starboard] |= is_steering;
                        steering_enabled[(int) thrust_dir.fore     ] |= is_steering;
                        steering_enabled[(int) thrust_dir.aft      ] |= is_steering;
                        break;
                }
            }
            _thruster_check_in_progress = false;
        }

        private void reset_overrides()
        {
            foreach (thruster_info cur_thruster_info in _thrusters_reset_override)
                cur_thruster_info.host_thruster.ThrustOverride = 0.0f;
            _thrusters_reset_override.Clear();
        }

        private void enable_control(thruster_info cur_thruster_info)
        {
            int dir_index = (int) cur_thruster_info.nozzle_direction;

            lock (_changed_thrusters)
            {
                _controlled_thrusters[dir_index].Add(cur_thruster_info);
                _uncontrolled_thrusters.Remove(cur_thruster_info);
                _max_force[dir_index] += cur_thruster_info.max_force;
                find_tandem_and_opposite_thrusters(cur_thruster_info);
                cur_thruster_info.thrust_limit    = 1.0f;
                _calibration_scheduled[dir_index] = true;
                _thrusters_reset_override.Add(cur_thruster_info);
            }
        }

        private void disable_control(thruster_info cur_thruster_info)
        {
            int dir_index = (int) cur_thruster_info.nozzle_direction;

            lock (_changed_thrusters)
            {
                cur_thruster_info.thrust_limit = 1.0f;
                _max_force[dir_index]         -= cur_thruster_info.max_force;
                _uncontrolled_thrusters.Add(cur_thruster_info);
                _controlled_thrusters[dir_index].Remove(cur_thruster_info);
                _calibration_scheduled[dir_index] = true;
                _thrusters_reset_override.Add(cur_thruster_info);
                remove_thruster_from_lists(cur_thruster_info);
            }
        }

        private float refresh_real_max_forces_for_single_direction(HashSet<thruster_info> thrusters/*, bool atmosphere_present, float air_density*/)
        {
            float actual_max_force = 0.0f;

            foreach (thruster_info cur_thruster_info in thrusters)
            {
                cur_thruster_info.actual_max_force     = cur_thruster_info.host_thruster.MaxEffectiveThrust;
                cur_thruster_info.actual_static_moment = cur_thruster_info.static_moment * (cur_thruster_info.actual_max_force / cur_thruster_info.max_force);
                actual_max_force                      += cur_thruster_info.actual_max_force;

                if (cur_thruster_info.actual_max_force < 1.0f)
                {
                    lock (_changed_thrusters)
                        _changed_thrusters.Add(cur_thruster_info);
                }
            }
            return actual_max_force;
        }

        private void refresh_real_max_forces_for_uncontrolled_thrusters()
        {
            float[] uncontrolled_max_force = _uncontrolled_max_force;
            Array.Clear(uncontrolled_max_force, 0, 6);
            foreach (thruster_info cur_thruster_info in _uncontrolled_thrusters)
            {
                cur_thruster_info.actual_max_force     = cur_thruster_info.host_thruster.MaxEffectiveThrust;
                cur_thruster_info.actual_static_moment = cur_thruster_info.static_moment * (cur_thruster_info.actual_max_force / cur_thruster_info.max_force);
                uncontrolled_max_force[(int) cur_thruster_info.nozzle_direction] += cur_thruster_info.actual_max_force;

                if ((cur_thruster_info.steering_on || cur_thruster_info.enable_limit) && cur_thruster_info.operational && cur_thruster_info.actual_max_force > 1.0f)
                {
                    lock (_changed_thrusters)
                        _changed_thrusters.Add(cur_thruster_info);
                }
            }
        }

        private void refresh_real_max_forces()
        {
            BoundingBoxD grid_bounding_box = _grid.PositionComp.WorldAABB;
            MyPlanet     closest_planetoid = MyGamePruningStructure.GetClosestPlanet(ref grid_bounding_box);
            float        new_air_density   = (closest_planetoid == null) ? 0.0f : closest_planetoid.GetAirDensity(grid_bounding_box.Center);

            float prev_air_density = _air_density;
            if (Math.Abs(new_air_density - prev_air_density) < 0.005f && (new_air_density == 0.0) == (prev_air_density == 0.0))
                return;
            
            bool[]  calibration_scheduled = _calibration_scheduled;
            float[] actual_max_force      = _actual_max_force, total_force = _total_force, uncontrolled_max_force = _uncontrolled_max_force;
            HashSet<thruster_info>[] controlled_thrusters = _controlled_thrusters;
            
            refresh_real_max_forces_for_uncontrolled_thrusters();
            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                actual_max_force[dir_index] = refresh_real_max_forces_for_single_direction(controlled_thrusters[dir_index]);
                total_force     [dir_index] = actual_max_force[dir_index] + uncontrolled_max_force[dir_index];
                if (prev_air_density >= 0.0f)
                    calibration_scheduled[dir_index] = true;
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

            _air_density = new_air_density;
            if (_current_mode_is_CoT)
                update_reference_vectors_for_CoT_mode();
        }

        public void check_thruster_status(IMyCubeBlock thruster)
        {
            thruster_info thruster_info = _all_thrusters[thruster.EntityId];
            thruster_info.operational   = thruster.IsWorking;
            lock (_changed_thrusters)
                _changed_thrusters.Add(thruster_info);
        }

        public void assign_thruster(IMyThrust thruster)
        {
            var new_thruster = new thruster_info();
            new_thruster.host_thruster        = thruster;
            new_thruster.grid_cells           = thruster.Min + thruster.Max;
            new_thruster.grid_centre_pos      = new_thruster.grid_cells * (_grid.GridSize / 2.0f);
            new_thruster.max_force            = new_thruster.actual_max_force = thruster.MaxThrust;
            new_thruster.CoM_offset           = new_thruster.grid_centre_pos - _grid_CoM_location;
            new_thruster.static_moment        = new_thruster.actual_static_moment = new_thruster.grid_centre_pos * new_thruster.max_force;
            new_thruster.nozzle_direction     = get_nozzle_orientation(thruster);
            new_thruster.torque_factor        = Vector3.Cross(new_thruster.CoM_offset, -_thrust_forward_vectors[(int) new_thruster.nozzle_direction]);
            new_thruster.reference_vector     = get_reference_vector(new_thruster, _grid_CoM_location, _thrust_forward_vectors[(int) new_thruster.nozzle_direction]);
            new_thruster.thrust_limit         = 1.0f;
            new_thruster.enable_limit         = new_thruster.enable_rotation = new_thruster.steering_on = new_thruster.collective_control_on = false;
            new_thruster.opposing_thruster    = null;
            new_thruster.next_tandem_thruster = new_thruster.prev_tandem_thruster = new_thruster;
            new_thruster.manual_throttle      = 0.0f;
            new_thruster.operational          = thruster.Enabled && thruster.IsFunctional;

            lock (_changed_thrusters)
            {
                _uncontrolled_thrusters.Add(new_thruster);
                _all_thrusters.Add(thruster.EntityId, new_thruster);
                thruster.IsWorkingChanged += check_thruster_status;
                _air_density = float.MinValue;
            }
            thruster_and_grid_tagger.attach_ECU(thruster, this);
        }

        public void dispose_thruster(IMyThrust thruster)
        {
            thruster_info removed_thruster_info = _all_thrusters[thruster.EntityId];
            thruster_and_grid_tagger.detach_ECU(thruster);
            lock (_changed_thrusters)
            {
                thruster.IsWorkingChanged -= check_thruster_status;
                _all_thrusters.Remove(thruster.EntityId);
                _changed_thrusters.Remove(removed_thruster_info);

                if (_uncontrolled_thrusters.Contains(removed_thruster_info))
                {
                    _uncontrolled_thrusters.Remove(removed_thruster_info);
                    return;
                }
                
                int dir_index = (int) get_nozzle_orientation(thruster);
                if (_controlled_thrusters[dir_index].Contains(removed_thruster_info))
                {
                    _calibration_interrupted = _calibration_in_progress;
                    _calibration_scheduled[dir_index] = true;
                    remove_thruster_from_lists(removed_thruster_info);
                    _max_force[dir_index] -= removed_thruster_info.max_force;
                    _controlled_thrusters[dir_index].Remove(removed_thruster_info);
                    return;
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

        public engine_control_unit(IMyCubeGrid grid_ref, torque_control grid_movement)
        {
            _changed_thrusters = _changed_thrusters1;
            
            _grid = (MyCubeGrid) grid_ref;
            _grid_movement = grid_movement;

            _control_sectors = new solver_entry[3 * 3];
            for (int index = 0; index < 3 * 3; ++index)
                _control_sectors[index] = new solver_entry();

            _physics_enable_delay = PHYSICS_ENABLE_DELAY;
        }

        #endregion

        #region Gyroscope handling

        private void calc_spherical_moment_of_inertia()
        {
            Vector3I grid_dimensions = _grid.Max - _grid.Min + Vector3I.One;
            int      low_dim         = grid_dimensions.X, med_dim = grid_dimensions.Y, high_dim = grid_dimensions.Z, temp;

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

            float radius_XY = (float) Math.Sqrt(grid_dimensions.X * grid_dimensions.Y);
            float radius_XZ = (float) Math.Sqrt(grid_dimensions.X * grid_dimensions.Z);
            float radius_YZ = (float) Math.Sqrt(grid_dimensions.Y * grid_dimensions.Z);
            _surface_radii[(int) thrust_dir.fore  ] = _surface_radii[(int) thrust_dir.aft      ] = Math.Min(radius_XZ, radius_YZ) * _grid.GridSize;
            _surface_radii[(int) thrust_dir.port  ] = _surface_radii[(int) thrust_dir.starboard] = Math.Min(radius_XY, radius_XZ) * _grid.GridSize;
            _surface_radii[(int) thrust_dir.dorsal] = _surface_radii[(int) thrust_dir.ventral  ] = Math.Min(radius_XY, radius_YZ) * _grid.GridSize;
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

        #region Control panel routines

        private void change_thruster_switch(long thruster_entity_id, Action<thruster_info, bool> invoke_setter, bool new_switch)
        {
            thruster_info thruster_info;

            if (_all_thrusters.TryGetValue(thruster_entity_id, out thruster_info))
            {
                invoke_setter(thruster_info, new_switch);
                lock (_changed_thrusters)
                    _changed_thrusters.Add(thruster_info);
            }
        }

        public void set_thruster_steering(long thruster_entity_id, bool steering_on)
        {
            //log_ECU_action("set_thruster_steering", steering_on.ToString());
            change_thruster_switch(thruster_entity_id,
                delegate (thruster_info thruster_info, bool new_switch)
                {
                    //log_ECU_action("set_thruster_steering", thruster_info.host_thruster.CustomName);
                    thruster_info.steering_on = thruster_info.enable_rotation = new_switch;
                },
                steering_on);
        }

        public void set_thruster_trim(long thruster_entity_id, bool trim_on)
        {
            //log_ECU_action("set_thruster_trim", trim_on.ToString());
            change_thruster_switch(thruster_entity_id,
                delegate (thruster_info thruster_info, bool new_switch)
                {
                    //log_ECU_action("set_thruster_trim", thruster_info.host_thruster.CustomName);
                    thruster_info.is_RCS = new_switch;
                },
                !trim_on);
        }

        public void set_thruster_limiter(long thruster_entity_id, bool limiter_on)
        {
            //log_ECU_action("set_thruster_limiter", limiter_on.ToString());
            change_thruster_switch(thruster_entity_id,
                delegate (thruster_info thruster_info, bool new_switch)
                {
                    //log_ECU_action("set_thruster_limiter", thruster_info.host_thruster.CustomName);
                    thruster_info.enable_limit = new_switch;
                },
                limiter_on);
        }

        public void set_thruster_linear_input(long thruster_entity_id, bool linear_disabled)
        {
            //log_ECU_action("set_thruster_linear_input", linear_disabled.ToString());
            change_thruster_switch(thruster_entity_id,
                delegate (thruster_info thruster_info, bool new_switch)
                {
                    //log_ECU_action("set_thruster_linear_input", thruster_info.host_thruster.CustomName);
                    thruster_info.disable_linear_input = new_switch;
                    _calibration_interrupted = _calibration_in_progress;
                    _calibration_scheduled[(int) thruster_info.nozzle_direction] = true;
                },
                linear_disabled);
        }

        public float extract_thrust_limit(long thruster_entity_id)
        {
            thruster_info thruster_info;

            if (_all_thrusters.TryGetValue(thruster_entity_id, out thruster_info))
            {
                if (_uncontrolled_thrusters.Contains(thruster_info))
                    return -2;
                if (!_is_solution_good[(int) thruster_info.nozzle_direction])
                    return -1;
                return thruster_info.thrust_limit * 100.0f;
            }
            return -2;
        }

        public void set_manual_throttle(long thruster_entity_id, float manual_throttle)
        {
            thruster_info thruster_info;

            //log_ECU_action("set_manual_throttle", manual_throttle.ToString());
            if (_all_thrusters.TryGetValue(thruster_entity_id, out thruster_info))
            {
                //log_ECU_action("set_manual_throttle", thruster_info.host_thruster.CustomName);
                if (manual_throttle < 0.0f)
                    manual_throttle = 0.0f;
                else if (manual_throttle > 1.0f)
                    manual_throttle = 1.0f;
                thruster_info.manual_throttle = manual_throttle;
            }
        }

        public void set_damper_enabled_axes(bool fore_aft_enable, bool port_starboard_enable, bool dorsal_ventral_enable)
        {
            //log_ECU_action("set_damper_enabled_axes", $"Z={fore_aft_enable} X={port_starboard_enable} Y={dorsal_ventral_enable}");
            _dampers_axes_enabled.Z =       fore_aft_enable ? 1.0f : 0.0f;
            _dampers_axes_enabled.X = port_starboard_enable ? 1.0f : 0.0f;
            _dampers_axes_enabled.Y = dorsal_ventral_enable ? 1.0f : 0.0f;
        }

        #endregion

        #region Flight controls handling

        private void check_manual_override()
        {
            bool    is_thrust_override_active     = false;
            bool[]  uncontrolled_override_checked = _uncontrolled_override_checked;
            float[] thrust_override_vector        = _thrust_override_vector;

            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                uncontrolled_override_checked[dir_index] = false;
                thrust_override_vector       [dir_index] = 0.0f;
            }
            foreach (thruster_info cur_thruster_info in _uncontrolled_thrusters)
            {
                if (!uncontrolled_override_checked[(int) cur_thruster_info.nozzle_direction] && cur_thruster_info.operational)
                {
                    if (cur_thruster_info.host_thruster.ThrustOverride >= 1.0f)
                    { 
                        thrust_override_vector       [(int) cur_thruster_info.nozzle_direction] = MIN_THROTTLE;
                        uncontrolled_override_checked[(int) cur_thruster_info.nozzle_direction] = is_thrust_override_active = true;
                    }
                }
            }
            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                foreach (thruster_info cur_thruster_info in _controlled_thrusters[dir_index])
                {
                    if (cur_thruster_info.manual_throttle > thrust_override_vector[dir_index] && cur_thruster_info.operational)
                    {
                        thrust_override_vector[dir_index] = cur_thruster_info.manual_throttle;
                        is_thrust_override_active         = true;
                    }
                }
            }
            _is_thrust_override_active = is_thrust_override_active;
            _thrust_override = Vector3.Clamp(recompose_vector(thrust_override_vector), neg_override_bound, pos_override_bound);
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
            if (secondary_ECU)
                return;
            var controller = current_controller as IMyShipController;
            if (controller == null)
                return;
            _match_velocity_with = ((Sandbox.Game.Entities.IMyControllableEntity) current_controller).RelativeDampeningEntity;
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
            out Vector3 linear_output, out Vector3 rotational_output, out bool main_gyro_override_active, out Vector3 main_gyro_override)
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
        }

        public void set_secondary_control_parameters(Vector3D world_linear_velocity, Vector3 target_linear_velocity, Vector3D world_angular_velocity, 
            Vector3 linear_input, Vector3 rotational_input, bool main_gyro_override_active, Vector3 main_gyro_override)
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
        }

        public void translate_damper_override(Vector3 input_override, IMyTerminalBlock controller)
        {
            Matrix cockpit_matrix;
            controller.Orientation.GetMatrix(out cockpit_matrix);

            Vector3 cockpit_damper_axis_disable = Vector3.Transform(input_override, cockpit_matrix);
            _dampers_axes_enabled.X = (Math.Abs(cockpit_damper_axis_disable.X) >= 0.5f) ? 0.0f : 1.0f;
            _dampers_axes_enabled.Y = (Math.Abs(cockpit_damper_axis_disable.Y) >= 0.5f) ? 0.0f : 1.0f;
            _dampers_axes_enabled.Z = (Math.Abs(cockpit_damper_axis_disable.Z) >= 0.5f) ? 0.0f : 1.0f;
        }

        public Vector3 get_damper_override_for_cockpit(IMyTerminalBlock controller)
        {
            Matrix inv_cockpit_matrix;
            controller.Orientation.GetMatrix(out inv_cockpit_matrix);
            inv_cockpit_matrix = Matrix.Invert(inv_cockpit_matrix);

            var result = Vector3.Transform(_dampers_axes_enabled, inv_cockpit_matrix);
            result.X = (Math.Abs(result.X) >= 0.5f) ? 0.0f : 1.0f;
            result.Y = (Math.Abs(result.Y) >= 0.5f) ? 0.0f : 1.0f;
            result.Z = (Math.Abs(result.Z) >= 0.5f) ? 0.0f : 1.0f;
            return result;
        }

        #endregion

        public void reset_ECU()
        {
            _prev_angular_velocity = Vector3.Zero;

            float[] current_trim = _current_trim, linear_integral = _linear_integral;
            for (int dir_index = 0; dir_index < 6; ++dir_index)
                current_trim[dir_index] = linear_integral[dir_index] = 0.0f;
        }

        public void handle_60Hz()
        {
            bool is_secondary = secondary_ECU;
            if (is_secondary)
            {
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

                torque_control grid_movement = _grid_movement;
                grid_movement.get_linear_and_angular_velocities(out _world_linear_velocity, out _world_angular_velocity);
                current_speed = (float) _world_linear_velocity.Length();
                if (_match_velocity_with?.Physics != null)
                    _target_velocity = _match_velocity_with.Physics.LinearVelocity;
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
                Vector3D current_grid_CoM = new_grid_CoM;
                bool     CoM_shifted      = (current_grid_CoM - _grid_CoM_location).LengthSquared() > 0.01f;

                if (CoM_shifted)
                {
                    _grid_CoM_location = current_grid_CoM;
                    refresh_reference_vectors(CoM_shifted: true);
                }
            }

            if (  autopilot_on || !_is_thrust_override_active && !_is_gyro_override_active 
                && _manual_rotation.LengthSquared() < 0.0001f && _manual_thrust.LengthSquared() < 0.0001f
                && (!rotational_damping_on || _world_angular_velocity.LengthSquared() < 0.0001f)
                && (!linear_dampers_on && !is_secondary || _grid.Physics.Gravity.LengthSquared() < 0.01f && current_speed < 0.1f))
            {
                handle_thrust_control(_world_linear_velocity, _target_velocity, _world_angular_velocity, sleep_mode_on: true);
                //if (autopilot_on)
                //    calculate_and_apply_torque();
            }
            else
            {
                handle_thrust_control(_world_linear_velocity, _target_velocity, _world_angular_velocity, sleep_mode_on: false);
                calculate_and_apply_torque();
            }
        }

        public void handle_4Hz_foreground()
        {
            reset_thrusters(null);
            reset_overrides();
            Vector3D current_grid_CoM = new_grid_CoM;
            lock (_all_thrusters)
            {
                _CoM_shifted |= (current_grid_CoM - _grid_CoM_location).LengthSquared() > 0.01f;
                if (_CoM_shifted)
                    _grid_CoM_location = current_grid_CoM;
            }
        }

        public void handle_4Hz_background()
        {
            if (_calibration_in_progress)
            {
                if (_calibration_complete || _calibration_interrupted)
                    set_up_thrust_limits();
            }
            if (!_calibration_in_progress)
            {
                bool CoM_shifted;

                lock (_all_thrusters)
                {
                    CoM_shifted  = _CoM_shifted;
                    _CoM_shifted = false;
                }
                check_changed_thrusters();
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
            _force_override_refresh = true;
        }

        public void handle_2s_period_background()
        {
            if (_individual_calibration_on && !_calibration_in_progress)
                prepare_individual_calibration();
        }

        public void perform_individual_calibration()
        {
            if (!_individual_calibration_on || !_calibration_in_progress || !_calibration_ready || _calibration_complete)
                return;

            _calibration_ready = false;
            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                if (!_calibration_interrupted && _calibration_scheduled[dir_index])
                {
                    if (CALIBRATION_DEBUG)
                        log_ECU_action("perform_individual_calibration", "Starting calibration on " + ((thrust_dir) dir_index).ToString() + " side");
                    _is_solution_good[dir_index] = _linear_solvers[dir_index].calculate_solution(_thruster_infos[dir_index].Count);
                }
            }
            _calibration_complete = !_calibration_interrupted;
        }
    }
}
