﻿using ParallelTasks;
using System;
using System.Collections.Generic;
using System.Text;

using Sandbox.Common.ObjectBuilders;
using Sandbox.Definitions;
using Sandbox.Game.Entities;
using Sandbox.ModAPI;
using Sandbox.ModAPI.Interfaces;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.ModAPI;
using VRage.Utils;
using VRageMath;

namespace ttdtwm
{
    sealed class engine_control_unit
    {
        #region fields

        const int   NUM_ROTATION_SAMPLES = 6, PHYSICS_ENABLE_DELAY = 6;
        const float DESCENDING_SPEED     = 0.5f, MIN_OVERRIDE = 1.001f;
        const bool  DEBUG_THR_ALWAYS_ON  = false, DEBUG_DISABLE_ALT_HOLD = false;

        enum thrust_dir { fore = 0, aft = 3, starboard = 1, port = 4, dorsal = 2, ventral = 5 };
        sealed class thruster_info     // Technically a struct
        {
            public float         max_force, actual_max_force, actual_min_force;
            public Vector3       max_torque, grid_centre_pos, static_moment, actual_static_moment, CoM_offset, reference_vector;
            public thrust_dir    nozzle_direction;
            public float         current_setting, thrust_limit, prev_setting, manual_throttle;
            public bool          enable_limit, enable_rotation, active_control_on, is_RCS, skip, is_reduced;
            public thruster_info next_tandem_thruster, prev_tandem_thruster, opposing_thruster;
            public string        throttle_setting;
        };

        private static readonly Vector3I[] _thrust_forward_vectors;

        private /*static*/ StringBuilder    _group_name     = new StringBuilder();
        private /*static*/ StringBuilder    _thruster_name  = new StringBuilder();
        private /*static*/ simplex_solver[] _linear_solvers =
        {
            new simplex_solver(),   // fore
            new simplex_solver(),   // starboard
            new simplex_solver(),   // dorsal
            new simplex_solver(),   // aft
            new simplex_solver(),   // port
            new simplex_solver()    // ventral
        };
        private readonly Action[] _solver_starters;

        private /*static*/ List<MyThrust> _thrusters_copy = new List<MyThrust>();
        private /*static*/ List<thruster_info>[] _thruster_infos =
        {
            new List<thruster_info>(),  // fore
            new List<thruster_info>(),  // starboard
            new List<thruster_info>(),  // dorsal
            new List<thruster_info>(),  // aft
            new List<thruster_info>(),  // port
            new List<thruster_info>()   // ventral
        };

        private static float[] __control_vector  = new float[6], __control_vector_copy = new float[6];
        private static float[] __braking_vector  = new float[6];
        private static float[] __requested_force = new float[6];
        private static float[] __actual_force    = new float[6];
        private static float[] __non_THR_force   = new float[6];

        private static float[][] __linear_component =
        {
            new float[6],   // fore
            new float[6],   // starboard
            new float[6],   // dorsal
            new float[6],   // aft
            new float[6],   // port
            new float[6],   // ventral
        };

        private static float[] __steering_input       = new float[6];
        private static float[] __steering_output      = new float[6];
        private static float[] __angular_velocity     = new float[6];
        private static float[] __angular_acceleration = new float[6];
        private static float[] __nominal_acceleration = new float[6];
        private static float[] __thrust_limits        = new float[6];

        private static Vector3[] __new_static_moment = new Vector3[6];
        private static   float[] __new_total_force   = new   float[6];

        private static float[] __local_gravity_inv         = new float[6];
        private static float[] __local_linear_velocity_inv = new float[6];
        private static float[] __thrust_override_vector    = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

        private static bool[] __uncontrolled_override_checked = new bool[6];

        private MyCubeGrid _grid;

        private Dictionary<MyThrust, thruster_info>[] _controlled_thrusters =
        {
            new Dictionary<MyThrust, thruster_info>(),   // fore
            new Dictionary<MyThrust, thruster_info>(),   // starboard
            new Dictionary<MyThrust, thruster_info>(),   // dorsal
            new Dictionary<MyThrust, thruster_info>(),   // aft
            new Dictionary<MyThrust, thruster_info>(),   // port
            new Dictionary<MyThrust, thruster_info>()    // ventral
        };
        private Dictionary<Vector3I, List<thruster_info>>[] _tandem_thrusters =
        {
            new Dictionary<Vector3I, List<thruster_info>>(),  // fore
            new Dictionary<Vector3I, List<thruster_info>>(),  // starboard
            new Dictionary<Vector3I, List<thruster_info>>(),  // dorsal
            new Dictionary<Vector3I, List<thruster_info>>(),  // aft
            new Dictionary<Vector3I, List<thruster_info>>(),  // port
            new Dictionary<Vector3I, List<thruster_info>>()   // ventral
        };
        private Dictionary<MyThrust, thruster_info> _uncontrolled_thrusters = new Dictionary<MyThrust, thruster_info>();
        private float[] _max_force        = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        private float[] _actual_max_force = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        private float[] _min_setting      = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

        private HashSet<MyGyro> _gyroscopes = new HashSet<MyGyro>();

        private bool     _calibration_scheduled = false, _calibration_in_progress = false;
        private Vector3D _grid_CoM_location = Vector3D.Zero, _prev_position = Vector3D.Zero;
        private MatrixD  _inverse_world_transform;
        private Matrix   _inverse_world_rotation_fixed;
        private float    _max_gyro_torque = 0.0f, _spherical_moment_of_inertia = 1.0f;
        private Task     _thrust_manager_task;
        private Task[]   _calibration_tasks = new Task[6];

        private  bool[]    _enable_integral   = { false, false, false, false, false, false };
        private  bool[]    _restrict_integral = { false, false, false, false, false, false };
        private  bool[]    _is_solution_good  = { false, false, false, false, false, false };
        private float[]    _current_trim      = new float[6];
        private float[]    _last_trim         = new float[6];
        private Vector3?[] _active_CoT        = new Vector3?[6];
        private Vector3    _local_angular_velocity, _prev_angular_velocity = Vector3.Zero, _torque, _manual_rotation, _prev_rotation = Vector3.Zero, _target_rotation, _gyro_override = Vector3.Zero;
        private bool       _current_mode_is_CoT = false, _new_mode_is_CoT = false, _force_CoT_mode = false, _is_gyro_override_active = false;
        private sbyte      _last_control_scheme = -1;
        private bool       _stabilisation_off = true, _all_engines_off = false, _active_control_on = false, _under_player_control = false, _force_override_refresh = false;
        private float      _angular_speed;

        private  bool   _integral_cleared = false, _landing_mode_on = false, _is_thrust_verride_active = false;
        private  bool[] _enable_linear_integral = {  true,  true,  true,  true,  true,  true };
        private float[] _linear_integral        = {  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f };
        private float   _speed, _vertical_speed;
        private Vector3 _manual_thrust, _thrust_override = Vector3.Zero, _linear_control = Vector3.Zero, _prev_control = Vector3.Zero;

        private Vector3[] _rotation_samples = new Vector3[NUM_ROTATION_SAMPLES];
        private Vector3   _sample_sum       = Vector3.Zero;
        private int       _current_index    = 0, _physics_enable_delay = PHYSICS_ENABLE_DELAY;

        #endregion

        #region Properties

        public bool linear_dampers_on { get; set; }
        public bool autopilot_on      { get; set; }

        public int  thrust_reduction      { get; private set; }
        public bool control_limit_reached { get; private set; }

        public bool active_control_enabled
        {
            get
            {
                return _active_control_on;
            }
        }

        public float vertical_speed
        {
            get
            {
                return _vertical_speed;
            }
        }

        public bool landing_mode_on
        {
            get
            {
                return _landing_mode_on;
            }
            set
            {
                _landing_mode_on = value;
            }
        }

        public bool CoT_mode_forced
        {
            get
            {
                return _force_CoT_mode;
            }
            set
            {
                _force_CoT_mode = value;
            }
        }

        #endregion

        #region DEBUG

        private void screen_info(string message, int display_time_ms, MyFontEnum font, bool controlled_only)
        {
            bool display = !controlled_only;

            if (!display)
            {
                var controller = MyAPIGateway.Session.ControlledObject as MyShipController;
                if (controller != null)
                    display = controller.CubeGrid == _grid;
            }
            if (display)
                MyAPIGateway.Utilities.ShowNotification(message, display_time_ms, font);
        }

        private void log_ECU_action(string method_name, string message)
        {
            MyLog.Default.WriteLine(string.Format("TTDTWM\tengine_control_unit<{0} [{1}]>.{2}(): {3}", _grid.DisplayName, _grid.EntityId, method_name, message));
            int num_controlled_thrusters = 0;
            foreach (var cur_direction in _controlled_thrusters)
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

        private void screen_text(string method_name, string message, int display_time_ms, bool controlled_only)
        {
            if (method_name == "")
                screen_info(string.Format("{0} {1}", controlled_only ? "" : ("\"" + _grid.DisplayName + "\""), message), display_time_ms, MyFontEnum.White, controlled_only);
            else
                screen_info(string.Format("engine_control_unit.{0}(): {1} {2}", method_name, controlled_only ? "" : ("\"" + _grid.DisplayName + "\""), message), display_time_ms, MyFontEnum.White, controlled_only);
        }

        private void screen_vector<type>(string method_name, string vector_name, type[] vector, int display_time_ms, bool controlled_only)
        {
            screen_text(method_name, string.Format("{0} = {1:F3}/{2:F3}/{3:F3}/{4:F3}/{5:F3}/{6:F3}", 
                vector_name,
                vector[(int) thrust_dir.fore     ],
                vector[(int) thrust_dir.aft      ],
                vector[(int) thrust_dir.starboard],
                vector[(int) thrust_dir.port     ],
                vector[(int) thrust_dir.dorsal   ],
                vector[(int) thrust_dir.ventral  ]), display_time_ms, controlled_only);
        }

        #endregion

        #region torque calculation

        private void refresh_thruster_info_for_single_direction(Dictionary<MyThrust, thruster_info> thrusters)
        {
            MyThrust      cur_thruster;
            thruster_info cur_thruster_info;

            foreach (var cur_thruster_entry in thrusters)
            {
                cur_thruster      = cur_thruster_entry.Key;
                cur_thruster_info = cur_thruster_entry.Value;
                cur_thruster_info.CoM_offset = cur_thruster_info.grid_centre_pos - _grid_CoM_location;
                cur_thruster_info.max_torque = Vector3.Cross(cur_thruster_info.CoM_offset, -cur_thruster.ThrustForwardVector * cur_thruster.BlockDefinition.ForceMagnitude);
            }
        }

        private void refresh_thruster_info()
        {
            refresh_thruster_info_for_single_direction(_uncontrolled_thrusters);
            for (int dir_index = 0; dir_index < 6; ++dir_index)
                refresh_thruster_info_for_single_direction(_controlled_thrusters[dir_index]);
            _calibration_scheduled = true;
        }

        private void check_manual_override()
        {
            thruster_info cur_thruster_info;
            float         thrust_override_value;
            uint          manual_throttle;
            bool          throttle_setting_stored;

            _is_thrust_verride_active = false;
            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                __uncontrolled_override_checked[dir_index] = false;
                __thrust_override_vector[       dir_index] = 0.0f;
            }
            foreach (var cur_thruster in _uncontrolled_thrusters)
            {
                cur_thruster_info = cur_thruster.Value;
                if (!__uncontrolled_override_checked[(int) cur_thruster_info.nozzle_direction] && cur_thruster.Key.IsWorking)
                {
                    thrust_override_value = cur_thruster.Key.CurrentStrength;
                    if (thrust_override_value >= 0.01f)
                        __thrust_override_vector[(int) cur_thruster_info.nozzle_direction] = 0.01f;
                    __uncontrolled_override_checked[(int) cur_thruster_info.nozzle_direction] = _is_thrust_verride_active = true;
                }
            }
            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                foreach (var cur_thruster in _controlled_thrusters[dir_index])
                {
                    cur_thruster_info                 = cur_thruster.Value;
                    throttle_setting_stored           = MyAPIGateway.Utilities.GetVariable(cur_thruster_info.throttle_setting, out manual_throttle);
                    cur_thruster_info.manual_throttle = !throttle_setting_stored ? 0.0f : (manual_throttle / 100.0f);
                }
                foreach (var cur_thruster in _controlled_thrusters[dir_index])
                {
                    if (cur_thruster.Value.manual_throttle >= 0.01f && cur_thruster.Key.IsWorking)
                    {
                        __thrust_override_vector[dir_index] = 0.01f;
                        _is_thrust_verride_active           = true;
                        break;
                    }
                }
            }
            recompose_vector(__thrust_override_vector, out _thrust_override);
        }

        private void calculate_and_apply_torque()
        {
            //if (MyAPIGateway.Multiplayer != null && !MyAPIGateway.Multiplayer.IsServer)
            //    return;

            const float MIN_ANGULAR_ACCELERATION = (float) (0.1 * Math.PI / 180.0);

            _torque = Vector3.Zero;
            foreach (var cur_direction in _controlled_thrusters)
            {
                foreach (var cur_thruster in cur_direction)
                {
                    //if (cur_thruster.Key.IsWorking)
                        _torque += cur_thruster.Value.max_torque * cur_thruster.Key.CurrentStrength;
                }
            }
            foreach (var cur_thruster in _uncontrolled_thrusters)
            {
                //if (cur_thruster.Key.IsWorking)
                    _torque += cur_thruster.Value.max_torque * cur_thruster.Key.CurrentStrength;
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
            else if (_torque.LengthSquared() > MIN_ANGULAR_ACCELERATION * MIN_ANGULAR_ACCELERATION * _spherical_moment_of_inertia * _spherical_moment_of_inertia)
            {
                Vector3 world_torque = Vector3.Transform(_torque, _grid.WorldMatrix.GetOrientation());
                _grid.Physics.AddForce(MyPhysicsForceType.APPLY_WORLD_IMPULSE_AND_WORLD_ANGULAR_IMPULSE, Vector3.Zero, null, world_torque);
            }
        }

        #endregion

        #region thrust calibration

        void start_solver(int dir_index)
        {
            try
            {
                _is_solution_good[dir_index] = _linear_solvers[dir_index].calculate_solution(_thruster_infos[dir_index].Count);
            }
            catch (Exception exp)
            {
                MyLog.Default.WriteLine(string.Format("TTDTWM\t\"{0}\" {1}(): {2}\n{3}", _grid.DisplayName, exp.Message, exp.TargetSite, exp.StackTrace));
                throw;
            }
        }

        void start_solver_fore()
        {
            start_solver((int) thrust_dir.fore);
        }

        void start_solver_aft()
        {
            start_solver((int) thrust_dir.aft);
        }

        void start_solver_starboard()
        {
            start_solver((int) thrust_dir.starboard);
        }

        void start_solver_port()
        {
            start_solver((int) thrust_dir.port);
        }

        void start_solver_dorsal()
        {
            start_solver((int) thrust_dir.dorsal);
        }

        void start_solver_ventral()
        {
            start_solver((int) thrust_dir.ventral);
        }

        private void perform_linear_calibration()
        {
            float x = 0.0f, y = 0.0f;

            _calibration_scheduled   = false;
            _calibration_in_progress = true;
            foreach (thrust_dir cur_direction in Enum.GetValues(typeof(thrust_dir)))
            {
                List<thruster_info> thruster_infos = _thruster_infos[(int) cur_direction];
                simplex_solver      linear_solver  = _linear_solvers[(int) cur_direction];

                //thruster_infos.Clear();
                //thruster_infos.AddRange(_controlled_thrusters[(int) cur_direction].Values);

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
                    linear_solver.items[index].max_value = thruster_infos[index].max_force;
                }

                //log_ECU_action("perform_linear_calibration", "Starting calibration on " + cur_direction.ToString() + " side");
                _calibration_tasks[(int) cur_direction] = MyAPIGateway.Parallel.Start(_solver_starters[(int) cur_direction]);
                //_calibration_tasks[(int) cur_direction].Wait();
            }
        }

        private void set_up_thrust_limits()
        {
            foreach (thrust_dir cur_direction in Enum.GetValues(typeof(thrust_dir)))
            {
                List<thruster_info> thruster_infos = _thruster_infos[(int) cur_direction];
                simplex_solver      linear_solver  = _linear_solvers[(int) cur_direction];

                for (int index = 0; index < thruster_infos.Count; ++index)
                {
                    if (!_is_solution_good[(int) cur_direction])
                    {
                        thruster_infos[index].thrust_limit    = 0.0f;
                        thruster_infos[index].enable_rotation = true;
                    }
                    else
                    {
                        thruster_infos[index].thrust_limit = (linear_solver.items[index].max_value > 1.0f)
                            ? (linear_solver.items[index].result / linear_solver.items[index].max_value) : 1.0f;
                        thruster_infos[index].enable_rotation = thruster_infos[index].active_control_on;
                    }
                    //log_ECU_action("set_up_thrust_limits", string.Format("{0} kN ({1})", linear_solver.items[index].result / 1000.0f, cur_direction));
                }

                /*
                log_ECU_action("set_up_thrust_limits", _is_solution_good[(int) cur_direction]
                    ? string.Format("successfully calibrated {0} thrusters on {1} side", thruster_infos.Count, cur_direction)
                    : string.Format("calibration on {0} side failed", cur_direction));
                */
            }
            _calibration_in_progress = false;
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

        private static void recompose_vector(float[] decomposed_vector, out Vector3 result_vector)
        {
            result_vector.Z = decomposed_vector[(int) thrust_dir.fore   ] - decomposed_vector[(int) thrust_dir.aft      ];
            result_vector.X = decomposed_vector[(int) thrust_dir.port   ] - decomposed_vector[(int) thrust_dir.starboard];
            result_vector.Y = decomposed_vector[(int) thrust_dir.ventral] - decomposed_vector[(int) thrust_dir.dorsal   ];
        }

        private void apply_thrust_settings(bool reset_all_thrusters)
        {
            float         setting, setting_ratio;
            bool          enforce_min_override, dry_run;
            thruster_info cur_thruster_info;

            if (reset_all_thrusters && _all_engines_off && !_force_override_refresh)
                return;

            /*
            if (MyAPIGateway.Multiplayer != null)
            {
                if (MyAPIGateway.Multiplayer.IsServer)
                {
                    if (false && _under_player_control && (sync_helper.local_player == null || !MyAPIGateway.Multiplayer.IsServerPlayer(sync_helper.local_player.Client)))
                        dry_run = true;

                }
                else if (!is_under_control_of(sync_helper.local_controller))
                    dry_run = true;
            }
            */

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

            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                if (reset_all_thrusters)
                    _current_trim[dir_index] = _last_trim[dir_index] = 0.0f;

                int opposite_dir = (dir_index < 3) ? (dir_index + 3) : (dir_index - 3);
                enforce_min_override = (_speed >= 0.1f || _angular_speed >= 0.002f) && _actual_max_force[dir_index] > _actual_max_force[opposite_dir] * MIN_OVERRIDE / 100.0f
                    || _landing_mode_on && linear_dampers_on && _vertical_speed > -DESCENDING_SPEED * 0.5f && _vertical_speed < DESCENDING_SPEED * 0.5f;
                foreach (var cur_thruster in _controlled_thrusters[dir_index])
                {
                    cur_thruster_info = cur_thruster.Value;
                    if (_force_override_refresh)
                        cur_thruster_info.prev_setting = cur_thruster.Key.CurrentStrength * 100.0f;

                    if (reset_all_thrusters || cur_thruster_info.actual_max_force < 1.0f || !cur_thruster.Key.IsWorking)
                    {
                        if (cur_thruster_info.prev_setting > 0.0f || reset_all_thrusters)
                        {
                            if (!dry_run)
                                cur_thruster.Key.SetValueFloat("Override", 0.0f);
                            cur_thruster_info.current_setting = cur_thruster_info.prev_setting = 0.0f;
                        }
                        continue;
                    }

                    setting = cur_thruster_info.current_setting * 100.0f;
                    if (enforce_min_override && setting < MIN_OVERRIDE)
                        setting = MIN_OVERRIDE;
                    setting_ratio = (cur_thruster_info.prev_setting == 0.0f) ? 1.0f : (setting / cur_thruster_info.prev_setting);
                    if (setting_ratio <= 0.9f || setting_ratio >= 1.1f || Math.Abs(setting - cur_thruster_info.prev_setting) >= 1.0f)
                    {
                        if (!dry_run)
                            cur_thruster.Key.SetValueFloat("Override", setting);
                        cur_thruster_info.prev_setting = setting;
                    }
                }
            }

            _all_engines_off        = reset_all_thrusters;
            _force_override_refresh = false;
        }

        private sbyte initialise_linear_controls(Vector3 local_linear_velocity_vector, Vector3 local_gravity_vector)
        {
            const float DAMPING_CONSTANT = -2.0f, INTEGRAL_CONSTANT = 0.05f;

            _linear_control = Vector3.Clamp(_manual_thrust + _thrust_override, -Vector3.One, Vector3.One);
            decompose_vector(_linear_control, __control_vector);
            sbyte control_scheme    = get_current_control_scheme();
            float gravity_magnitude = local_gravity_vector.Length();
            bool  controls_active   = _linear_control.LengthSquared() > 0.0001f;

            _stabilisation_off = false;

            if (!linear_dampers_on)
            {
                if (!_integral_cleared)
                {
                    for (int dir_index = 0; dir_index < 6; ++dir_index)
                    {
                        _enable_linear_integral[dir_index] = false;
                        _linear_integral[dir_index]        = __braking_vector[dir_index] = 0.0f;
                    }
                    _integral_cleared = true;
                }
                _stabilisation_off |= !controls_active && gravity_magnitude > 0.1f && _vertical_speed > -DESCENDING_SPEED * 0.5f && _vertical_speed < DESCENDING_SPEED * 0.5f;
            }
            else
            {
                _integral_cleared      = false;
                Vector3 linear_damping = local_linear_velocity_vector * DAMPING_CONSTANT;
                if (!_landing_mode_on)
                    linear_damping -= local_gravity_vector;
                else
                {
                    float counter_thrust_limit = -_vertical_speed / DESCENDING_SPEED;
                    if (counter_thrust_limit < 0.0f)
                        counter_thrust_limit = 0.0f;
                    else if (counter_thrust_limit > 0.5f)
                        counter_thrust_limit = 0.5f;
                    linear_damping     -= local_gravity_vector * counter_thrust_limit;
                    _stabilisation_off |= !controls_active && (control_limit_reached || _vertical_speed >= -DESCENDING_SPEED * 0.5f  && _vertical_speed < 0.0f);
                }
                decompose_vector(linear_damping * _grid.Physics.Mass,            __braking_vector);
                decompose_vector(              -local_gravity_vector,         __local_gravity_inv);
                decompose_vector(      -local_linear_velocity_vector, __local_linear_velocity_inv);
                Array.Copy(__control_vector, __control_vector_copy, 6);

                for (int dir_index = 0, opposite_dir = 3; dir_index < 3; ++dir_index, ++opposite_dir)
                {
                    _enable_linear_integral[dir_index] = _enable_linear_integral[opposite_dir] = !DEBUG_DISABLE_ALT_HOLD &&
                           (  __local_gravity_inv[dir_index] >  0.0f  ||   __local_gravity_inv[opposite_dir] >  0.0f)
                        &&  __control_vector_copy[dir_index] <  0.01f && __control_vector_copy[opposite_dir] <  0.01f;
                    _enable_linear_integral[   dir_index] &= _actual_max_force[   dir_index] >= 1.0f;
                    _enable_linear_integral[opposite_dir] &= _actual_max_force[opposite_dir] >= 1.0f;


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

                    float gravity_ratio = (gravity_magnitude < 0.01f) ? 1.0f : ((__local_gravity_inv[dir_index] + __local_gravity_inv[opposite_dir]) / gravity_magnitude),
                          axis_speed    = __local_linear_velocity_inv[dir_index] + __local_linear_velocity_inv[opposite_dir],
                          linear_integral_change = INTEGRAL_CONSTANT * (__local_linear_velocity_inv[dir_index] - __local_linear_velocity_inv[opposite_dir]);
                    if (linear_integral_change > INTEGRAL_CONSTANT)
                        linear_integral_change = INTEGRAL_CONSTANT;
                    else if (linear_integral_change < -INTEGRAL_CONSTANT)
                        linear_integral_change = -INTEGRAL_CONSTANT;
                    if (axis_speed < 1.0f)
                        linear_integral_change *= axis_speed * (gravity_ratio - 1.0f) + 1.0f;
                    else
                        linear_integral_change *= gravity_ratio;
                    if (linear_integral_change > 0.0f)
                        set_linear_integral(   dir_index, opposite_dir,  linear_integral_change);
                    else if (linear_integral_change < 0.0f)
                        set_linear_integral(opposite_dir,    dir_index, -linear_integral_change);

                    if (_landing_mode_on)
                    {
                        float dir_multiplier = (_vertical_speed > -DESCENDING_SPEED * 0.5f) ? 1.0f : gravity_ratio;

                        _linear_integral[dir_index] -= INTEGRAL_CONSTANT * DESCENDING_SPEED * dir_multiplier;
                        if (_linear_integral[dir_index] < 0.0f)
                            _linear_integral[dir_index] = 0.0f;
                        _linear_integral[opposite_dir] -= INTEGRAL_CONSTANT * DESCENDING_SPEED * dir_multiplier;
                        if (_linear_integral[opposite_dir] < 0.0f)
                            _linear_integral[opposite_dir] = 0.0f;
                    }
                    /*
                    if (__local_gravity_inv[dir_index] <= 0.0f && __local_gravity_inv[opposite_dir] <= 0.0f)
                        _linear_integral[dir_index] = _linear_integral[opposite_dir] = 0.0f;
                    */
                }
            }

            return control_scheme;
        }

        private void set_linear_integral(int dir_index, int opposite_dir, float linear_integral_change)
        {
            if (_linear_integral[opposite_dir] <= 0.0f)
            {
                if (_enable_linear_integral[dir_index])
                    _linear_integral[dir_index] += linear_integral_change;
                _linear_integral[opposite_dir] = 0.0f;
            }
            else
            {
                _linear_integral[opposite_dir] -= linear_integral_change;
                if (_linear_integral[opposite_dir] >= 0.0f)
                    _linear_integral[dir_index] = 0.0f;
                else
                {
                    _linear_integral[   dir_index] = -_linear_integral[opposite_dir];
                    _linear_integral[opposite_dir] = 0.0f;
                }
            }
        }

        private void set_brake(int dir_index, int opposite_dir)
        {
            if (__control_vector_copy[opposite_dir] < 0.01f && _actual_max_force[dir_index] >= 1.0f)
            {
                float braking_force = __braking_vector[dir_index] + _grid.Physics.Mass * _linear_integral[dir_index];

                if (_actual_max_force[opposite_dir] < 1.0f)
                    braking_force -= _grid.Physics.Mass * _linear_integral[opposite_dir];
                __control_vector[dir_index] += braking_force / _actual_max_force[dir_index];
                if (__control_vector[dir_index] < 0.0f)
                    __control_vector[dir_index] = 0.0f;
                else if (__control_vector[dir_index] >= 1.0f)
                {
                    __control_vector[dir_index]        = 1.0f;
                    _enable_linear_integral[dir_index] = _enable_linear_integral[opposite_dir] = false;
                }
            }
        }

        private sbyte get_current_control_scheme()
        {
            const float MIN_CONTROL = 0.3f;

            sbyte result;

            if (__control_vector[(int) thrust_dir.fore] > MIN_CONTROL)
                result = 1;
            else if (__control_vector[(int) thrust_dir.aft] > MIN_CONTROL)
                result = 2;
            else
                result = 0;

            if (__control_vector[(int) thrust_dir.starboard] > MIN_CONTROL)
                result += 3;
            else if (__control_vector[(int) thrust_dir.port] > MIN_CONTROL)
                result += 6;

            if (__control_vector[(int) thrust_dir.dorsal] > MIN_CONTROL)
                result += 9;
            else if (__control_vector[(int) thrust_dir.ventral] > MIN_CONTROL)
                result += 18;

            return result;
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
            const float DAMPING_CONSTANT = 10.0f, MIN_LINEAR_OPPOSITION = 0.1f, MAX_LINEAR_OPPOSITION = 0.2f;

            if (_actual_max_force[cur_dir] <= 1.0f)
            {
                __new_static_moment[cur_dir] = Vector3.Zero;
                return;
            }

            Vector3 angular_velocity_diff = desired_angular_velocity - _local_angular_velocity, total_static_moment = Vector3.Zero;
            float   max_linear_opposition, damping = DAMPING_CONSTANT * _grid.Physics.Mass / _actual_max_force[cur_dir], current_limit = __thrust_limits[cur_dir], 
                    angular_velocity_diff_magnitude = angular_velocity_diff.Length(), total_force = 0.0f, control, min_setting = _min_setting[cur_dir];
            float[] linear_component = __linear_component[cur_dir];
            bool    enforce_thrust_limit = !_current_mode_is_CoT && __control_vector[opposite_dir] >= 0.01f, THR_mode_used;

            max_linear_opposition = MAX_LINEAR_OPPOSITION * (1.0f - __control_vector[opposite_dir]) + MIN_LINEAR_OPPOSITION * __control_vector[opposite_dir];
            if (_actual_max_force[opposite_dir] < _actual_max_force[cur_dir])
                max_linear_opposition *= _actual_max_force[opposite_dir] / _actual_max_force[cur_dir];
            //max_linear_opposition += min_setting;
            if (max_linear_opposition > MAX_LINEAR_OPPOSITION)
                max_linear_opposition = MAX_LINEAR_OPPOSITION;

            foreach (var cur_thruster_info in _controlled_thrusters[cur_dir].Values)
            {
                if (cur_thruster_info.skip)
                {
                    cur_thruster_info.is_reduced = false;
                    continue;
                }
                cur_thruster_info.is_reduced = Vector3.Dot(angular_velocity_diff, cur_thruster_info.max_torque) < 0.0f;

                decompose_vector(Vector3.Cross(angular_velocity_diff, cur_thruster_info.reference_vector), linear_component);
                if (linear_component[cur_dir] > 0.0f)
                {
                    control = linear_component[cur_dir];
                    //if (control > 1.0f)
                    //    control = 1.0f;
                    cur_thruster_info.current_setting += damping * control + max_linear_opposition * (1.0f - __thrust_limits[cur_dir]);
                    if (enforce_thrust_limit && cur_thruster_info.active_control_on && !cur_thruster_info.is_RCS)
                    {
                        // Limit thrusters opposing player/ID linear input
                        if (cur_thruster_info.current_setting > max_linear_opposition)
                            cur_thruster_info.current_setting = max_linear_opposition;
                    }
                    else if (cur_thruster_info.current_setting > 1.0f)
                        cur_thruster_info.current_setting = 1.0f;

                    if (_force_CoT_mode)
                    {
                        total_static_moment += cur_thruster_info.current_setting * cur_thruster_info.actual_static_moment;
                        total_force         += cur_thruster_info.current_setting * cur_thruster_info.actual_max_force;
                    }
                }
                else if (linear_component[opposite_dir] > 0.0f)
                {
                    THR_mode_used = cur_thruster_info.active_control_on && !cur_thruster_info.is_RCS;
                    control       = linear_component[opposite_dir];
                    //if (control > 1.0f)
                    //    control = 1.0f;
                    cur_thruster_info.current_setting -= damping * control;
                    if (THR_mode_used)
                        cur_thruster_info.current_setting += 0.5f * cur_thruster_info.thrust_limit * __control_vector[cur_dir] * (1.0f - cur_thruster_info.current_setting);
                    if (cur_thruster_info.current_setting < min_setting)
                        cur_thruster_info.current_setting = min_setting;
                    if (enforce_thrust_limit && THR_mode_used && cur_thruster_info.current_setting > max_linear_opposition)
                    {
                        // Limit thrusters opposing player/ID linear input
                        cur_thruster_info.current_setting = max_linear_opposition;
                    }

                    if (_force_CoT_mode)
                    {
                        total_static_moment += cur_thruster_info.current_setting * cur_thruster_info.actual_static_moment;
                        total_force         += cur_thruster_info.current_setting * cur_thruster_info.actual_max_force;
                    }
                }

                if (cur_thruster_info.is_reduced && cur_thruster_info.current_setting > current_limit)
                    cur_thruster_info.current_setting = current_limit;
            }

            __new_static_moment[cur_dir] = total_static_moment;
            __new_total_force  [cur_dir] = total_force;

            if (_force_CoT_mode && _active_CoT[opposite_dir] != null)
            {
                Vector3 effective_CoT = (Vector3) _active_CoT[opposite_dir], thruster_dir = _thrust_forward_vectors[cur_dir];

                foreach (var cur_thruster_info in _controlled_thrusters[cur_dir].Values)
                {
                    if (cur_thruster_info.skip || !cur_thruster_info.is_reduced && cur_thruster_info.current_setting > 0.5f)
                        continue;

                    decompose_vector(Vector3.Cross(angular_velocity_diff, get_reference_vector(cur_thruster_info, effective_CoT, thruster_dir)), linear_component);
                    if (linear_component[cur_dir] > 0.0f)
                    {
                        control = damping * linear_component[cur_dir];
                        if (__control_vector[opposite_dir] >= 0.01f && cur_thruster_info.active_control_on && !cur_thruster_info.is_RCS && control > max_linear_opposition)
                            control = max_linear_opposition;
                        cur_thruster_info.current_setting += control;
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

                foreach (var cur_thruster_info in _controlled_thrusters[dir_index].Values)
                {
                    first_opposite_trhuster = cur_thruster_info.opposing_thruster;
                    if (first_opposite_trhuster != null)
                    {
                        cur_opposite_thruster = first_opposite_trhuster;
                        do
                        {
                            //screen_text("", string.Format("CT({0}) = {1:F2}, OT({2}) = {3:F2}", cur_thruster_info.nozzle_direction, cur_thruster_info.current_setting, cur_opposite_thruster.nozzle_direction, cur_opposite_thruster.current_setting), 16, controlled_only: true);

                            if (cur_opposite_thruster.current_setting >= cur_thruster_info.current_setting)
                            {
                                cur_opposite_thruster.current_setting -= cur_thruster_info.current_setting;
                                cur_thruster_info.current_setting      = 0.0f;
                                break;
                            }
                            cur_thruster_info.current_setting    -= cur_opposite_thruster.current_setting;
                            cur_opposite_thruster.current_setting = 0.0f;

                            cur_opposite_thruster = cur_opposite_thruster.next_tandem_thruster;
                        }
                        while (cur_opposite_thruster != first_opposite_trhuster);
                    }
                }
            };
            //for (int dir_index = 0; dir_index < 3; ++dir_index)
            //    eliminate_direct_opposition(dir_index);
            MyAPIGateway.Parallel.For(0, 3, eliminate_direct_opposition);

            for (int dir_index = 0, opposite_dir = 3; dir_index < 3; ++dir_index, ++opposite_dir)
            { 
                foreach (var cur_thruster_info in _controlled_thrusters[dir_index].Values)
                {
                    if (max_setting < cur_thruster_info.current_setting)
                        max_setting = cur_thruster_info.current_setting;
                }
                foreach (var cur_thruster_info in _controlled_thrusters[opposite_dir].Values)
                {
                    if (max_setting < cur_thruster_info.current_setting)
                        max_setting = cur_thruster_info.current_setting;
                }
                if (max_control < __control_vector[   dir_index])
                    max_control = __control_vector[   dir_index];
                if (max_control < __control_vector[opposite_dir])
                    max_control = __control_vector[opposite_dir];
            }

            if (max_setting > 0.0f)
            {
                float max_normalisation_multiplier = 1.0f / max_setting;

                if (max_normalisation_multiplier > MAX_NORMALISATION)
                    max_normalisation_multiplier = MAX_NORMALISATION;
                max_normalisation_multiplier = 1.0f + max_control * (max_normalisation_multiplier - 1.0f);

                Action<int> normalise_direction = delegate (int dir_index)
                {
                    float min_setting = _min_setting[dir_index];

                    __actual_force[dir_index] = __non_THR_force[dir_index] = 0.0f;
                    foreach (var cur_thruster_info in _controlled_thrusters[dir_index].Values)
                    {
                        cur_thruster_info.current_setting *= max_normalisation_multiplier;
                        if (cur_thruster_info.current_setting < min_setting)
                            cur_thruster_info.current_setting = min_setting;

                        if (!cur_thruster_info.active_control_on || cur_thruster_info.is_RCS)
                            __non_THR_force[dir_index] += cur_thruster_info.current_setting * cur_thruster_info.actual_max_force;
                        else
                        {
                            __non_THR_force[dir_index] += cur_thruster_info.actual_min_force;
                            __actual_force [dir_index] += cur_thruster_info.current_setting * cur_thruster_info.actual_max_force - cur_thruster_info.actual_min_force;
                        }
                    }
                };
                //for (int dir_index = 0; dir_index < 6; ++dir_index)
                //    normalise_direction(dir_index);
                MyAPIGateway.Parallel.For(0, 6, normalise_direction);
            }

            Action<int> equalise_2_directions = delegate (int dir_index)
            {
                int   opposite_dir     = dir_index + 3;
                float force1           = __actual_force[   dir_index] + __non_THR_force[   dir_index];
                float force2           = __actual_force[opposite_dir] + __non_THR_force[opposite_dir];
                float new_force_ratio1 = 1.0f, new_force_ratio2 = 1.0f;
                float min_setting1 = _min_setting[dir_index], min_setting2 = _min_setting[opposite_dir];

                if (__actual_force[dir_index] >= 1.0f && force1 - __requested_force[dir_index] > force2)
                {
                    new_force_ratio1 = (force2 + __requested_force[dir_index] - __non_THR_force[dir_index]) / __actual_force[dir_index];
                    if (new_force_ratio1 < 0.0f)
                        new_force_ratio1 = 0.0f;
                    else if (new_force_ratio1 > 1.0f)
                        new_force_ratio1 = 1.0f;

                    foreach (var cur_thruster_info in _controlled_thrusters[dir_index].Values)
                    {
                        if (cur_thruster_info.active_control_on && !cur_thruster_info.is_RCS)
                            cur_thruster_info.current_setting = (cur_thruster_info.current_setting - min_setting1) * new_force_ratio1 + min_setting1;
                    }
                }
                if (__actual_force[opposite_dir] >= 1.0f && force2 - __requested_force[opposite_dir] > force1)
                {
                    new_force_ratio2 = (force1 + __requested_force[opposite_dir] - __non_THR_force[opposite_dir]) / __actual_force[opposite_dir];
                    if (new_force_ratio2 < 0.0f)
                        new_force_ratio2 = 0.0f;
                    else if (new_force_ratio2 > 1.0f)
                        new_force_ratio2 = 1.0f;

                    foreach (var cur_thruster_info in _controlled_thrusters[opposite_dir].Values)
                    {
                        if (cur_thruster_info.active_control_on && !cur_thruster_info.is_RCS)
                            cur_thruster_info.current_setting = (cur_thruster_info.current_setting - min_setting2) * new_force_ratio2 + min_setting2;
                    }
                }
                __actual_force[   dir_index] *= new_force_ratio1;
                __actual_force[opposite_dir] *= new_force_ratio2;
            };
            //for (int dir_index = 0; dir_index < 3; ++dir_index)
            //    equalise_2_directions(dir_index);
            MyAPIGateway.Parallel.For(0, 3, equalise_2_directions);

            for (int dir_index = 0, opposite_dir = 3; dir_index < 3; ++dir_index, ++opposite_dir)
            { 
                if (   __requested_force[   dir_index] >= _actual_max_force[   dir_index] * 0.3f && _actual_max_force[   dir_index] >= 0.05f * _grid.Physics.Mass 
                    || __requested_force[opposite_dir] >= _actual_max_force[opposite_dir] * 0.3f && _actual_max_force[opposite_dir] >= 0.05f * _grid.Physics.Mass)
                {
                    zero_thrust_reduction = false;
                    dir_force1 = (__actual_force[dir_index] + __non_THR_force[dir_index]) - (__actual_force[opposite_dir] + __non_THR_force[opposite_dir]);
                    dir_force2 = __requested_force[dir_index] + __requested_force[opposite_dir];
                    if (dir_force1 > dir_force2)
                        dir_force1 = dir_force2;
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
                    thrust_reduction = (requested_force < 0.05f * _grid.Physics.Mass) ? 0 : ((int) ((1.0f - linear_force / requested_force) * 100.0f + 0.5f));

                    if (thrust_reduction < 0)
                        thrust_reduction = 0;
                }
            }
        }

        private bool adjust_trim_setting(sbyte control_scheme, out Vector3 desired_angular_velocity)
        {
            const float ANGULAR_INTEGRAL_COEFF = -0.4f, ANGULAR_DERIVATIVE_COEFF = -0.005f, MAX_TRIM = 5.0f, THRUST_CUTOFF_TRIM = 4.0f;

            bool    update_inverse_world_matrix = false;
            float   trim_change, thrust_limit_pitch, thrust_limit_yaw, thrust_limit_roll;
            Vector3 local_angular_acceleration  = (_local_angular_velocity - _prev_angular_velocity) * MyEngineConstants.UPDATE_STEPS_PER_SECOND, trim_vector;
            _prev_angular_velocity = _local_angular_velocity;

            decompose_vector(          _manual_rotation,       __steering_input);
            decompose_vector(   _local_angular_velocity,     __angular_velocity);
            decompose_vector(local_angular_acceleration, __angular_acceleration);
            Vector3 nominal_acceleration_vector = _torque / _spherical_moment_of_inertia;
            if (nominal_acceleration_vector.LengthSquared() > 1.0f)
                nominal_acceleration_vector.Normalize();
            decompose_vector(nominal_acceleration_vector, __nominal_acceleration);
            int opposite_dir = 3;
            control_limit_reached = false;
            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                if (__steering_input[opposite_dir] > 0.01f)
                    __steering_output[dir_index] = 0.0f;
                else if (__steering_input[dir_index] > 0.01f)
                {
                    __steering_output[dir_index] = /*(__angular_velocity[dir_index] < 0.1f) ? */
                          (__steering_input[dir_index] * 15.0f) 
                        /*: (__angular_velocity[dir_index] + __steering_input[dir_index] * 3.0f)*/;
                    //__steering_output[dir_index]  = __steering_input[dir_index] * 3.0f + __angular_velocity[dir_index] + __angular_velocity[opposite_dir];
                    __steering_output[dir_index] += _last_trim[dir_index];
                    _enable_integral[dir_index]   = _enable_integral[opposite_dir] = false;
                    _current_trim[   dir_index]   = _current_trim[   opposite_dir] = 0.0f;
                    _last_trim[   opposite_dir]  *= 0.95f;
                    update_inverse_world_matrix   = true;
                }
                else
                {
                    if (_stabilisation_off || !_active_control_on)
                        _current_trim[dir_index] = _last_trim[dir_index] = 0.0f;
                    else if (_enable_integral[dir_index])
                    {
                        if (_landing_mode_on)
                            _restrict_integral[dir_index] = update_inverse_world_matrix = true;
                        else if (_restrict_integral[dir_index] && __angular_velocity[dir_index] < 0.01f)
                        {
                            _restrict_integral[dir_index] = false;
                            update_inverse_world_matrix   = true;
                        }
                        else if (_is_gyro_override_active && __angular_velocity[dir_index] > 0.2f)
                            _restrict_integral[dir_index] = !_stabilisation_off && _active_control_on;

                        if (!_restrict_integral[dir_index] || __angular_acceleration[opposite_dir] < 0.05f)
                        {
                            trim_change = ANGULAR_INTEGRAL_COEFF * ((!_restrict_integral[dir_index] || __angular_velocity[dir_index] < 0.01f) ? __angular_velocity[dir_index] : 0.01f);
                            // _nominal_acceleration is used to combat trim "stickiness" caused by rotational friction
                            if (__angular_velocity[dir_index] < 0.0001f && __angular_velocity[opposite_dir] < 0.0001f)
                                 trim_change += ANGULAR_DERIVATIVE_COEFF * __nominal_acceleration[dir_index];

                            if (_current_trim[dir_index] > 0.0f)
                            {
                                // trim_change is always negative or zero
                                _current_trim[dir_index] += trim_change;
                                if (_current_trim[dir_index] < 0.0f)
                                {
                                    _current_trim[opposite_dir] = -_current_trim[dir_index];
                                    _current_trim[   dir_index] = 0.0f;
                                    control_limit_reached      |= _current_trim[opposite_dir] >= MAX_TRIM;
                                }
                            }
                            else
                            {
                                // trim_change is always negative or zero
                                if (_current_trim[opposite_dir] < MAX_TRIM)
                                    _current_trim[opposite_dir] -= trim_change;
                                else
                                    control_limit_reached = true;
                            }
                        }

                        if (_current_trim[opposite_dir] <= 0.0f)
                        {
                            _last_trim[dir_index] = (_last_trim[dir_index] <= _current_trim[dir_index]) ? 
                                    _current_trim[dir_index]
                                : ((_current_trim[dir_index] + _last_trim[dir_index]) / 2.0f);
                        }
                        else
                            _last_trim[dir_index] = 0.0f;
                    }
                    else
                    {
                        _enable_integral[  dir_index] = update_inverse_world_matrix = true;
                        _restrict_integral[dir_index] =  !_stabilisation_off && _active_control_on && __angular_velocity[dir_index] > 0.0f;
                        _current_trim[     dir_index] = (!_stabilisation_off && _active_control_on && control_scheme == _last_control_scheme) ? _last_trim[dir_index] : 0.0f;
                    }

                    __steering_output[dir_index] = __angular_velocity[opposite_dir] + _current_trim[dir_index];
                }

                if (++opposite_dir >= 6)
                    opposite_dir = 0;
            }

            recompose_vector(_current_trim, out trim_vector);
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

            recompose_vector(__steering_output, out desired_angular_velocity);
            desired_angular_velocity += ANGULAR_DERIVATIVE_COEFF * local_angular_acceleration;
            _last_control_scheme      = control_scheme;
            return update_inverse_world_matrix;
        }

        private void handle_thrust_control(Vector3 world_linear_velocity, bool sleep_mode_on)
        {
            // Using "fixed" (it changes orientation only when the player steers a ship) inverse rotation matrix here to 
            // prevent Dutch Roll-like tendencies at high speeds
            Vector3 local_linear_velocity = Vector3.Transform(world_linear_velocity, _inverse_world_rotation_fixed);

            Matrix       inverse_world_rotation = _inverse_world_transform.GetOrientation();
            //BoundingBoxD grid_bounding_box      = _grid.PositionComp.WorldAABB;
            //MyPlanet     closest_planetoid      = MyGamePruningStructure.GetClosestPlanet(ref grid_bounding_box);
            //Vector3      world_gravity          = (closest_planetoid == null) ? Vector3.Zero : closest_planetoid.GetWorldGravity(grid_bounding_box.Center);
            Vector3      local_gravity          = Vector3.Transform(_grid.Physics.Gravity, inverse_world_rotation);
            float        gravity_magnitude      = local_gravity.Length();
            _vertical_speed = (gravity_magnitude < 0.1f) ? 0.0f : (Vector3.Dot(local_linear_velocity, local_gravity) / (-gravity_magnitude));
            if (sleep_mode_on)
            {
                thrust_reduction = 0;
                apply_thrust_settings(reset_all_thrusters: true);
                return;
            }

            _local_angular_velocity = Vector3.Transform(_grid.Physics.AngularVelocity, inverse_world_rotation);
            if (_is_gyro_override_active)
                _local_angular_velocity -= _gyro_override;
            _angular_speed = _local_angular_velocity.Length();

            Vector3 desired_angular_velocity;
            sbyte   control_scheme = initialise_linear_controls(local_linear_velocity, local_gravity);
            bool    update_inverse_world_matrix;
            update_inverse_world_matrix = adjust_trim_setting(control_scheme, out desired_angular_velocity);

            // Update fixed inverse rotation matrix when angle exceeds 11 degrees or speed is low 
            // (decoupling inertia dampers' axes from ship orientation isn't needed at low velocities)
            if (update_inverse_world_matrix || _speed <= 20.0f || Vector3.Dot(_inverse_world_rotation_fixed.Forward, inverse_world_rotation.Forward) < 0.98f)
                _inverse_world_rotation_fixed = inverse_world_rotation;

            _new_mode_is_CoT = true; 

            //int counter = 6;
            Action<int> set_up_thrusters = delegate (int dir_index)
            {
                thruster_info cur_thruster_info;
                float         control = __control_vector[dir_index], min_setting = _min_setting[dir_index];

                if (__control_vector[dir_index] > 0.05f)
                    _new_mode_is_CoT = _force_CoT_mode;
                __requested_force[dir_index] = 0.0f;
                foreach (var cur_thruster in _controlled_thrusters[dir_index])
                {
                    cur_thruster_info = cur_thruster.Value;
                    if (!cur_thruster.Key.IsWorking || cur_thruster_info.actual_max_force < 1.0f)
                    { 
                        cur_thruster_info.current_setting = 0.0f;
                        cur_thruster_info.skip            = true;
                    }
                    else
                    {
                        cur_thruster_info.current_setting = ((cur_thruster_info.manual_throttle >= 0.01f) ? cur_thruster_info.manual_throttle : control) + min_setting;
                        if (cur_thruster_info.current_setting > 1.0f)
                            cur_thruster_info.current_setting = 1.0f;
                        __requested_force[dir_index] += cur_thruster_info.current_setting * cur_thruster_info.actual_max_force;
                        if (cur_thruster_info.enable_limit && (!cur_thruster_info.enable_rotation || cur_thruster_info.active_control_on))
                        {
                            cur_thruster_info.current_setting *= cur_thruster_info.thrust_limit;
                            if (cur_thruster_info.current_setting < min_setting)
                                cur_thruster_info.current_setting = min_setting;
                        }
                        cur_thruster_info.skip = !cur_thruster_info.enable_rotation;
                    }
                }
                adjust_thrust_for_steering(dir_index, (dir_index < 3) ? (dir_index + 3) : (dir_index - 3), desired_angular_velocity);
                //--counter;
            };

            //if (MyAPIGateway.Parallel != null)
            MyAPIGateway.Parallel.For(0, 6, set_up_thrusters);
            //else
            /*
            {
                for (int dir_index = 0; dir_index < 6; ++dir_index)
                    set_up_thrusters(dir_index);
            }
            */
            //if (counter > 0)
            //    throw new Exception("Non-blocking execution detected");

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
                Dictionary<MyThrust, thruster_info> cur_direction = _controlled_thrusters[  dir_index];
                Vector3                             thruster_dir  = _thrust_forward_vectors[dir_index];

                foreach (var cur_thruster_info in cur_direction.Values)
                    cur_thruster_info.reference_vector = get_reference_vector(cur_thruster_info, _grid_CoM_location, thruster_dir);
            }
        }

        private void update_reference_vectors_for_CoT_mode()
        {
            Vector3                             total_static_moment, CoT_location, thruster_dir;
            Dictionary<MyThrust, thruster_info> cur_direction;

            for (int dir_index = 0, opposite_dir = 3; dir_index < 3; ++dir_index, ++opposite_dir)
            {
                if (_actual_max_force[dir_index] < 1.0f || _actual_max_force[opposite_dir] < 1.0f)
                {
                    cur_direction = _controlled_thrusters[  dir_index];
                    thruster_dir  = _thrust_forward_vectors[dir_index];
                    foreach (var cur_thruster_info in cur_direction.Values)
                        cur_thruster_info.reference_vector = get_reference_vector(cur_thruster_info, _grid_CoM_location, thruster_dir);
                    cur_direction = _controlled_thrusters[  opposite_dir];
                    thruster_dir  = _thrust_forward_vectors[opposite_dir];
                    foreach (var cur_thruster_info in cur_direction.Values)
                        cur_thruster_info.reference_vector = get_reference_vector(cur_thruster_info, _grid_CoM_location, thruster_dir);
                }
                else
                {
                    total_static_moment = Vector3.Zero;
                    cur_direction = _controlled_thrusters[dir_index];
                    foreach (var cur_thruster_info in cur_direction.Values)
                        total_static_moment += cur_thruster_info.actual_static_moment;
                    cur_direction = _controlled_thrusters[opposite_dir];
                    foreach (var cur_thruster_info in cur_direction.Values)
                        total_static_moment += cur_thruster_info.actual_static_moment;
                    CoT_location = total_static_moment / (_actual_max_force[dir_index] + _actual_max_force[opposite_dir]);

                    cur_direction = _controlled_thrusters[  dir_index];
                    thruster_dir  = _thrust_forward_vectors[dir_index];
                    foreach (var cur_thruster_info in cur_direction.Values)
                        cur_thruster_info.reference_vector = get_reference_vector(cur_thruster_info, CoT_location, thruster_dir);
                    cur_direction = _controlled_thrusters[  opposite_dir];
                    thruster_dir  = _thrust_forward_vectors[opposite_dir];
                    foreach (var cur_thruster_info in cur_direction.Values)
                        cur_thruster_info.reference_vector = get_reference_vector(cur_thruster_info, CoT_location, thruster_dir);
                }
            }
        }

        #endregion

        #region thruster manager

        private static thrust_dir get_nozzle_orientation(MyThrust thruster)
        {
            Vector3I dir_vector = thruster.ThrustForwardVector;

            foreach (thrust_dir cur_direction in Enum.GetValues(typeof(thrust_dir)))
            {
                if (_thrust_forward_vectors[(int) cur_direction] == dir_vector)
                    return cur_direction;
            }
            throw new ArgumentException("Thruster " + ((IMyTerminalBlock) thruster).CustomName  + " is not grid-aligned");
        }

        private void find_tandem_and_opposite_thrusters(MyThrust thruster, thruster_info examined_thruster_info)
        {
            Dictionary<Vector3I, List<thruster_info>> cur_direction = _tandem_thrusters[(int) examined_thruster_info.nozzle_direction];
            Vector3I                                  filter_vector = thruster.ThrustForwardVector, filtered_location;

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

        private void remove_thruster_from_lists(MyThrust thruster, thruster_info removed_thruster_info)
        {
            if (removed_thruster_info.opposing_thruster != null)
            {
                thruster_info first_opposing_info = removed_thruster_info.opposing_thruster, current_thruster_info;

                for (current_thruster_info  = removed_thruster_info.next_tandem_thruster;
                     current_thruster_info != removed_thruster_info;
                     current_thruster_info  = current_thruster_info.next_tandem_thruster)
                {
                    current_thruster_info.opposing_thruster = first_opposing_info;
                }

                if (first_opposing_info != null)
                {
                    thruster_info next_tandem_thruster = (removed_thruster_info.next_tandem_thruster == removed_thruster_info) ? null : removed_thruster_info.next_tandem_thruster;
                    current_thruster_info = first_opposing_info;
                    do
                    {
                        current_thruster_info.opposing_thruster = next_tandem_thruster;
                        current_thruster_info                   = current_thruster_info.next_tandem_thruster;
                    }
                    while (current_thruster_info != first_opposing_info);
                }

                removed_thruster_info.opposing_thruster = null;
            }

            removed_thruster_info.next_tandem_thruster.prev_tandem_thruster = removed_thruster_info.prev_tandem_thruster;
            removed_thruster_info.prev_tandem_thruster.next_tandem_thruster = removed_thruster_info.next_tandem_thruster;
            removed_thruster_info.next_tandem_thruster = removed_thruster_info.prev_tandem_thruster = removed_thruster_info;

            Dictionary<Vector3I, List<thruster_info>> cur_direction = _tandem_thrusters[(int) removed_thruster_info.nozzle_direction];
            Vector3I                                  filter_vector = thruster.ThrustForwardVector, filtered_location;

            filter_vector     = Vector3I.One - filter_vector / (filter_vector.X + filter_vector.Y + filter_vector.Z);
            filtered_location = (thruster.Min + thruster.Max) * filter_vector;
            List<thruster_info> tandem_thrusters = cur_direction[filtered_location];
            tandem_thrusters.Remove(removed_thruster_info);
            if (tandem_thrusters.Count == 0)
                cur_direction.Remove(filtered_location);
        }

        private void check_thruster_control_changed()
        {
            List<thruster_info> thruster_infos;
            MyThrust            cur_thruster;
            thruster_info       cur_thruster_info;
            bool                changes_made = false, contains_THR, contains_RCS, contains_STAT, use_active_control;
            int                 dir_index, opposite_dir;

            if (_calibration_in_progress)
            {
                foreach (var cur_task in _calibration_tasks)
                {
                    if (cur_task.valid && !cur_task.IsComplete)
                        return;
                }
                set_up_thrust_limits();
            }

            _thrusters_copy.Clear();
            _thrusters_copy.AddRange(_uncontrolled_thrusters.Keys);
            thruster_infos = _thruster_infos[0];
            thruster_infos.Clear();
            thruster_infos.AddRange(_uncontrolled_thrusters.Values);
            for (int index = 0; index < _thrusters_copy.Count; ++index)
            {
                cur_thruster_info = thruster_infos[index];
                cur_thruster      = _thrusters_copy[index];
                ((IMyTerminalBlock) cur_thruster).CustomName.ToUpperTo(_thruster_name);
                contains_THR  = _thruster_name.ContainsTHRTag() || DEBUG_THR_ALWAYS_ON;
                contains_RCS  = _thruster_name.ContainsRCSTag();
                contains_STAT = _thruster_name.ContainsSTATTag();
                if ((contains_THR || contains_RCS || contains_STAT) && cur_thruster_info.actual_max_force > 0.01f * cur_thruster_info.max_force && cur_thruster.IsWorking)
                {
                    enable_control(cur_thruster, cur_thruster_info);
                    cur_thruster_info.enable_rotation = cur_thruster_info.active_control_on = contains_THR || contains_RCS;
                    changes_made = true;
                }
            }

            _active_control_on = false;
            for (dir_index = 0; dir_index < 6; ++dir_index)
            {
                Dictionary<MyThrust, thruster_info> cur_direction = _controlled_thrusters[dir_index];

                _thrusters_copy.Clear();
                _thrusters_copy.AddRange(cur_direction.Keys);
                thruster_infos = _thruster_infos[dir_index];
                thruster_infos.Clear();
                thruster_infos.AddRange(cur_direction.Values);
                for (int index = 0; index < _thrusters_copy.Count; ++index)
                {
                    cur_thruster_info = thruster_infos[index];
                    cur_thruster      = _thrusters_copy[index];
                    ((IMyTerminalBlock) cur_thruster).CustomName.ToUpperTo(_thruster_name);
                    contains_THR  = _thruster_name.ContainsTHRTag() || DEBUG_THR_ALWAYS_ON;
                    contains_RCS  = _thruster_name.ContainsRCSTag();
                    contains_STAT = !contains_RCS && _thruster_name.ContainsSTATTag();
                    if (!contains_THR && !contains_RCS && !contains_STAT || cur_thruster_info.actual_max_force < 0.01f * cur_thruster_info.max_force || !cur_thruster.IsWorking)
                    {
                        disable_control(cur_thruster, cur_thruster_info);
                        changes_made = true;
                    }
                    else
                    {
                        use_active_control  = contains_THR || contains_RCS;
                        _active_control_on |= use_active_control;
                        if (cur_thruster_info.active_control_on != use_active_control)
                            cur_thruster_info.enable_rotation = cur_thruster_info.active_control_on = use_active_control;
                        cur_thruster_info.is_RCS = contains_RCS;
                        if (cur_thruster_info.enable_limit != contains_STAT)
                        {
                            cur_thruster_info.enable_limit = contains_STAT;
                            _calibration_scheduled        |= contains_STAT;
                        }
                    }
                }
            }

            if (_current_mode_is_CoT)
                update_reference_vectors_for_CoT_mode();
            else
                update_reference_vectors_for_CoM_mode();
            for (dir_index = 0, opposite_dir = 3; dir_index < 3; ++dir_index, ++opposite_dir)
            {
                _min_setting[dir_index] = _min_setting[opposite_dir] = MIN_OVERRIDE / 100.0f;
                if (_actual_max_force[dir_index] < _actual_max_force[opposite_dir])
                {
                    if (_actual_max_force[dir_index] >= 1.0f)
                        _min_setting[dir_index] *= _actual_max_force[opposite_dir] / _actual_max_force[dir_index];
                }
                else if (_actual_max_force[opposite_dir] >= 1.0f)
                    _min_setting[opposite_dir] *= _actual_max_force[dir_index] / _actual_max_force[opposite_dir];
            }

            if (changes_made)
            {
                _calibration_scheduled = true;
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

            if (_calibration_scheduled)
                perform_linear_calibration();
        }

        private void enable_control(MyThrust cur_thruster, thruster_info cur_thruster_info)
        {
            int dir_index = (int) cur_thruster_info.nozzle_direction;

            _controlled_thrusters[dir_index].Add(cur_thruster, cur_thruster_info);
            _uncontrolled_thrusters.Remove(cur_thruster);
            _max_force[dir_index] += cur_thruster_info.max_force;
            find_tandem_and_opposite_thrusters(cur_thruster, cur_thruster_info);
            cur_thruster_info.thrust_limit = 0.0f;
        }

        private void disable_control(MyThrust cur_thruster, thruster_info cur_thruster_info)
        {
            int dir_index = (int) cur_thruster_info.nozzle_direction;

            remove_thruster_from_lists(cur_thruster, cur_thruster_info);
            cur_thruster_info.thrust_limit = 0.0f;
            _max_force[dir_index]         -= cur_thruster_info.max_force;
            _uncontrolled_thrusters.Add(cur_thruster, cur_thruster_info);
            _controlled_thrusters[dir_index].Remove(cur_thruster);
        }

        private float refresh_real_max_forces_for_single_direction(Dictionary<MyThrust, thruster_info> thrusters, bool atmosphere_present, float air_density)
        {
            thruster_info      cur_thruster_info;
            float              thrust_multiplier, planetoid_influence, actual_max_force = 0.0f;
            MyThrustDefinition thruster_definition;

            foreach (var cur_thruster in thrusters)
            {
                cur_thruster_info   = cur_thruster.Value;
                thruster_definition = cur_thruster.Key.BlockDefinition;

                if (!atmosphere_present && thruster_definition.NeedsAtmosphereForInfluence)
                    planetoid_influence = 0.0f;
                else if (thruster_definition.MaxPlanetaryInfluence <= thruster_definition.MinPlanetaryInfluence)
                    planetoid_influence = 1.0f;
                else
                {
                    planetoid_influence = (air_density - thruster_definition.MinPlanetaryInfluence) / (thruster_definition.MaxPlanetaryInfluence - thruster_definition.MinPlanetaryInfluence);
                    if (planetoid_influence < 0.0f)
                        planetoid_influence = 0.0f;
                    else if (planetoid_influence > 1.0f)
                        planetoid_influence = 1.0f;
                }
                thrust_multiplier = (1.0f - planetoid_influence) * thruster_definition.EffectivenessAtMinInfluence + planetoid_influence * thruster_definition.EffectivenessAtMaxInfluence;

                cur_thruster_info.actual_max_force     = cur_thruster_info.max_force        * thrust_multiplier;
                cur_thruster_info.actual_static_moment = cur_thruster_info.static_moment    * thrust_multiplier;
                cur_thruster_info.actual_min_force     = cur_thruster_info.actual_max_force * MIN_OVERRIDE / 100.0f;
                actual_max_force += cur_thruster_info.actual_max_force;
            }
            return actual_max_force;
        }

        private void refresh_real_max_forces()
        {
            BoundingBoxD grid_bounding_box = _grid.PositionComp.WorldAABB;
            MyPlanet     closest_planetoid = MyGamePruningStructure.GetClosestPlanet(ref grid_bounding_box);
            bool         atmosphere_present;
            float        air_density;

            if (closest_planetoid == null)
            {
                atmosphere_present = false;
                air_density        = 0.0f;
            }
            else
            {
                atmosphere_present = closest_planetoid.HasAtmosphere;
                air_density        = closest_planetoid.GetAirDensity(grid_bounding_box.Center);
            }

            for (int dir_index = 0; dir_index < 6; ++dir_index)
                _actual_max_force[dir_index] = refresh_real_max_forces_for_single_direction(_controlled_thrusters[dir_index], atmosphere_present, air_density);
            refresh_real_max_forces_for_single_direction(_uncontrolled_thrusters, atmosphere_present, air_density);
        }

        public void assign_thruster(IMyThrust thruster_ref)
        {
            var thruster = (MyThrust) thruster_ref;
            if (_thrust_manager_task.valid && !_thrust_manager_task.IsComplete)
                _thrust_manager_task.Wait();
            if (MyAPIGateway.Multiplayer == null || MyAPIGateway.Multiplayer.IsServer)
                thruster.SetValueFloat("Override", 0.0f);
            var new_thruster = new thruster_info();
            new_thruster.grid_centre_pos      = (thruster.Min + thruster.Max) * (_grid.GridSize / 2.0f);
            new_thruster.max_force            = new_thruster.actual_max_force = thruster.BlockDefinition.ForceMagnitude;
            new_thruster.CoM_offset           = new_thruster.grid_centre_pos - _grid_CoM_location;
            new_thruster.max_torque           = Vector3.Cross(new_thruster.CoM_offset, -thruster.ThrustForwardVector * thruster.BlockDefinition.ForceMagnitude);
            new_thruster.static_moment        = new_thruster.actual_static_moment = new_thruster.grid_centre_pos * new_thruster.max_force;
            new_thruster.nozzle_direction     = get_nozzle_orientation(thruster);
            new_thruster.reference_vector     = get_reference_vector(new_thruster, _grid_CoM_location, _thrust_forward_vectors[(int) new_thruster.nozzle_direction]);
            new_thruster.thrust_limit         = 1.0f;
            new_thruster.enable_limit         = new_thruster.enable_rotation = new_thruster.active_control_on = false;
            new_thruster.opposing_thruster    = null;
            new_thruster.next_tandem_thruster = new_thruster.prev_tandem_thruster = new_thruster;
            new_thruster.throttle_setting     = "TTDTWM_MT_" + thruster.EntityId.ToString();
            _uncontrolled_thrusters.Add(thruster, new_thruster);
            //log_ECU_action("assign_thruster", string.Format("{0} ({1}) [{2}]\n\t\t\tCentre position: {3}",
            //    ((IMyTerminalBlock) thruster).CustomName, new_thruster.nozzle_direction.ToString(), thruster.EntityId, 
            //    new_thruster.grid_centre_pos));
        }

        public void dispose_thruster(IMyThrust thruster_ref)
        {
            var  thruster       = (MyThrust) thruster_ref;
            bool thruster_found = false;

            if (_thrust_manager_task.valid && !_thrust_manager_task.IsComplete)
                _thrust_manager_task.Wait();
            if (_uncontrolled_thrusters.ContainsKey(thruster))
            {
                thruster_found = true;
                _uncontrolled_thrusters.Remove(thruster);
                //log_ECU_action("dispose_thruster", string.Format("{0} ({1}) [{2}]", ((IMyTerminalBlock) thruster).CustomName, get_nozzle_orientation(thruster).ToString(), thruster.EntityId));
            }
            else
            {
                for (int dir_index = 0; dir_index < 6; ++dir_index)
                {
                    if (_controlled_thrusters[dir_index].ContainsKey(thruster))
                    {
                        thruster_found = _calibration_scheduled = true;
                        remove_thruster_from_lists(thruster, _controlled_thrusters[dir_index][thruster]);
                        _max_force[dir_index] -= _controlled_thrusters[dir_index][thruster].max_force;
                        _controlled_thrusters[dir_index].Remove(thruster);
                        //log_ECU_action("dispose_thruster", string.Format("{0} ({1}) [{2}]", ((IMyTerminalBlock) thruster).CustomName, get_nozzle_orientation(thruster).ToString(), thruster.EntityId));
                        break;
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

        public engine_control_unit(IMyCubeGrid grid_ref)
        {
            _grid = (MyCubeGrid) grid_ref;
            _inverse_world_transform      = _grid.PositionComp.WorldMatrixNormalizedInv;
            _inverse_world_rotation_fixed = _inverse_world_transform.GetOrientation();

            _solver_starters = new Action[6];
            _solver_starters[(int) thrust_dir.fore     ] = start_solver_fore;
            _solver_starters[(int) thrust_dir.aft      ] = start_solver_aft;
            _solver_starters[(int) thrust_dir.starboard] = start_solver_starboard;
            _solver_starters[(int) thrust_dir.port     ] = start_solver_port;
            _solver_starters[(int) thrust_dir.dorsal   ] = start_solver_dorsal;
            _solver_starters[(int) thrust_dir.ventral  ] = start_solver_ventral;
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
            _spherical_moment_of_inertia = 0.4f * ((_grid.Physics.Mass >= 1.0f) ? _grid.Physics.Mass : 1.0f) * reference_radius * reference_radius;
            //log_ECU_action("calc_spherical_moment_of_inertia", string.Format("smallest area = {0} m2, radius = {1} m, SMoI = {2} t*m2", smallest_area, reference_radius, _spherical_moment_of_inertia / 1000.0f));
        }

        private void refresh_gyro_info()
        {
            uint num_overriden_gyroscopes = 0;

            _gyro_override   = Vector3.Zero;
            _max_gyro_torque = 0.0f;
            foreach (var cur_gyroscope in _gyroscopes)
            {
                if (cur_gyroscope.IsWorking)
                {
                    _max_gyro_torque += cur_gyroscope.MaxGyroForce;
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
                reset_user_input(reset_gyros_only: true);
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

        public bool is_under_control_of(VRage.Game.ModAPI.Interfaces.IMyControllableEntity current_controller)
        {
            var    controller = current_controller as MyShipController;
            return controller != null && controller.CubeGrid == _grid;
        }

        public void check_autopilot(IMyRemoteControl RC_block)
        {
            var RC_block_proper = (MyRemoteControl) RC_block;
            autopilot_on       |= ((MyObjectBuilder_RemoteControl) RC_block_proper.GetObjectBuilderCubeBlock()).AutoPilotEnabled;
        }

        public void reset_user_input(bool reset_gyros_only)
        {
            _manual_thrust         = _manual_rotation = _target_rotation = Vector3.Zero;
            _under_player_control &= reset_gyros_only;
        }

        public void translate_linear_input(Vector3 input_thrust, VRage.Game.ModAPI.Interfaces.IMyControllableEntity current_controller)
        {
            var controller = current_controller as MyShipController;
            if (controller == null || controller.CubeGrid != _grid)
            {
                reset_user_input(reset_gyros_only: false);
                return;
            }

            Matrix cockpit_matrix;
            controller.Orientation.GetMatrix(out cockpit_matrix);
            _manual_thrust        = Vector3.Clamp(Vector3.Transform(input_thrust, cockpit_matrix), -Vector3.One, Vector3.One);
            _under_player_control = true;
        }

        public void translate_rotation_input(Vector3 input_rotation, VRage.Game.ModAPI.Interfaces.IMyControllableEntity current_controller)
        {
            var controller = current_controller as MyShipController;
            if (controller == null || controller.CubeGrid != _grid)
            {
                reset_user_input(reset_gyros_only: false);
                return;
            }

            Matrix cockpit_matrix;
            controller.Orientation.GetMatrix(out cockpit_matrix);
            _target_rotation.X = input_rotation.X * (-0.05f);
            _target_rotation.Y = input_rotation.Y * (-0.05f);
            _target_rotation.Z = input_rotation.Z * (-0.2f);
            _target_rotation   = Vector3.Transform(_target_rotation, cockpit_matrix);
            _under_player_control = true;
        }

        #endregion

        public void reset_ECU()
        {
            _prev_position = _prev_angular_velocity = Vector3.Zero;
            for (int dir_index = 0; dir_index < 6; ++dir_index)
                _current_trim[dir_index] = _last_trim[dir_index] = _linear_integral[dir_index] = 0.0f;
        }

        public void handle_60Hz()
        {
            //screen_text("", string.Format("Manager = {0}, exceptions = {1}, complete = {2}", _thrust_manager_task.valid, (_thrust_manager_task.Exceptions == null) ? 0 : _thrust_manager_task.Exceptions.GetLength(0), _thrust_manager_task.IsComplete), 16, controlled_only: false);
            if (_grid.Physics == null || _grid.Physics.IsStatic)
            {
                _physics_enable_delay = PHYSICS_ENABLE_DELAY;
                return;
            }

            // Suppress input noise caused by analog controls
            _sample_sum += _target_rotation - _rotation_samples[_current_index];
            _rotation_samples[_current_index] = _target_rotation;
            if (++_current_index >= NUM_ROTATION_SAMPLES)
                _current_index = 0;
            _manual_rotation = _sample_sum / NUM_ROTATION_SAMPLES;

            if (Vector3D.IsZero(_prev_position))
                _prev_position = _grid.Physics.CenterOfMassWorld;
            Vector3D current_position      = _grid.Physics.CenterOfMassWorld;
            Vector3D world_linear_velocity = (current_position - _prev_position) * MyEngineConstants.UPDATE_STEPS_PER_SECOND;
            _speed                         = (float) world_linear_velocity.Length();
            _prev_position                 = current_position;
            _inverse_world_transform       = _grid.PositionComp.WorldMatrixNormalizedInv;

            refresh_gyro_info();
            if (_thrust_manager_task.valid && !_thrust_manager_task.IsComplete)
                _thrust_manager_task.Wait();
            //check_manual_override();
            if (  autopilot_on || !_is_thrust_verride_active && !_is_gyro_override_active && _manual_rotation.LengthSquared() < 0.0001f 
                && _manual_thrust.LengthSquared() < 0.0001f && _grid.Physics.AngularVelocity.LengthSquared() < 0.0001f
                && (!linear_dampers_on || _grid.Physics.Gravity.LengthSquared() < 0.01f && _speed < 0.1f))
            {
                handle_thrust_control(world_linear_velocity, sleep_mode_on: true);
                if (autopilot_on)
                    calculate_and_apply_torque();
            }
            else
            {
                handle_thrust_control(world_linear_velocity, sleep_mode_on: false);
                calculate_and_apply_torque();
            }
        }

        public void handle_4Hz()
        {
            if (_grid.Physics == null || _grid.Physics.IsStatic)
            {
                reset_ECU();
                return;
            }

            var  current_grid_CoM = Vector3D.Transform(_grid.Physics.CenterOfMassWorld, _inverse_world_transform);
            bool CoM_shifted      = (current_grid_CoM - _grid_CoM_location).LengthSquared() > 0.01f;
            if (_thrust_manager_task.valid && !_thrust_manager_task.IsComplete)
                _thrust_manager_task.Wait();
            if (CoM_shifted)
            {
                _grid_CoM_location = current_grid_CoM;
                refresh_thruster_info();
                //log_ECU_action("handle_4Hz", "CoM refreshed");
            }
            if (CoM_shifted || _current_mode_is_CoT != _new_mode_is_CoT)
            {
                if (_new_mode_is_CoT)
                    update_reference_vectors_for_CoT_mode();
                else
                    update_reference_vectors_for_CoM_mode();
                _current_mode_is_CoT = _new_mode_is_CoT;
            }
            refresh_real_max_forces();
            calc_spherical_moment_of_inertia();
            check_manual_override();
        }

        private void start_2s_manager_thread()
        {
            try
            {
                check_thruster_control_changed();
            }
            catch (Exception exp)
            {
                MyLog.Default.WriteLine(string.Format("TTDTWM\t\"{0}\" {1}(): {2}\n{3}", _grid.DisplayName, exp.Message, exp.TargetSite, exp.StackTrace));
                throw;
            }
        }

        public void handle_2s_period()
        {
            if (_grid.Physics == null || _grid.Physics.IsStatic)
                return;

            if (!_thrust_manager_task.valid || _thrust_manager_task.IsComplete)
            {
                thruster_info cur_thrust_info;

                foreach (var cur_thruster in _uncontrolled_thrusters)
                {
                    cur_thrust_info = cur_thruster.Value;

                    if (cur_thrust_info.enable_rotation || cur_thrust_info.enable_limit)
                    {
                        if (MyAPIGateway.Multiplayer == null || MyAPIGateway.Multiplayer.IsServer)
                            cur_thruster.Key.SetValueFloat("Override", 0.0f);
                        cur_thrust_info.enable_limit = cur_thrust_info.enable_rotation = cur_thrust_info.active_control_on = cur_thrust_info.is_RCS = false;
                    }
                }
                _thrust_manager_task = MyAPIGateway.Parallel.Start(start_2s_manager_thread);
                //_thrust_manager_task.Wait();
            }

            //_inverse_world_rotation_fixed = _inverse_world_transform.GetOrientation();
            _force_override_refresh = true;
            _prev_rotation          = _manual_rotation;
        }
    }
}
