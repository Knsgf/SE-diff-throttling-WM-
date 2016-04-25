using ParallelTasks;
using System;
using System.Collections.Generic;
using System.Text;

using Sandbox.Common.ObjectBuilders;
using Sandbox.Game.Entities;
using Sandbox.ModAPI;
using Sandbox.ModAPI.Interfaces;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.ModAPI;
using VRage.Utils;
using VRageMath;

using PB = Sandbox.ModAPI.Ingame;

namespace ttdtwm
{
    class engine_control_unit
    {
        #region fields

        const int   NUM_ROTATION_SAMPLES = 6;
        const float DESCENDING_SPEED     = 0.5f;
        const bool  DEBUG_THR_ALWAYS_ON  = false;

        enum thrust_dir { fore = 0, aft = 3, starboard = 1, port = 4, dorsal = 2, ventral = 5 };
        class thruster_info     // Technically a struct
        {
            public float         max_force, actual_max_force;
            public Vector3       max_torque, grid_centre_pos, static_moment, CoM_offset, reference_vector, leverage_vector;
            public thrust_dir    nozzle_direction;
            public float         current_setting, thrust_limit;
            public int           prev_setting;
            public bool          enable_limit, enable_rotation, active_control_on, is_RCS, is_neutral, skip, is_reduced;
            public thruster_info next_tandem_thruster, prev_tandem_thruster, opposing_thruster;
        };

        private /*static*/ StringBuilder  _group_name       = new StringBuilder();
        private /*static*/ StringBuilder  _thruster_name    = new StringBuilder();
        private     static StringBuilder  __controller_name = new StringBuilder();
        private /*static*/ simplex_solver _linear_solver    = new simplex_solver();

        private /*static*/ List<     MyThrust> _thrusters_copy = new List<     MyThrust>();
        private /*static*/ List<thruster_info> _thruster_infos = new List<thruster_info>();

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
        private static float[] __new_active_leverage  = new float[6];

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
        private Dictionary<MyThrust, thruster_info> _uncontrolled_thrusters = new Dictionary<MyThrust, thruster_info>();
        private float[] _max_force        = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

        private HashSet<MyGyro> _gyroscopes = new HashSet<MyGyro>();

        private bool     _status_displayed = false, _RC_status_displayed = false, _calibration_scheduled = true;
        private Vector3D _grid_CoM_location = Vector3D.Zero, _prev_position = Vector3D.Zero;
        private MatrixD  _inverse_world_transform;
        private Matrix   _inverse_world_rotation_fixed;
        private float    _max_gyro_torque = 0.0f, _spherical_moment_of_inertia = 1.0f;
        private Task     _thrust_manager_task;

        private  bool[] _enable_integral    = { false, false, false, false, false, false };
        private  bool[] _restrict_integral  = { false, false, false, false, false, false };
        private float[] _current_trim       = new float[6];
        private float[] _last_trim          = new float[6];
        private float[] _active_leverage    = new float[6];
        private Vector3 _local_angular_velocity, _prev_angular_velocity = Vector3.Zero, _torque, _manual_rotation, _target_rotation, _gyro_override = Vector3.Zero;
        private bool    _current_mode_is_CoT = false, _new_mode_is_CoT = false, _force_CoT_mode = false, _is_gyro_override_active = false, _use_triangular_mode = false;
        private sbyte   _last_control_scheme = -1;
        private bool    _stabilisation_off = true, _all_engines_off = false, _active_control_on = false, _under_player_control = false, _was_dry_run = false;

        private  bool   _allow_extra_linear_opposition = false, _integral_cleared = false, _landing_mode_on = false, _RC_landing_mode_on = false, _is_thrust_verride_active = false;
        private  bool[] _enable_linear_integral = {  true,  true,  true,  true,  true,  true };
        private float[] _linear_integral        = {  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f };
        private float   _speed;
        private Vector3 _manual_thrust, _thrust_override = Vector3.Zero;

        private Vector3[] _rotation_samples = new Vector3[NUM_ROTATION_SAMPLES];
        private Vector3   _sample_sum       = Vector3.Zero;
        private int       _current_index    = 0;

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

        public bool landing_mode_on
        {
            get
            {
                return _landing_mode_on;
            }
            set
            {
                if (MyAPIGateway.Multiplayer != null && !MyAPIGateway.Multiplayer.IsServer)
                    _landing_mode_on = value;
            }
        }

        public bool CoT_mode_on
        {
            get
            {
                return _force_CoT_mode;
            }
            set
            {
                if (MyAPIGateway.Multiplayer != null && !MyAPIGateway.Multiplayer.IsServer)
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
                screen_info(string.Format("\"{0}\" {1}", _grid.DisplayName, message), display_time_ms, MyFontEnum.White, controlled_only);
            else
                screen_info(string.Format("engine_control_unit.{0}(): \"{1}\" {2}", method_name, _grid.DisplayName, message), display_time_ms, MyFontEnum.White, controlled_only);
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
            thruster_info cur_thruster_info;

            foreach (var cur_thruster in thrusters)
            {
                cur_thruster_info = cur_thruster.Value;
                cur_thruster_info.CoM_offset      = cur_thruster_info.grid_centre_pos - _grid_CoM_location;
                cur_thruster_info.max_torque      = Vector3.Cross(cur_thruster_info.CoM_offset, -cur_thruster.Key.ThrustForwardVector * cur_thruster.Key.BlockDefinition.ForceMagnitude);
                cur_thruster_info.leverage_vector = (cur_thruster_info.max_force > 1.0f) ? (cur_thruster_info.max_torque / cur_thruster_info.max_force) : Vector3.Zero;
            }
        }

        private void refresh_thruster_info()
        {
            refresh_thruster_info_for_single_direction(_uncontrolled_thrusters);
            for (int dir_index = 0; dir_index < 6; ++dir_index)
                refresh_thruster_info_for_single_direction(_controlled_thrusters[dir_index]);
            _calibration_scheduled = true;
            _use_triangular_mode   = false;
        }

        private void check_override_on_uncontrolled()
        {
            thruster_info thrust_info;
            float         thrust_override_value;

            _is_thrust_verride_active = false;
            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                __uncontrolled_override_checked[dir_index] = false;
                __thrust_override_vector[       dir_index] = 0.0f;
            }
            foreach (var cur_thruster in _uncontrolled_thrusters)
            {
                thrust_info = cur_thruster.Value;
                if (!__uncontrolled_override_checked[(int) thrust_info.nozzle_direction] && cur_thruster.Key.IsWorking)
                {
                    thrust_override_value = cur_thruster.Key.CurrentStrength;
                    if (thrust_override_value > 0.01f)
                        __thrust_override_vector[(int) thrust_info.nozzle_direction] = thrust_override_value;
                    __uncontrolled_override_checked[(int) thrust_info.nozzle_direction] = _is_thrust_verride_active = true;
                }
            }
            recompose_vector(__thrust_override_vector, out _thrust_override);
        }

        private void calculate_and_apply_torque(Vector3 desired_angular_velocity)
        {
            //if (MyAPIGateway.Multiplayer != null && !MyAPIGateway.Multiplayer.IsServer)
            //    return;

            _torque = Vector3.Zero;
            foreach (var cur_direction in _controlled_thrusters)
            {
                foreach (var cur_thruster in cur_direction)
                {
                    if (cur_thruster.Key.IsWorking)
                        _torque += cur_thruster.Value.max_torque * cur_thruster.Key.CurrentStrength;
                }
            }

            foreach (var cur_thruster in _uncontrolled_thrusters)
            {
                if (cur_thruster.Key.IsWorking)
                    _torque += cur_thruster.Value.max_torque * cur_thruster.Key.CurrentStrength;
            }

            //_are_gyroscopes_saturated = _max_gyro_torque < 1.0f || torque.LengthSquared() / _max_gyro_torque_squared >= 0.75f * 0.75f;
            if (!_stabilisation_off && _active_control_on && !autopilot_on && _max_gyro_torque >= 1.0f && _manual_rotation.LengthSquared() <= 0.0001f)
            {
                //float gyro_limit = _gyro_control.ResourceSink.SuppliedRatio * (_max_gyro_torque - _gyro_control.Torque.Length());
                float   inverse_angular_acceleration = _spherical_moment_of_inertia / _max_gyro_torque;
                float   gyro_load                    = _local_angular_velocity.Length() * inverse_angular_acceleration;
                if (gyro_load > 1.0f)
                    gyro_load = 1.0f;
                Vector3 gyro_torque_dir = /*10.0f **/ inverse_angular_acceleration * (desired_angular_velocity - _local_angular_velocity);
                if (gyro_torque_dir.LengthSquared() > 1.0f)
                    gyro_torque_dir.Normalize();
                //gyro_torque     *= gyro_limit;
                _torque += gyro_torque_dir * _max_gyro_torque * (1.0f - gyro_load);
                //screen_text("", (gyro_torque * _max_gyro_torque).ToString(), 16, controlled_only: false);
            }
            Vector3 world_torque = Vector3.Transform(_torque, _grid.WorldMatrix.GetOrientation());
            _grid.Physics.AddForce(MyPhysicsForceType.APPLY_WORLD_IMPULSE_AND_WORLD_ANGULAR_IMPULSE, Vector3.Zero, null, world_torque);
        }

        #endregion

        #region thrust calibration

        private void perform_linear_calibration()
        {
            bool  is_solution_good;
            float x = 0.0f, y = 0.0f;

            foreach (thrust_dir cur_direction in Enum.GetValues(typeof(thrust_dir)))
            {
                _thruster_infos.Clear();
                _thruster_infos.AddRange(_controlled_thrusters[(int) cur_direction].Values);

                for (int index = 0; index < _thruster_infos.Count; ++index)
                {
                    switch (cur_direction)
                    {
                        case thrust_dir.fore:
                        case thrust_dir.aft:
                            x = _thruster_infos[index].CoM_offset.X;
                            y = _thruster_infos[index].CoM_offset.Y;
                            break;

                        case thrust_dir.starboard:
                        case thrust_dir.port:
                            x = _thruster_infos[index].CoM_offset.Y;
                            y = _thruster_infos[index].CoM_offset.Z;
                            break;

                        case thrust_dir.dorsal:
                        case thrust_dir.ventral:
                            x = _thruster_infos[index].CoM_offset.X;
                            y = _thruster_infos[index].CoM_offset.Z;
                            break;
                    }
                    if (index >= _linear_solver.items.Count)
                        _linear_solver.items.Add(new solver_entry());
                    _linear_solver.items[index].x = x;
                    _linear_solver.items[index].y = y;
                    _linear_solver.items[index].max_value = _thruster_infos[index].max_force;
                }

                is_solution_good = _linear_solver.calculate_solution(_thruster_infos.Count);
                for (int index = 0; index < _thruster_infos.Count; ++index)
                {
                    if (!is_solution_good)
                    {
                        _thruster_infos[index].thrust_limit    = 0.0f;
                        _thruster_infos[index].enable_rotation = true;
                    }
                    else
                    {
                        _thruster_infos[index].thrust_limit = (_linear_solver.items[index].max_value > 0.0f)
                            ? (_linear_solver.items[index].result / _linear_solver.items[index].max_value) : 1.0f;
                        _thruster_infos[index].enable_rotation = _thruster_infos[index].active_control_on;
                    }
                    //log_ECU_action("perform_linear_calibration", string.Format("{0} kN ({1})", __linear_solver.items[index].result / 1000.0f, cur_direction));
                }

                /*
                log_ECU_action("perform_linear_calibration", is_solution_good
                    ? string.Format("successfully calibrated {0} thrusters on {1} side", __thruster_infos.Count, cur_direction)
                    : string.Format("calibration on {0} side failed", cur_direction));
                */
            }

            _calibration_scheduled = false;
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
            const float MIN_OVERRIDE = 1.001f;

            float         setting;
            int           setting_int;
            bool          enforce_min_override, dry_run = false;
            thruster_info cur_thruster_info;

            if (reset_all_thrusters && _all_engines_off)
                return;
            if (MyAPIGateway.Multiplayer != null)
            {
                if (MyAPIGateway.Multiplayer.IsServer)
                {
                    if (_under_player_control && (sync_helper.local_player == null || !MyAPIGateway.Multiplayer.IsServerPlayer(sync_helper.local_player.Client)))
                        dry_run = true;
                }
                else if (sync_helper.local_player == null || !is_under_control_of(sync_helper.local_controller))
                    dry_run = true;
            }

            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                if (reset_all_thrusters)
                    _current_trim[dir_index] = _last_trim[dir_index] = 0.0f;

                enforce_min_override = __control_vector[dir_index] > 0.01f && _speed > DESCENDING_SPEED || _landing_mode_on && linear_dampers_on && _speed < DESCENDING_SPEED * 0.5f;
                foreach (var cur_thruster in _controlled_thrusters[dir_index])
                {
                    cur_thruster_info = cur_thruster.Value;
                    if (reset_all_thrusters || !cur_thruster.Key.IsWorking || cur_thruster_info.actual_max_force < 1.0f)
                    {
                        if (cur_thruster_info.prev_setting != 0 || !dry_run && _was_dry_run)
                        {
                            if (!dry_run)
                                cur_thruster.Key.SetValueFloat("Override", 0.0f);
                            cur_thruster_info.current_setting = cur_thruster_info.prev_setting = 0;
                        }
                        continue;
                    }

                    setting = cur_thruster_info.current_setting * 100.0f;
                    if (enforce_min_override && setting < MIN_OVERRIDE)
                        setting = MIN_OVERRIDE;
                    setting_int = (int) Math.Ceiling(setting);
                    if (setting_int != cur_thruster_info.prev_setting || !dry_run && _was_dry_run)
                    {
                        if (!dry_run)
                            cur_thruster.Key.SetValueFloat("Override", setting);
                        cur_thruster_info.prev_setting = setting_int;
                    }
                }
            }

            _all_engines_off = reset_all_thrusters;
            _was_dry_run     = dry_run;
        }

        private sbyte initialise_linear_controls(Vector3 local_linear_velocity_vector, Vector3 local_gravity_vector)
        {
            const float DAMPING_CONSTANT = -2.0f, INTEGRAL_CONSTANT = 0.05f;

            Vector3 control = Vector3.Clamp(_manual_thrust + _thrust_override, -Vector3.One, Vector3.One);
            _allow_extra_linear_opposition = control.LengthSquared() > 0.75f * 0.75f;
            decompose_vector(control, __control_vector);
            sbyte control_scheme    = get_current_control_scheme();
            float gravity_magnitude = local_gravity_vector.Length();
            _stabilisation_off      = _is_gyro_override_active;

            if (!linear_dampers_on)
            {
                if (!_integral_cleared)
                {
                    for (int dir_index = 0; dir_index < 6; ++dir_index)
                    {
                        _enable_linear_integral[dir_index] = false;
                        _linear_integral[dir_index]        = 0.0f;
                    }
                    _integral_cleared = true;
                }
                _stabilisation_off |= gravity_magnitude > 0.1f && _speed < DESCENDING_SPEED * 0.5f && control.LengthSquared() < 0.0001f;
            }
            else
            {
                _integral_cleared      = false;
                Vector3 linear_damping = local_linear_velocity_vector * DAMPING_CONSTANT;
                if (!_landing_mode_on)
                    linear_damping -= local_gravity_vector;
                else
                {
                    float counter_thrust_limit = _speed / DESCENDING_SPEED;
                    if (counter_thrust_limit > 1.0f)
                        counter_thrust_limit = 1.0f;
                    linear_damping     -= local_gravity_vector * counter_thrust_limit;
                    _stabilisation_off |= _speed < DESCENDING_SPEED * 0.5f;
                }
                decompose_vector(linear_damping * _grid.Physics.Mass,            __braking_vector);
                decompose_vector(              -local_gravity_vector,         __local_gravity_inv);
                decompose_vector(      -local_linear_velocity_vector, __local_linear_velocity_inv);
                Array.Copy(__control_vector, __control_vector_copy, 6);

                int opposite_dir = 3;
                for (int dir_index = 0; dir_index < 6; ++dir_index)
                {
                    _enable_linear_integral[dir_index] = (__local_gravity_inv[dir_index] > 0.0f || __local_gravity_inv[opposite_dir] > 0.0f)
                        && __control_vector_copy[dir_index] < 0.01f && __control_vector_copy[opposite_dir] < 0.01f;

                    if (_controlled_thrusters[dir_index].Count == 0 || _max_force[dir_index] < 1.0f)
                        _enable_linear_integral[dir_index] = false;
                    else if (__control_vector_copy[opposite_dir] < 0.01f)
                    {
                        __control_vector[dir_index] += (__braking_vector[dir_index] + _grid.Physics.Mass * _linear_integral[dir_index]) 
                            / _max_force[dir_index];
                        if (__control_vector[dir_index] >= 1.0f)
                        {
                            __control_vector[dir_index]        = 1.0f;
                            _enable_linear_integral[dir_index] = _enable_linear_integral[opposite_dir] = false;
                        }
                        if (__control_vector[dir_index] > 0.75f)
                            _allow_extra_linear_opposition = true;
                    }

                    if (!_enable_linear_integral[dir_index])
                        _linear_integral[dir_index] = 0.0f;
                    else if (_linear_integral[dir_index] > 0.0f || _linear_integral[opposite_dir] == 0.0f)
                    {
                        float gravity_ratio          = (__local_gravity_inv[dir_index] + __local_gravity_inv[opposite_dir]) / gravity_magnitude,
                              linear_integral_change = INTEGRAL_CONSTANT * (__local_linear_velocity_inv[dir_index] - __local_linear_velocity_inv[opposite_dir]);
                        if (linear_integral_change > INTEGRAL_CONSTANT)
                            linear_integral_change = INTEGRAL_CONSTANT;
                        else if (linear_integral_change < -INTEGRAL_CONSTANT)
                            linear_integral_change = -INTEGRAL_CONSTANT;
                        if (__local_linear_velocity_inv[dir_index] < 1.0f)
                            linear_integral_change *= __local_linear_velocity_inv[dir_index] * (gravity_ratio - 1.0f) + 1.0f;
                        else
                            linear_integral_change *= gravity_ratio;
                        _linear_integral[dir_index] += linear_integral_change;
                        if (_linear_integral[dir_index] >= 0.0f)
                            _linear_integral[opposite_dir] = 0.0f;
                        else
                        {
                            _linear_integral[opposite_dir] = -_linear_integral[dir_index];
                            _linear_integral[   dir_index] = 0.0f;
                        }
                        if (_landing_mode_on)
                            _linear_integral[dir_index] -= INTEGRAL_CONSTANT * DESCENDING_SPEED * gravity_ratio;
                        if (_linear_integral[dir_index] < 0.0f)
                            _linear_integral[dir_index] = 0.0f;
                    }

                    if (++opposite_dir >= 6)
                        opposite_dir = 0;
                }
            }

            return control_scheme;
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

        private void adjust_thrust_for_steering(int cur_dir, int opposite_dir, Vector3 desired_angular_velocity)
        {
            const float DAMPING_CONSTANT = 5.0f, MIN_LINEAR_OPPOSITION = 0.1f, MAX_LINEAR_OPPOSITION = 0.5f;

            Vector3       angular_velocity_diff = desired_angular_velocity - _local_angular_velocity, total_torque = Vector3.Zero;
            float         max_linear_opposition, damping = DAMPING_CONSTANT * _grid.Physics.Mass / _max_force[cur_dir], current_limit,
                          rotation_threshold = 0.1f / damping, angular_velocity_diff_magnitude = angular_velocity_diff.Length(), leverage, total_force = 0.0f;
            float[]       linear_component = __linear_component[cur_dir];
            thruster_info cur_thruster_info;
            bool          enforce_thrust_limit = !_current_mode_is_CoT && __control_vector[opposite_dir] > 0.01f, rotational_thrusters_active = false, 
                          is_weaker_side = _max_force[cur_dir] < _max_force[opposite_dir], set_neutral_thrusters = false;
            Vector3       angular_velocity_diff_norm = (angular_velocity_diff_magnitude > 0.0f) ? Vector3.Normalize(angular_velocity_diff) : Vector3.Zero;

            if (_max_force[cur_dir] <= 0.0f)
            {
                __new_active_leverage[cur_dir] = 0.0f;
                return;
            }

            max_linear_opposition = MIN_LINEAR_OPPOSITION * __control_vector[opposite_dir];
            if (_allow_extra_linear_opposition)
                max_linear_opposition += MAX_LINEAR_OPPOSITION * (1.0f - __control_vector[opposite_dir]);
            if (_max_force[opposite_dir] < _max_force[cur_dir])
                max_linear_opposition *= _max_force[opposite_dir] / _max_force[cur_dir];
            float neutral_setting = enforce_thrust_limit ? max_linear_opposition : 1.0f;
            if (neutral_setting > __thrust_limits[cur_dir])
                neutral_setting = __thrust_limits[cur_dir];

            foreach (var cur_thruster in _controlled_thrusters[cur_dir])
            {
                cur_thruster_info = cur_thruster.Value;
                cur_thruster_info.is_neutral = cur_thruster_info.is_reduced = false;
                cur_thruster_info.skip       = !cur_thruster_info.enable_rotation || cur_thruster_info.actual_max_force < 1.0f || !cur_thruster.Key.IsWorking;
                if (cur_thruster_info.skip)
                    continue;
                cur_thruster_info.skip |= enforce_thrust_limit && cur_thruster_info.opposing_thruster != null;

                decompose_vector(Vector3.Cross(angular_velocity_diff, cur_thruster_info.reference_vector), linear_component);
                if (linear_component[cur_dir] >= rotation_threshold)
                {
                    if (cur_thruster_info.skip)
                        continue;

                    cur_thruster_info.current_setting += damping * linear_component[cur_dir];
                    if (enforce_thrust_limit && cur_thruster_info.active_control_on && !cur_thruster_info.is_RCS)
                    {
                        // Limit thrusters opposing player/ID linear input
                        if (cur_thruster_info.current_setting > max_linear_opposition)
                            cur_thruster_info.current_setting = max_linear_opposition;
                    }
                    cur_thruster_info.current_setting += 1.0f - __thrust_limits[cur_dir];
                    if (cur_thruster_info.current_setting > 1.0f)
                        cur_thruster_info.current_setting = 1.0f;

                    if (_use_triangular_mode)
                    {
                        total_torque += cur_thruster_info.current_setting * cur_thruster_info.max_torque;
                        total_force  += cur_thruster_info.current_setting * cur_thruster_info.max_force;
                    }
                    rotational_thrusters_active = true;
                }
                else if (linear_component[opposite_dir] >= rotation_threshold)
                {
                    cur_thruster_info.is_reduced       = true;
                    cur_thruster_info.current_setting -= damping * linear_component[opposite_dir];
                    current_limit = 0.5f * cur_thruster_info.thrust_limit * (1.0f - __thrust_limits[cur_dir]) + __thrust_limits[cur_dir];
                    if (cur_thruster_info.current_setting > current_limit)
                        cur_thruster_info.current_setting = current_limit;
                    if (cur_thruster_info.current_setting < 0.0f)
                        cur_thruster_info.current_setting = 0.0f;
                    else if (enforce_thrust_limit && cur_thruster_info.active_control_on && !cur_thruster_info.is_RCS && cur_thruster_info.current_setting > max_linear_opposition)
                    {
                        // Limit thrusters opposing player/ID linear input
                        cur_thruster_info.current_setting = max_linear_opposition;
                    }
                }
                else
                    cur_thruster_info.is_neutral = set_neutral_thrusters = true;
            }
            __new_active_leverage[cur_dir] = (total_force < 1.0f) ? 0.0f : Math.Abs(Vector3.Dot(angular_velocity_diff_norm, total_torque) / total_force);

            if (set_neutral_thrusters && (is_weaker_side || !rotational_thrusters_active))
            {
                _use_triangular_mode = true;
                if (_active_leverage[opposite_dir] >= _grid.GridSize)
                {
                    foreach (var cur_thruster in _controlled_thrusters[cur_dir])
                    {
                        cur_thruster_info = cur_thruster.Value;
                        if (cur_thruster_info.skip || !cur_thruster_info.is_neutral)
                            continue;

                        cur_thruster_info.current_setting += neutral_setting;
                        if (cur_thruster_info.current_setting > 1.0f)
                            cur_thruster_info.current_setting = 1.0f;
                        rotational_thrusters_active = true;
                    }
                }
            }

            if (!rotational_thrusters_active && (_use_triangular_mode || angular_velocity_diff_magnitude > 0.1f))
            {
                float max_opposite_leverage = _active_leverage[opposite_dir];

                _use_triangular_mode = true;
                foreach (var cur_thruster in _controlled_thrusters[cur_dir])
                {
                    cur_thruster_info = cur_thruster.Value;
                    if (cur_thruster_info.skip || cur_thruster_info.is_neutral)
                        continue;

                    leverage = Math.Abs(Vector3.Dot(angular_velocity_diff_norm, cur_thruster_info.leverage_vector));
                    if (leverage < max_opposite_leverage)
                    {
                        cur_thruster_info.current_setting += neutral_setting * (1.0f - leverage / max_opposite_leverage);
                        if (cur_thruster_info.current_setting > 1.0f)
                            cur_thruster_info.current_setting = 1.0f;
                    }
                }
            }
        }

        // Ensures that resulting linear force doesn't exceed player/ID input (to prevent undesired drift when turning)
        void normalise_thrust()
        {
            const float MAX_NORMALISATION = 5.0f;

            int   opposite_dir = 3;
            float new_force_ratio, linear_force = 0.0f, requested_force = 0.0f, force1, force2, max_setting = 0.0f, max_control = 0.0f;
            bool  zero_thrust_reduction = true;
            thruster_info first_opposite_trhuster, cur_opposite_thruster, cur_thruster_info;

            for (int dir_index = 0; dir_index < 3; ++dir_index)
            {
                foreach (var cur_thruster in _controlled_thrusters[dir_index])
                {
                    cur_thruster_info       = cur_thruster.Value;
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

                foreach (var cur_thruster in _controlled_thrusters[dir_index])
                {
                    cur_thruster_info = cur_thruster.Value;
                    if (max_setting < cur_thruster_info.current_setting)
                        max_setting = cur_thruster_info.current_setting;
                }
                foreach (var cur_thruster in _controlled_thrusters[opposite_dir])
                {
                    cur_thruster_info = cur_thruster.Value;
                    if (max_setting < cur_thruster_info.current_setting)
                        max_setting = cur_thruster_info.current_setting;
                }
                if (max_control < __control_vector[   dir_index])
                    max_control = __control_vector[   dir_index];
                if (max_control < __control_vector[opposite_dir])
                    max_control = __control_vector[opposite_dir];

                ++opposite_dir;
            }

            if (max_setting > 0.0f)
            {
                float max_normalisation_multiplier = 1.0f / max_setting;
                if (max_normalisation_multiplier > MAX_NORMALISATION)
                    max_normalisation_multiplier = MAX_NORMALISATION;
                max_normalisation_multiplier = 1.0f + max_control * (max_normalisation_multiplier - 1.0f);
                for (int dir_index = 0; dir_index < 6; ++dir_index)
                {
                    float reduced_normalisation_multiplier = __thrust_limits[dir_index] * max_normalisation_multiplier;
                    __actual_force[dir_index] = __non_THR_force[dir_index] = 0.0f;
                    foreach (var cur_thruster in _controlled_thrusters[dir_index])
                    {
                        cur_thruster_info = cur_thruster.Value;
                        cur_thruster_info.current_setting *= cur_thruster_info.is_reduced ? reduced_normalisation_multiplier : max_normalisation_multiplier;
                        if (!cur_thruster_info.active_control_on || cur_thruster_info.is_RCS)
                            __non_THR_force[dir_index] += cur_thruster_info.current_setting * cur_thruster_info.actual_max_force;
                        else
                            __actual_force[dir_index] += cur_thruster_info.current_setting * cur_thruster_info.actual_max_force;
                    }
                }
            }

            opposite_dir = 3;
            for (int dir_index = 0; dir_index < 3; ++dir_index)
            {
                force1 = __actual_force[   dir_index] + __non_THR_force[   dir_index];
                force2 = __actual_force[opposite_dir] + __non_THR_force[opposite_dir];
                if (__actual_force[dir_index] >= 1.0f && force1 - __requested_force[dir_index] > force2)
                {
                    new_force_ratio = (force2 + __requested_force[dir_index] - __non_THR_force[dir_index]) / __actual_force[dir_index];
                    if (new_force_ratio < 0.0f)
                        new_force_ratio = 0.0f;
                    foreach (var cur_thruster in _controlled_thrusters[dir_index])
                    {
                        cur_thruster_info = cur_thruster.Value;
                        if (cur_thruster_info.active_control_on && !cur_thruster_info.is_RCS)
                            cur_thruster_info.current_setting *= new_force_ratio;
                    }
                    __actual_force[dir_index] *= new_force_ratio;
                }
                if (__actual_force[opposite_dir] >= 1.0f && force2 - __requested_force[opposite_dir] > force1)
                {
                    new_force_ratio = (force1 + __requested_force[opposite_dir] - __non_THR_force[opposite_dir]) / __actual_force[opposite_dir];
                    if (new_force_ratio < 0.0f)
                        new_force_ratio = 0.0f;
                    foreach (var cur_thruster in _controlled_thrusters[opposite_dir])
                    {
                        cur_thruster_info = cur_thruster.Value;
                        if (cur_thruster_info.active_control_on && !cur_thruster_info.is_RCS)
                            cur_thruster_info.current_setting *= new_force_ratio;
                    }
                    __actual_force[opposite_dir] *= new_force_ratio;
                }

                if (   __control_vector[   dir_index] >= 0.3f && _max_force[   dir_index] >= 0.05f * _grid.Physics.Mass 
                    || __control_vector[opposite_dir] >= 0.3f && _max_force[opposite_dir] >= 0.05f * _grid.Physics.Mass)
                {
                    zero_thrust_reduction = false;
                    force1                = (__actual_force[dir_index] + __non_THR_force[dir_index]) - (__actual_force[opposite_dir] + __non_THR_force[opposite_dir]);
                    linear_force         += force1 * force1;
                    force2                = __requested_force[dir_index] + __requested_force[opposite_dir];
                    requested_force      += force2 * force2;
                }
                ++opposite_dir;
            }

            if (MyAPIGateway.Multiplayer == null || MyAPIGateway.Multiplayer.IsServer)
            { 
                if (zero_thrust_reduction)
                    thrust_reduction = 0;
                else
                {
                    linear_force     = (float) Math.Sqrt(   linear_force);
                    requested_force  = (float) Math.Sqrt(requested_force);
                    thrust_reduction = (  int) ((1.0f - linear_force / requested_force) * 100.0f + 0.5f);

                    // Possible due to relaxed limits on hover thrusters
                    if (thrust_reduction < 0 || requested_force < 0.05f * _grid.Physics.Mass)
                        thrust_reduction = 0;
                }
            }
        }

        private bool adjust_trim_setting(sbyte control_scheme, out Vector3 desired_angular_velocity)
        {
            const float ANGULAR_INTEGRAL_COEFF = -0.05f, ANGULAR_DERIVATIVE_COEFF = -0.01f, MAX_TRIM = 1.0f, THRUST_CUTOFF_TRIM = 0.9f;

            bool    update_inverse_world_matrix = false;
            float   trim_change, thrust_limit_pitch, thrust_limit_yaw, thrust_limit_roll;
            Vector3 local_angular_acceleration  = (_local_angular_velocity - _prev_angular_velocity) / MyEngineConstants.UPDATE_STEP_SIZE_IN_SECONDS, trim_vector;
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
                    __steering_output[dir_index] = (__angular_velocity[dir_index] < 0.1f) ? 
                          (__steering_input[dir_index] * 15.0f) 
                        : (__angular_velocity[dir_index] + __steering_input[dir_index] * 3.0f);
                    //__steering_output[dir_index]  = __steering_input[dir_index] * 3.0f + __angular_velocity[dir_index] + __angular_velocity[opposite_dir];
                    __steering_output[dir_index] += _last_trim[dir_index];
                    _enable_integral[dir_index]   = _enable_integral[opposite_dir] = false;
                    _current_trim[opposite_dir]   = 0.0f;
                    update_inverse_world_matrix   = true;
                    //if (_stabilisation_off || !_active_control_on)
                        _current_trim[dir_index] = 0.0f;
                    /*else if (__angular_velocity[opposite_dir] > 0.1f)
                        _current_trim[dir_index] -= ANGULAR_INTEGRAL_COEFF * __angular_velocity[opposite_dir];
                    else if (__angular_velocity[dir_index] < 0.1f)
                        _current_trim[dir_index] -= ANGULAR_INTEGRAL_COEFF * 0.5f * (0.1f + __angular_velocity[opposite_dir] - __angular_velocity[dir_index]);
                    _current_trim[dir_index] = (__angular_velocity[dir_index] > 0.1f) ? 0.0f : (_current_trim[dir_index] - ANGULAR_INTEGRAL_COEFF * 0.01f);*/

                }
                else
                {
                    if (_stabilisation_off || !_active_control_on)
                        _current_trim[dir_index] = _last_trim[dir_index] = 0.0f;
                    else if (_enable_integral[dir_index])
                    {
                        if (_restrict_integral[dir_index] && __angular_velocity[dir_index] < 0.001f)
                        {
                            _restrict_integral[dir_index] = false;
                            update_inverse_world_matrix   = true;
                        }

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
            __thrust_limits[(int) thrust_dir.fore     ] = __thrust_limits[(int) thrust_dir.aft    ] = (thrust_limit_pitch < thrust_limit_yaw ) ? thrust_limit_pitch : thrust_limit_yaw;
            __thrust_limits[(int) thrust_dir.starboard] = __thrust_limits[(int) thrust_dir.port   ] = (thrust_limit_yaw   < thrust_limit_roll) ? thrust_limit_yaw   : thrust_limit_roll;
            __thrust_limits[(int) thrust_dir.dorsal   ] = __thrust_limits[(int) thrust_dir.ventral] = (thrust_limit_pitch < thrust_limit_roll) ? thrust_limit_pitch : thrust_limit_roll;

            recompose_vector(__steering_output, out desired_angular_velocity);
            desired_angular_velocity += ANGULAR_DERIVATIVE_COEFF * local_angular_acceleration;
            _last_control_scheme      = control_scheme;
            return update_inverse_world_matrix;
        }

        private Vector3 handle_thrust_control(Vector3 world_linear_velocity)
        {
            Vector3 desired_angular_velocity;
            
            // Using "fixed" (it changes orientation only when the player steers a ship) inverse rotation matrix here to 
            // prevent Dutch Roll-like tendencies at high speeds
            Vector3 local_linear_velocity = Vector3.Transform(world_linear_velocity, _inverse_world_rotation_fixed);

            Matrix inverse_world_rotation = _inverse_world_transform.GetOrientation();
            Vector3 local_gravity         = Vector3.Transform(_grid.Physics.Gravity, inverse_world_rotation);
            _local_angular_velocity       = Vector3.Transform(_grid.Physics.AngularVelocity, inverse_world_rotation);
            if (_is_gyro_override_active)
                _manual_rotation = _gyro_override - _local_angular_velocity;

            sbyte control_scheme = initialise_linear_controls(local_linear_velocity, local_gravity);
            bool  update_inverse_world_matrix;
            if (!_is_gyro_override_active)
                update_inverse_world_matrix = adjust_trim_setting(control_scheme, out desired_angular_velocity);
            else
            {
                desired_angular_velocity    = _gyro_override;
                update_inverse_world_matrix = true;
            }

            // Update fixed inverse rotation matrix when angle exceeds 11 degrees or speed is low 
            // (decoupling inertia dampers' axes from ship orientation isn't needed at low velocities)
            if (update_inverse_world_matrix || _speed <= 20.0f || Vector3.Dot(_inverse_world_rotation_fixed.Forward, inverse_world_rotation.Forward) < 0.98f)
                _inverse_world_rotation_fixed = inverse_world_rotation;

            _new_mode_is_CoT                = true; 
            _allow_extra_linear_opposition |= _manual_rotation.LengthSquared() > 0.0001f || _local_angular_velocity.LengthSquared() > 0.0003f;
            //int opposite_dir                = 3;
            /*
            thruster_info cur_thruster_info;
            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                if (__control_vector[dir_index] > 0.05f)
                    _new_mode_is_CoT = _force_CoT_mode;
                __requested_force[dir_index] = 0.0f;
                foreach (var cur_thruster in _controlled_thrusters[dir_index])
                {
                    cur_thruster_info = cur_thruster.Value;
                    if (!cur_thruster.Key.IsWorking || cur_thruster_info.actual_max_force < 1.0f)
                        cur_thruster_info.current_setting = 0.0f;
                    else
                    {
                        cur_thruster_info.current_setting = __control_vector[dir_index];
                        __requested_force[dir_index]     += cur_thruster_info.current_setting * cur_thruster_info.actual_max_force;
                        if (cur_thruster_info.enable_limit)
                            cur_thruster_info.current_setting *= cur_thruster_info.thrust_limit;
                    }
                }
                //adjust_thrust_for_steering(dir_index, opposite_dir, desired_angular_velocity);
                //if (++opposite_dir >= 6)
                //    opposite_dir = 0;
            }
            */

            //int counter = 6;
            Action<int> set_up_thrusters = delegate (int dir_index)
            {
                thruster_info cur_thruster_info;

                if (__control_vector[dir_index] > 0.05f)
                    _new_mode_is_CoT = _force_CoT_mode;
                __requested_force[dir_index] = 0.0f;
                foreach (var cur_thruster in _controlled_thrusters[dir_index])
                {
                    cur_thruster_info = cur_thruster.Value;
                    if (!cur_thruster.Key.IsWorking || cur_thruster_info.actual_max_force < 1.0f)
                        cur_thruster_info.current_setting = 0.0f;
                    else
                    {
                        cur_thruster_info.current_setting = __control_vector[dir_index];
                        __requested_force[dir_index] += cur_thruster_info.current_setting * cur_thruster_info.actual_max_force;
                        if (cur_thruster_info.enable_limit && (!cur_thruster_info.enable_rotation || cur_thruster_info.active_control_on))
                            cur_thruster_info.current_setting *= cur_thruster_info.thrust_limit;
                    }
                }
                adjust_thrust_for_steering(dir_index, (dir_index < 3) ? (dir_index + 3) : (dir_index - 3), desired_angular_velocity);
                //--counter;
            };
            //if (MyAPIGateway.Parallel != null)
            MyAPIGateway.Parallel.For(0, 6, set_up_thrusters);
            /*else
            {
                for (int dir_index = 0; dir_index < 6; ++dir_index)
                    set_up_thrusters(dir_index);
            }*/

            //if (counter > 0)
            //    throw new Exception("Non-blocking execution detected");
            Array.Copy(__new_active_leverage, _active_leverage, 6);

            normalise_thrust();
            apply_thrust_settings(reset_all_thrusters: false);
            return desired_angular_velocity;
        }

        private static void set_thruster_reference_vector(thruster_info thruster, Vector3 reference)
        {
            thruster.reference_vector = reference;
        }

        private void update_reference_vectors_for_CoM_mode()
        {
            foreach (var cur_direction in _controlled_thrusters)
            {
                foreach (var cur_thruster in cur_direction)
                    set_thruster_reference_vector(cur_thruster.Value, cur_thruster.Value.CoM_offset);
            }
        }

        private void update_reference_vectors_for_CoT_mode()
        {
            Vector3 total_static_moment, CoT_location;

            for (int dir_index = 0; dir_index < 3; ++dir_index)
            {
                if (_max_force[dir_index] < 1.0f || _max_force[dir_index + 3] < 1.0f)
                {
                    foreach (var cur_thruster in _controlled_thrusters[dir_index    ])
                        set_thruster_reference_vector(cur_thruster.Value, cur_thruster.Value.CoM_offset);
                    foreach (var cur_thruster in _controlled_thrusters[dir_index + 3])
                        set_thruster_reference_vector(cur_thruster.Value, cur_thruster.Value.CoM_offset);
                }
                else
                {
                    total_static_moment = Vector3.Zero;
                    foreach (var cur_thruster in _controlled_thrusters[dir_index    ].Values)
                        total_static_moment += cur_thruster.static_moment;
                    foreach (var cur_thruster in _controlled_thrusters[dir_index + 3].Values)
                        total_static_moment += cur_thruster.static_moment;
                    CoT_location = total_static_moment / (_max_force[dir_index] + _max_force[dir_index + 3]);
                    foreach (var cur_thruster in _controlled_thrusters[dir_index    ])
                        set_thruster_reference_vector(cur_thruster.Value, cur_thruster.Value.grid_centre_pos - CoT_location);
                    foreach (var cur_thruster in _controlled_thrusters[dir_index + 3])
                        set_thruster_reference_vector(cur_thruster.Value, cur_thruster.Value.grid_centre_pos - CoT_location);
                }
            }
        }

        #endregion

        #region thruster manager

        private static thrust_dir get_nozzle_orientation(MyThrust thruster)
        {
            Vector3I dir_vector = thruster.ThrustForwardVector;
            if (dir_vector == Vector3I.Forward)
                return thrust_dir.fore;
            if (dir_vector == Vector3I.Backward)
                return thrust_dir.aft;
            if (dir_vector == Vector3I.Left)
                return thrust_dir.port;
            if (dir_vector == Vector3I.Right)
                return thrust_dir.starboard;
            if (dir_vector == Vector3I.Up)
                return thrust_dir.dorsal;
            if (dir_vector == Vector3I.Down)
                return thrust_dir.ventral;
            throw new ArgumentException("Thruster " + ((PB.IMyTerminalBlock) thruster).CustomName  + " is not grid-aligned");
        }

        private void find_tandem_and_opposite_thrusters(MyThrust thruster, thruster_info thrust_info)
        {
            Dictionary<MyThrust, thruster_info> 
                cur_direction      = _controlled_thrusters[ (int) thrust_info.nozzle_direction         ],
                opposite_direction = _controlled_thrusters[((int) thrust_info.nozzle_direction + 3) % 6];
            thruster_info cur_thruster_info, first_opposing_thruster;
            Vector3I      filter_vector = thruster.ThrustForwardVector;

            filter_vector = Vector3I.One - filter_vector / (filter_vector.X + filter_vector.Y + filter_vector.Z);
            Vector3 filtered_location = thrust_info.grid_centre_pos * filter_vector;

            foreach (var cur_thruster in cur_direction)
            {
                cur_thruster_info = cur_thruster.Value;
                if (cur_thruster_info != thrust_info && (cur_thruster_info.grid_centre_pos * filter_vector - filtered_location).LengthSquared() <= 0.01f)
                {
                    thrust_info.next_tandem_thruster = cur_thruster_info.next_tandem_thruster;
                    thrust_info.prev_tandem_thruster = cur_thruster_info;

                    cur_thruster_info.next_tandem_thruster = thrust_info.next_tandem_thruster.prev_tandem_thruster = thrust_info;
                    //log_ECU_action("find_tandem_and_opposite_thrusters", string.Format("tandem thruster found \"{0}\" [{1}] for \"{2}\" [{3}]", 
                    //    ((PB.IMyTerminalBlock) cur_thruster.Key).CustomName, cur_thruster.Key.EntityId, 
                    //    ((PB.IMyTerminalBlock)         thruster).CustomName,         thruster.EntityId));
                    break;
                }
            }

            if (thrust_info.next_tandem_thruster != thrust_info)
                thrust_info.opposing_thruster = thrust_info.next_tandem_thruster.opposing_thruster;
            else
            {
                foreach (var cur_thruster in opposite_direction)
                {
                    cur_thruster_info = cur_thruster.Value;
                    if ((cur_thruster_info.grid_centre_pos * filter_vector - filtered_location).LengthSquared() <= 0.01f)
                    {
                        thrust_info.opposing_thruster = cur_thruster_info;
                        //log_ECU_action("find_tandem_and_opposite_thrusters", string.Format("opposing thruster found \"{0}\" [{1}] for \"{2}\" [{3}]",
                        //    ((PB.IMyTerminalBlock)cur_thruster.Key).CustomName, cur_thruster.Key.EntityId,
                        //    ((PB.IMyTerminalBlock)        thruster).CustomName,         thruster.EntityId));
                        break;
                    }
                }

                first_opposing_thruster = cur_thruster_info = thrust_info.opposing_thruster;
                if (cur_thruster_info == null)
                    return;

                //int count = 0;
                do
                {
                    cur_thruster_info.opposing_thruster = thrust_info;
                    cur_thruster_info                   = cur_thruster_info.next_tandem_thruster;
                    //++count;
                }
                while (cur_thruster_info != first_opposing_thruster);
                //log_ECU_action("find_tandem_and_opposite_thrusters", (count < 2) ? "1 opposing thruster set" : (count.ToString() + " opposing tandem thrusters set"));
            }
        }

        private void remove_thruster_from_lists(thruster_info thrust_info)
        {
            thrust_info.next_tandem_thruster.prev_tandem_thruster = thrust_info.prev_tandem_thruster;
            thrust_info.prev_tandem_thruster.next_tandem_thruster = thrust_info.next_tandem_thruster;
            if (thrust_info.opposing_thruster != null)
            {
                thruster_info first_thruster_info = thrust_info.opposing_thruster, cur_thruster_info, next_tandem_thruster = (thrust_info.next_tandem_thruster == thrust_info) ? null : thrust_info.next_tandem_thruster;

                if (first_thruster_info != null)
                {
                    cur_thruster_info = first_thruster_info;
                    if (next_tandem_thruster != null)
                    {
                        next_tandem_thruster.opposing_thruster = first_thruster_info;
                        //log_ECU_action("find_tandem_and_opposite_thrusters", "reassigned opposite thruster to next tandem one on own side");
                    }

                    //int count = 0;
                    do
                    {
                        cur_thruster_info.opposing_thruster = next_tandem_thruster;
                        cur_thruster_info                   = cur_thruster_info.next_tandem_thruster;
                        //++count;
                    }
                    while (cur_thruster_info != first_thruster_info);
                    //log_ECU_action("find_tandem_and_opposite_thrusters", string.Format("reassigned {0} opposite {1}thruster{2} to {3} on opposite side", count, (count >= 2) ? "tandem " : "", (count >= 2) ? "s" : "", (next_tandem_thruster == null) ? "<none>" : "next tandem one"));
                }
                thrust_info.opposing_thruster = null;
            }
            thrust_info.next_tandem_thruster = thrust_info.prev_tandem_thruster = thrust_info;
        }

        private void check_thruster_control_changed()
        {
            MyThrust      cur_thruster;
            thruster_info cur_thrust_info;
            bool          changes_made = false, contains_THR, contains_RCS, contains_STAT, use_active_control;
            int           dir_index;

            _active_control_on = false;
            foreach (var cur_direction in _controlled_thrusters)
            {
                foreach (var cur_thruster_entry in cur_direction)
                    cur_thruster_entry.Value.skip = false;
            }
            foreach (var cur_thruster_entry in _uncontrolled_thrusters)
                cur_thruster_entry.Value.skip = false;
            List<MyObjectBuilder_BlockGroup> all_groups = ((MyObjectBuilder_CubeGrid) _grid.GetObjectBuilder()).BlockGroups;
            foreach (var cur_group in all_groups)
            {
                IMySlimBlock cur_block;

                cur_group.Name.ToUpperTo(_group_name);
                contains_THR  = _group_name.ContainsTHRTag();
                contains_RCS  = _group_name.ContainsRCSTag();
                contains_STAT = _group_name.ContainsSTATTag();
                if (contains_THR || contains_RCS || contains_STAT)
                {
                    foreach (var cur_block_location in cur_group.Blocks)
                    {
                        cur_block = _grid.GetCubeBlock(cur_block_location);
                        if (cur_block == null || cur_block.FatBlock == null)
                            continue;
                        cur_thruster = cur_block.FatBlock as MyThrust;
                        if (cur_thruster == null || !cur_thruster.IsWorking)
                            continue;
                        if (_uncontrolled_thrusters.ContainsKey(cur_thruster))
                        {
                            cur_thrust_info = _uncontrolled_thrusters[cur_thruster];
                            if (cur_thrust_info.actual_max_force < 0.01f * cur_thrust_info.max_force)
                                continue;
                            enable_control(cur_thruster, cur_thrust_info);
                            changes_made = true;
                        }
                        else
                        {
                            cur_thrust_info = null;
                            foreach (var cur_direction in _controlled_thrusters)
                            {
                                if (cur_direction.ContainsKey(cur_thruster))
                                {
                                    cur_thrust_info = cur_direction[cur_thruster];
                                    break;
                                }
                            }
                            if (cur_thrust_info == null || cur_thrust_info.actual_max_force < 0.01f * cur_thrust_info.max_force)
                                continue;
                        }

                        use_active_control  = contains_THR || contains_RCS;
                        _active_control_on |= use_active_control;
                        if (cur_thrust_info.active_control_on != use_active_control)
                            cur_thrust_info.enable_rotation = cur_thrust_info.active_control_on |= use_active_control;
                        cur_thrust_info.is_RCS |= contains_RCS;
                        if (cur_thrust_info.enable_limit != contains_STAT)
                        {
                            cur_thrust_info.enable_limit |= contains_STAT;
                            _calibration_scheduled       |= contains_STAT;
                        }
                        cur_thrust_info.skip = true;
                    }
                }
            }

            for (dir_index = 0; dir_index < 6; ++dir_index)
            {
                Dictionary<MyThrust, thruster_info> cur_direction = _controlled_thrusters[dir_index];

                _thrusters_copy.Clear();
                _thrusters_copy.AddRange(cur_direction.Keys);
                _thruster_infos.Clear();
                _thruster_infos.AddRange(cur_direction.Values);
                for (int index = 0; index < _thrusters_copy.Count; ++index)
                {
                    cur_thrust_info = _thruster_infos[index];
                    if (cur_thrust_info.skip)
                        continue;
                    cur_thruster = _thrusters_copy[index];
                    ((PB.IMyTerminalBlock) cur_thruster).CustomName.ToUpperTo(_thruster_name);
                    contains_THR  = _thruster_name.ContainsTHRTag() || DEBUG_THR_ALWAYS_ON;
                    contains_RCS  = _thruster_name.ContainsRCSTag();
                    contains_STAT = _thruster_name.ContainsSTATTag();
                    if (!contains_THR && !contains_RCS && !contains_STAT || !cur_thruster.IsWorking || cur_thrust_info.actual_max_force < 0.01f * cur_thrust_info.max_force)
                    {
                        disable_control(cur_thruster, cur_thrust_info);
                        changes_made = true;
                    }
                    else
                    {
                        use_active_control  = contains_THR || contains_RCS;
                        _active_control_on |= use_active_control;
                        if (cur_thrust_info.active_control_on != use_active_control)
                            cur_thrust_info.enable_rotation = cur_thrust_info.active_control_on = use_active_control;
                        cur_thrust_info.is_RCS = contains_RCS;
                        if (cur_thrust_info.enable_limit != contains_STAT)
                        {
                            cur_thrust_info.enable_limit    = contains_STAT;
                            _calibration_scheduled         |= contains_STAT;
                        }
                    }
                }
            }

            _thrusters_copy.Clear();
            _thrusters_copy.AddRange(_uncontrolled_thrusters.Keys);
            _thruster_infos.Clear();
            _thruster_infos.AddRange(_uncontrolled_thrusters.Values);
            for (int index = 0; index < _thrusters_copy.Count; ++index)
            {
                cur_thrust_info = _thruster_infos[index];
                if (cur_thrust_info.skip)
                    continue;
                cur_thruster = _thrusters_copy[index];
                ((PB.IMyTerminalBlock) cur_thruster).CustomName.ToUpperTo(_thruster_name);
                contains_THR  = _thruster_name.ContainsTHRTag() || DEBUG_THR_ALWAYS_ON;
                contains_RCS  = _thruster_name.ContainsRCSTag();
                contains_STAT = _thruster_name.ContainsSTATTag();
                if ((contains_THR || contains_RCS || contains_STAT) && cur_thruster.IsWorking  && cur_thrust_info.actual_max_force > 0.01f * cur_thrust_info.max_force)
                {
                    enable_control(cur_thruster, cur_thrust_info);
                    cur_thrust_info.enable_rotation = cur_thrust_info.active_control_on = contains_THR || contains_RCS;
                    changes_made = true;
                }
            }

            if (changes_made)
            {
                if (_current_mode_is_CoT)
                    update_reference_vectors_for_CoT_mode();
                else
                    update_reference_vectors_for_CoM_mode();
                _calibration_scheduled = true;
                _use_triangular_mode   = false;
                log_ECU_action("check_thruster_control_changed", string.Format("{0}/{1}/{2}/{3}/{4}/{5} kN",
                    _max_force[(int) thrust_dir.fore     ] / 1000.0f,
                    _max_force[(int) thrust_dir.aft      ] / 1000.0f,
                    _max_force[(int) thrust_dir.starboard] / 1000.0f,
                    _max_force[(int) thrust_dir.port     ] / 1000.0f,
                    _max_force[(int) thrust_dir.dorsal   ] / 1000.0f,
                    _max_force[(int) thrust_dir.ventral  ] / 1000.0f));
            }

            if (_calibration_scheduled)
                perform_linear_calibration();
        }

        private void enable_control(MyThrust cur_thruster, thruster_info cur_thrust_info)
        {
            int dir_index = (int) cur_thrust_info.nozzle_direction;

            _controlled_thrusters[dir_index].Add(cur_thruster, cur_thrust_info);
            _uncontrolled_thrusters.Remove(cur_thruster);
            _max_force[dir_index] += cur_thrust_info.max_force;
            find_tandem_and_opposite_thrusters(cur_thruster, cur_thrust_info);
        }

        private void disable_control(MyThrust cur_thruster, thruster_info cur_thrust_info)
        {
            int dir_index = (int) cur_thrust_info.nozzle_direction;

            if (MyAPIGateway.Multiplayer == null || MyAPIGateway.Multiplayer.IsServer)
                cur_thruster.SetValueFloat("Override", 0.0f);
            remove_thruster_from_lists(cur_thrust_info);
            cur_thrust_info.thrust_limit = 1.0f;
            cur_thrust_info.enable_limit = cur_thrust_info.enable_rotation = cur_thrust_info.active_control_on = false;
            _max_force[dir_index]       -= cur_thrust_info.max_force;
            _uncontrolled_thrusters.Add(cur_thruster, cur_thrust_info);
            _controlled_thrusters[dir_index].Remove(cur_thruster);
        }

        private void refresh_real_max_forces_for_single_direction(Dictionary<MyThrust, thruster_info> thruster_array)
        {
            thruster_info cur_thruster_info;
            float         thrust_multiplier;

            foreach (var cur_thruster in thruster_array)
            {
                cur_thruster_info = cur_thruster.Value;
                thrust_multiplier = ((IMyThrust) cur_thruster.Key).ThrustMultiplier;
                cur_thruster_info.actual_max_force = cur_thruster_info.max_force  * thrust_multiplier;
            }
        }

        private void refresh_real_max_forces()
        {
            foreach (var cur_direction in _controlled_thrusters)
                refresh_real_max_forces_for_single_direction(cur_direction);
            refresh_real_max_forces_for_single_direction(_uncontrolled_thrusters);
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
            new_thruster.static_moment        = new_thruster.grid_centre_pos * new_thruster.max_force;
            new_thruster.nozzle_direction     = get_nozzle_orientation(thruster);
            new_thruster.thrust_limit         = 1.0f;
            new_thruster.enable_limit         = new_thruster.enable_rotation = new_thruster.active_control_on = false;
            new_thruster.opposing_thruster    = null;
            new_thruster.next_tandem_thruster = new_thruster.prev_tandem_thruster = new_thruster;
            _uncontrolled_thrusters.Add(thruster, new_thruster);
            //log_ECU_action("assign_thruster", string.Format("{0} ({1}) [{2}]\n\t\t\tCentre position: {3}",
            //    ((PB.IMyTerminalBlock) thruster).CustomName, new_thruster.nozzle_direction.ToString(), thruster.EntityId, 
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
                //log_ECU_action("dispose_thruster", string.Format("{0} ({1}) [{2}]", ((PB.IMyTerminalBlock) thruster).CustomName, get_nozzle_orientation(thruster).ToString(), thruster.EntityId));
            }
            else
            {
                for (int dir_index = 0; dir_index < 6; ++dir_index)
                {
                    if (_controlled_thrusters[dir_index].ContainsKey(thruster))
                    {
                        thruster_found = _calibration_scheduled = true;
                        remove_thruster_from_lists(_controlled_thrusters[dir_index][thruster]);
                        _max_force[dir_index] -= _controlled_thrusters[dir_index][thruster].max_force;
                        _controlled_thrusters[dir_index].Remove(thruster);
                        //log_ECU_action("dispose_thruster", string.Format("{0} ({1}) [{2}]", ((PB.IMyTerminalBlock) thruster).CustomName, get_nozzle_orientation(thruster).ToString(), thruster.EntityId));
                        break;
                    }
                }
            }
        }

        public engine_control_unit(IMyCubeGrid grid_ref)
        {
            _grid = (MyCubeGrid) grid_ref;
            _inverse_world_transform      = _grid.PositionComp.WorldMatrixNormalizedInv;
            _inverse_world_rotation_fixed = _inverse_world_transform.GetOrientation();
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

        public bool is_under_control_of(VRage.Game.ModAPI.Interfaces.IMyControllableEntity current_controller)
        {
            var    controller = current_controller as MyShipController;
            return controller != null && controller.CubeGrid == _grid;
        }

        public void select_flight_modes(VRage.Game.ModAPI.Interfaces.IMyControllableEntity current_controller, bool RC_block_present)
        {
            if (MyAPIGateway.Multiplayer != null && !MyAPIGateway.Multiplayer.IsServer)
                return;

            if (current_controller != null)
            {
                ((PB.IMyTerminalBlock) current_controller).CustomName.ToUpperTo(__controller_name);
                _force_CoT_mode = __controller_name.ContainsCOTTag();
            }

            if (_speed >= DESCENDING_SPEED * 10.0f || _grid.HasMainCockpit() || _grid.Physics == null || _grid.Physics.Gravity.LengthSquared() < 0.0001f)
                _landing_mode_on = _RC_landing_mode_on = false;
            else if (current_controller is PB.IMyRemoteControl)
                _landing_mode_on = _RC_landing_mode_on = __controller_name.ContainsLANDINGTag();
            else
                _landing_mode_on = !RC_block_present || _RC_landing_mode_on;
       }

        public void check_autopilot(PB.IMyRemoteControl RC_block)
        {
            var RC_block_proper = (MyRemoteControl) RC_block;
            autopilot_on       |= ((MyObjectBuilder_RemoteControl) RC_block_proper.GetObjectBuilderCubeBlock()).AutoPilotEnabled;
        }

        public void reset_user_input()
        {
            _manual_thrust        = _manual_rotation = _target_rotation = Vector3.Zero;
            _under_player_control = false;
        }

        public void translate_linear_input(Vector3 input_thrust, VRage.Game.ModAPI.Interfaces.IMyControllableEntity current_controller)
        {
            var controller = current_controller as MyShipController;
            if (controller == null || controller.CubeGrid != _grid)
            {
                reset_user_input();
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
                reset_user_input();
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
            if (_grid.Physics == null)
                return;

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
            check_override_on_uncontrolled();
            if (  autopilot_on || !_is_thrust_verride_active && !_is_gyro_override_active && _manual_rotation.LengthSquared() < 0.0001f 
                && _manual_thrust.LengthSquared() < 0.0001f && _grid.Physics.AngularVelocity.LengthSquared() < 1.0E-6f && _grid.Physics.Gravity.LengthSquared() < 0.01f
                && (!linear_dampers_on || _speed < 0.01f))
            {
                thrust_reduction = 0;
                apply_thrust_settings(reset_all_thrusters: true);
                if (autopilot_on)
                    calculate_and_apply_torque(Vector3.Zero);
            }
            else
            {
                Vector3 desired_angular_velocity = handle_thrust_control(world_linear_velocity);
                calculate_and_apply_torque(desired_angular_velocity);
            }
        }

        public void handle_4Hz()
        {
            if (_grid.Physics == null)
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
            if (_grid.Physics == null || _thrust_manager_task.valid && !_thrust_manager_task.IsComplete)
                return;

            _thrust_manager_task          = MyAPIGateway.Parallel.StartBackground(start_2s_manager_thread);
            _inverse_world_rotation_fixed = _inverse_world_transform.GetOrientation();
        }
    }
}
