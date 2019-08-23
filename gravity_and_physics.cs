using System;
using System.Collections.Generic;
using System.Text;

using Sandbox.Game.Entities;
using Sandbox.ModAPI;
using VRage.Game;
using VRage.Game.ModAPI;
using VRage.Game.Components;
using VRageMath;

namespace ttdtwm
{
    class orbit_elements
    {
        // Primary
        public string   name;
        public bool     foreign_reference;
        public Vector3D radius_vector, specific_angular_momentum, LAN_vector, eccentricity_vector;
        public double   semi_major_axis, eccentricity;
        public double   inclination, longitude_of_ascending_node;
        public double   argument_of_periapsis, true_anomaly;

        // Derived
        public double orbit_period;
        public double periapsis_radius, apoapsis_radius;
        public double mean_anomaly, time_from_periapsis;
        public double circular_speed, escape_speed, local_gravity_magnitude;
    }

    class orbit_plane_intersection
    {
        public Vector3D target_angular_momentum, ascending_node_vector;
        public double   relative_inclination, time_to_ascending_node, time_to_descending_node;
    }

    interface gravity_simulation: IDisposable
    {
        orbit_elements           current_elements_reader();
        orbit_plane_intersection plane_alignment_reader();

        void simulate_gravity_and_torque();
        void mark_elements_for_refresh();
    }

    interface torque_and_orbit_control
    {
        void     apply_torque(Vector3 absolute_torque);
        void     get_linear_and_angular_velocities(out Vector3D world_linear_velocity, out Vector3D world_angular_velocity);
        Vector3D get_circular_orbit_velocity(bool refresh_plane);
        Vector3  get_maneuvre_direction(engine_control_unit.ID_maneuvres selection);
    }

    class gravity_and_physics: gravity_simulation, torque_and_orbit_control
    {
        const float REFERENCE_GRAVITY = 9.81f;
        const bool  GRAVITY_ON        = true;

        class gravity_source
        {
            public string   name;
            public double   surface_gravity, mean_radius2, standard_gravitational_parameter;
            public Vector3D centre_position;
        }

        private static Dictionary< MyPlanet  , gravity_source     > _gravity_sources = new Dictionary< MyPlanet  , gravity_source     >();
        private static Dictionary<IMyCubeGrid, gravity_and_physics> _grid_list       = new Dictionary<IMyCubeGrid, gravity_and_physics>();

        private MyCubeGrid     _grid;
        private gravity_source _current_reference, _display_reference = null;

        private Vector3D _grid_position, _absolute_linear_velocity, _absolute_angular_velocity, _accumulated_gravity = Vector3D.Zero;
        private Vector3D _eccentricity_vector, _current_angular_momentum;
        private Vector3D _grid_forward, _grid_right, _grid_up;

        private Vector3 _current_torque = Vector3.Zero;

        private orbit_elements           _current_elements = new orbit_elements();
        private orbit_plane_intersection _alignment_info   = new orbit_plane_intersection();
        private bool                     _refresh_elements = true;
        private Vector3D[]               _vector_elements  = new Vector3D[ 4];
        private   double[]               _scalar_elements  = new   double[14];

        public static bool world_has_gravity
        {
            get
            {
                return _gravity_sources.Count > 0;
            }
        }

        #region Auxiliaries

        public orbit_elements current_elements_reader()
        {
            if (_refresh_elements)
                calculate_elements(false);
            return _current_elements;
        }

        public orbit_plane_intersection plane_alignment_reader()
        {
            if (_refresh_elements || _display_reference != null && _display_reference != _current_reference)
            {
                calculate_elements(true);
                _refresh_elements |= _display_reference != null && _display_reference != _current_reference;
            }
            return _alignment_info;
        }


        public static void register_gravity_source(MyPlanet new_planetoid)
        {
            gravity_source new_gravity_source = new gravity_source();

            new_gravity_source.name                             = new_planetoid.Name;
            new_gravity_source.mean_radius2                     = new_planetoid.AverageRadius * new_planetoid.AverageRadius;
            new_gravity_source.surface_gravity                  = REFERENCE_GRAVITY * new_planetoid.GetInitArguments.SurfaceGravity;
            new_gravity_source.standard_gravitational_parameter = new_gravity_source.surface_gravity * new_gravity_source.mean_radius2;
            new_gravity_source.centre_position                  = new_planetoid.PositionComp.WorldAABB.Center;
            _gravity_sources[new_planetoid] = new_gravity_source;
        }

        public static void deregister_gravity_source(MyPlanet removed_planetoid)
        {
            if (_gravity_sources.ContainsKey(removed_planetoid))
                _gravity_sources.Remove(removed_planetoid);
        }
        
        private static double acosh(double x)
        {
            if (double.IsPositiveInfinity(x))
                return double.PositiveInfinity;
            if (x < 1.0 || double.IsNaN(x))
                return double.NaN;

            return Math.Log(x + Math.Sqrt(x * x - 1.0));
        }

        private static double floor_mod(double dividend, double divisor)
        {
            return dividend - divisor * Math.Floor(dividend / divisor);
        }

        private static gravity_source get_reference_body(Vector3D grid_position, out Vector3D grid_vector, out double distance, out double gravity_magnitude)
        {
            double         gravity, distance2;
            gravity_source reference_body = null;
            Vector3D       cur_body_vector;

            gravity_magnitude = distance = 0.0;
            grid_vector       = Vector3D.Zero;
            foreach (var cur_body in _gravity_sources.Values)
            {
                cur_body_vector = grid_position - cur_body.centre_position;
                distance2       = cur_body_vector.LengthSquared();
                if (distance2 <= cur_body.mean_radius2)
                {
                    gravity_magnitude = cur_body.surface_gravity * Math.Sqrt(distance2 / cur_body.mean_radius2);
                    distance          = Math.Sqrt(distance2);
                    grid_vector       = cur_body_vector;
                    return cur_body;
                }

                gravity = cur_body.standard_gravitational_parameter / distance2;
                if (gravity_magnitude < gravity)
                {
                    gravity_magnitude = gravity;
                    reference_body    = cur_body;
                    distance          = Math.Sqrt(distance2);
                    grid_vector       = cur_body_vector;
                }
            }
            return reference_body;
        }

        public void mark_elements_for_refresh()
        {
            _refresh_elements = true;
        }

        public void Dispose()
        {
            if (_grid_list.ContainsKey(_grid))
                _grid_list.Remove(_grid);
        }

        #endregion

        #region Orbit calculation

        private void zero_all_elements()
        {
            if (_current_elements.name != null)
                _current_elements = new orbit_elements();
        }

        private void calculate_anomalies(Vector3D grid_vector, Vector3D specific_angular_momentum, Vector3D LAN_vector, double eccentricity, double mean_motion2,
            out double true_anomaly, out double mean_anomaly, out double time_from_periapsis)
        {
            double grid_vector_length = grid_vector.Length();

            if (eccentricity > 0.0)
            {
                Vector3D minor_axis_vector = Vector3D.Cross(specific_angular_momentum, _eccentricity_vector);

                true_anomaly = Math.Acos(MathHelperD.Clamp(Vector3D.Dot(_eccentricity_vector, grid_vector) / (eccentricity * grid_vector_length), -1.0, 1.0));
                if (Vector3D.Dot(grid_vector, minor_axis_vector) < 0.0)
                    true_anomaly = 2.0 * Math.PI - true_anomaly;
            }
            else
            {
                double LAN_vector_length = LAN_vector.Length();

                if (LAN_vector.Length() > 0.0)
                {
                    true_anomaly = Math.Acos(MathHelperD.Clamp(Vector3D.Dot(LAN_vector, grid_vector) / (LAN_vector_length * grid_vector_length), -1.0, 1.0));
                    if (Vector3D.Dot(LAN_vector, _absolute_linear_velocity) > 0.0)
                        true_anomaly = 2.0 * Math.PI - true_anomaly;
                }
                else
                {
                    true_anomaly = Math.Acos(grid_vector.X / grid_vector_length);
                    if (_absolute_linear_velocity.X > 0.0)
                        true_anomaly = 2.0 * Math.PI - true_anomaly;
                }
            }

            double eccentric_anomaly, true_anomaly_cos = Math.Cos(true_anomaly);
            if (eccentricity < 1.0)
            {
                eccentric_anomaly = Math.Acos(MathHelperD.Clamp((eccentricity + true_anomaly_cos) / (1.0 + eccentricity * true_anomaly_cos), -1.0, 1.0));
                if (true_anomaly > Math.PI)
                    eccentric_anomaly = 2.0 * Math.PI - eccentric_anomaly;
                mean_anomaly        = eccentric_anomaly - eccentricity * Math.Sin(eccentric_anomaly);
                time_from_periapsis = mean_anomaly * Math.Sqrt(mean_motion2);
            }
            else
            {
                eccentric_anomaly = acosh((eccentricity + true_anomaly_cos) / (1.0 + eccentricity * true_anomaly_cos));
                if (double.IsInfinity(eccentric_anomaly) || double.IsNaN(eccentric_anomaly))
                    mean_anomaly = time_from_periapsis = 0.0;
                else
                {
                    if (true_anomaly > Math.PI)
                        eccentric_anomaly = -eccentric_anomaly;
                    mean_anomaly        = eccentricity * Math.Sinh(eccentric_anomaly) - eccentric_anomaly;
                    time_from_periapsis = mean_anomaly * Math.Sqrt(-mean_motion2);
                }
            }
        }

        private void calculate_elements(bool use_major_body)
        {
            gravity_source selected_reference = use_major_body ? _current_reference : _display_reference;

            _refresh_elements = false;
            if (selected_reference == null)
                selected_reference = _current_reference;
            if (selected_reference == null)
            {
                zero_all_elements();
                return;
            }

            Vector3D grid_vector        = _grid_position - selected_reference.centre_position;
            double   grid_vector_length = grid_vector.Length();
            if (grid_vector_length * grid_vector_length < selected_reference.mean_radius2)
            {
                zero_all_elements();
                return;
            }

            double   SGP                        = selected_reference.standard_gravitational_parameter;
            Vector3D specific_angular_momentum  = Vector3D.Cross(grid_vector, _absolute_linear_velocity);
            double   angular_momentum_magnitude = specific_angular_momentum.Length();
            Vector3D LAN_vector                 = new Vector3D(specific_angular_momentum.Z, 0.0, -specific_angular_momentum.X);
            double   LAN_vector_length          = LAN_vector.Length();
            double   speed                      = _absolute_linear_velocity.Length();
            double   gravity_energy             = SGP            / grid_vector_length;
            double   gravity_magnitude          = gravity_energy / grid_vector_length;
            double   circular_speed             = Math.Sqrt(      gravity_energy);
            double   escape_speed               = Math.Sqrt(2.0 * gravity_energy);
            Vector3D new_eccentricity_vector    = (((speed - circular_speed) * (speed + circular_speed)) * grid_vector - Vector3D.Dot(grid_vector, _absolute_linear_velocity) * _absolute_linear_velocity) / SGP;
            if ((new_eccentricity_vector - _eccentricity_vector).LengthSquared() >= 0.001 * 0.001)
                _eccentricity_vector = new_eccentricity_vector;

            double semi_major_axis = SGP / ((escape_speed - speed) * (escape_speed + speed));
            double eccentricity    = _eccentricity_vector.Length();
            if (semi_major_axis > 0.0)
            {
                if (eccentricity >= 1.0)
                    eccentricity = 0.9999;
            }
            else if (semi_major_axis < 0.0)
            {
                if (eccentricity <= 1.0)
                    eccentricity = 1.0001;
            }
            else
                eccentricity = 1.0;
            double periapsis_radius = semi_major_axis * (1.0 - eccentricity);
            double apoapsis_radius  = semi_major_axis * (1.0 + eccentricity);
            double mean_motion2     = semi_major_axis * semi_major_axis * semi_major_axis / SGP;
            double period           = (semi_major_axis > 0.0 && !double.IsInfinity(semi_major_axis)) ? (2.0 * Math.PI * Math.Sqrt(mean_motion2)) : 0.0;

            double inclination = (angular_momentum_magnitude > 0.0) ? Math.Acos(specific_angular_momentum.Y / angular_momentum_magnitude) : 0.0;
            double LAN, argument_of_perapsis;
            if (inclination != 0.0)
            {
                LAN                  = Math.Acos(LAN_vector.X / LAN_vector_length);
                argument_of_perapsis = Math.Acos(Vector3D.Dot(LAN_vector, _eccentricity_vector) / (LAN_vector_length * eccentricity));
                if (double.IsNaN(argument_of_perapsis))
                    argument_of_perapsis = 0.0;
            }
            else
            {
                LAN                  = 0.0;
                argument_of_perapsis = (eccentricity > 0.0) ? (Math.Acos(_eccentricity_vector.X / eccentricity)) : 0.0;
            }
            if (LAN_vector.Z > 0.0)
                LAN = 2.0 * Math.PI - LAN;
            if (_eccentricity_vector.Y < 0.0)
                argument_of_perapsis = 2.0 * Math.PI - argument_of_perapsis;

            double true_anomaly, mean_anomaly, time_from_periapsis;
            calculate_anomalies(grid_vector, specific_angular_momentum, LAN_vector, eccentricity, mean_motion2, out true_anomaly, out mean_anomaly, out time_from_periapsis);

            _current_elements.name                      = selected_reference.name;
            _current_elements.foreign_reference         = _display_reference != null && _display_reference != _current_reference;
            _current_elements.radius_vector             = grid_vector;
            _current_elements.specific_angular_momentum = specific_angular_momentum;
            _current_elements.LAN_vector                = LAN_vector;
            _current_elements.eccentricity_vector       = _eccentricity_vector;

            _current_elements.semi_major_axis             = semi_major_axis;
            _current_elements.eccentricity                = eccentricity;
            _current_elements.inclination                 = inclination;
            _current_elements.longitude_of_ascending_node = LAN;
            _current_elements.argument_of_periapsis       = argument_of_perapsis;
            _current_elements.true_anomaly                = true_anomaly;

            _current_elements.orbit_period            = period;
            _current_elements.periapsis_radius        = periapsis_radius;
            _current_elements.apoapsis_radius         = apoapsis_radius;
            _current_elements.mean_anomaly            = mean_anomaly;
            _current_elements.time_from_periapsis     = time_from_periapsis;
            _current_elements.circular_speed          = circular_speed;
            _current_elements.escape_speed            = escape_speed;
            _current_elements.local_gravity_magnitude = gravity_magnitude;

            //float v1 = (float) Math.Sqrt(gravity_energy);
            //float v1 = (float) Math.Sqrt(grid.Physics.Gravity.Length() * grid_vector_length);

            /*
            screen_info.screen_text(_grid, "", string.Format("R = {0:F1} km, g = {3:F2} m/s^2, v1 = {1:F1} m/s, v2 = {2:F1} m/s", grid_vector_length / 1000.0, v1, v1 * Math.Sqrt(2.0), gravity_magnitude), 16, controlled_only: true);
            screen_info.screen_text(_grid, "", string.Format("SMA = {0:F1} km, e = {3:F3}, PR = {1:F1} km, AR = {2:F1} km", semi_major_axis / 1000.0, periapsis_radius / 1000.0, apoapsis_radius / 1000.0, eccentricity), 16, controlled_only: true);
            screen_info.screen_text(_grid, "", string.Format("T = {0:F0} s, TA = {1:F0}, MA = {2:F0}, TtCA = {3:F0} s", period, true_anomaly * 180.0 / Math.PI, mean_anomaly * 180.0 / Math.PI, (true_anomaly > Math.PI) ?  (period - time_from_periapsis) : (period / 2.0 - time_from_periapsis)), 16, controlled_only: true);
            */
            //set_target_plane(_grid, _current_elements.inclination, _current_elements.longitude_of_ascending_node);
            //screen_info.screen_text(_grid, "", (Vector3.Cross(specific_angular_momentum, _alignment_info.target_angular_momentum).Length()).ToString() + " " + specific_angular_momentum.Length().ToString() + " " + _alignment_info.target_angular_momentum.Length().ToString(), 16, controlled_only: true);

            if ((_display_reference == null || _display_reference == _current_reference) && _alignment_info.target_angular_momentum.LengthSquared() > 0.25)
                calculate_plane_intersection(specific_angular_momentum, LAN_vector, eccentricity, mean_motion2, period, time_from_periapsis);
        }

        private void calculate_plane_intersection(Vector3D specific_angular_momentum, Vector3D LAN_vector, double eccentricity, double mean_motion2, double period, double current_time)
        {
            _alignment_info.relative_inclination = Math.Acos(MathHelperD.Clamp(Vector3D.Dot(specific_angular_momentum, _alignment_info.target_angular_momentum) / specific_angular_momentum.Length(), -1.0, 1.0));

            Vector3D ascending_node_vector = Vector3D.Cross(_alignment_info.target_angular_momentum, specific_angular_momentum);
            _alignment_info.ascending_node_vector = ascending_node_vector;

            double true_anomaly, mean_anomaly, node_time_offset;
            calculate_anomalies(ascending_node_vector, specific_angular_momentum, LAN_vector, eccentricity, mean_motion2, out true_anomaly, out mean_anomaly, out node_time_offset);
            _alignment_info.time_to_ascending_node = floor_mod(node_time_offset - current_time, period);
            calculate_anomalies(-ascending_node_vector, specific_angular_momentum, LAN_vector, eccentricity, mean_motion2, out true_anomaly, out mean_anomaly, out node_time_offset);
            _alignment_info.time_to_descending_node = floor_mod(node_time_offset - current_time, period);
        }

        #endregion

        #region Orbit information

        public static Vector3D[] fill_vector_elements(IMyCubeGrid instance_grid)
        {
            gravity_and_physics instance;
            if (!_grid_list.TryGetValue(instance_grid, out instance))
                return null;

            orbit_elements elements        = instance.current_elements_reader();
            Vector3D[]     vector_elements = instance._vector_elements;

            vector_elements[0] = elements.radius_vector;
            vector_elements[1] = elements.specific_angular_momentum;
            vector_elements[2] = elements.LAN_vector;
            vector_elements[3] = elements.eccentricity_vector;

            return vector_elements;
        }

        public static double[] fill_scalar_elements(IMyCubeGrid instance_grid)
        {
            gravity_and_physics instance;
            if (!_grid_list.TryGetValue(instance_grid, out instance))
                return null;

            orbit_elements elements        = instance.current_elements_reader();
            double[]       scalar_elements = instance._scalar_elements;

            scalar_elements[0] = elements.semi_major_axis;
            scalar_elements[1] = elements.eccentricity;
            scalar_elements[2] = elements.inclination;
            scalar_elements[3] = elements.longitude_of_ascending_node;
            scalar_elements[4] = elements.argument_of_periapsis;
            scalar_elements[5] = elements.true_anomaly;

            scalar_elements[ 6] = elements.orbit_period;
            scalar_elements[ 7] = elements.periapsis_radius;
            scalar_elements[ 8] = elements.apoapsis_radius;
            scalar_elements[ 9] = elements.mean_anomaly;
            scalar_elements[10] = elements.time_from_periapsis;
            scalar_elements[11] = elements.circular_speed;
            scalar_elements[12] = elements.escape_speed;
            scalar_elements[13] = elements.local_gravity_magnitude;

            return scalar_elements;
        }

        public static void list_current_elements(IMyTerminalBlock controller, StringBuilder info_text)
        {
            gravity_and_physics instance;
            if (!_grid_list.TryGetValue(controller.CubeGrid, out instance))
                return;

            if (instance._refresh_elements || instance._display_reference != null && instance._display_reference != instance._current_reference)
            {
                instance.calculate_elements(true);
                if (instance._display_reference != null && instance._display_reference != instance._current_reference)
                    instance._refresh_elements = true;
            }

            orbit_elements current_elements = instance._current_elements;
            if (current_elements.name == null)
                return;
            if (info_text.Length > 0)
                info_text.AppendLine();
            if (current_elements.eccentricity < 1.0)
            {
                info_text.AppendFormat("Orbiting {0}\nRadius: {1:F1} km\nLocal gravity: {5:F2} m/s^2\nSemi-major axis: {2:F1} km\nPeriapsis: {3:F1} km\nApoapsis: {4:F1} km",
                    current_elements.name, current_elements.radius_vector.Length() / 1000.0, current_elements.semi_major_axis / 1000.0, 
                    current_elements.periapsis_radius / 1000.0, current_elements.apoapsis_radius / 1000.0, current_elements.local_gravity_magnitude);
                info_text.AppendFormat("\nEccentricity: {0:F3}\nInclination: {1:F0} deg\nLAN: {2:F0} deg", current_elements.eccentricity, 
                    current_elements.inclination * 180.0 / Math.PI, current_elements.longitude_of_ascending_node * 180.0 / Math.PI);
                info_text.AppendFormat("\nPeriod: {0:F0} s\nTime to periapsis: {1:F0} s\nTime to apoapsis: {2:F0} s", current_elements.orbit_period,
                    current_elements.orbit_period - current_elements.time_from_periapsis, 
                    (1.5 * current_elements.orbit_period - current_elements.time_from_periapsis) % current_elements.orbit_period);
                info_text.AppendFormat("\nArgument of periapsis: {0:F0} deg\nTrue anomaly: {1:F0} deg\nMean anomaly: {2:F0} deg", 
                    current_elements.argument_of_periapsis * 180.0 / Math.PI, current_elements.true_anomaly * 180.0 / Math.PI, 
                    current_elements.mean_anomaly * 180.0 / Math.PI);
            }
            else
            {
                info_text.AppendFormat("Escaping {0}\nRadius: {1:F1} km\nLocal gravity: {4:F2} m/s^2\nSemi-major axis: {2:F1} km\nPeriapsis: {3:F1} km\nApoapsis: N/A",
                    current_elements.name, current_elements.radius_vector.Length() / 1000.0, current_elements.semi_major_axis / 1000.0, 
                    current_elements.periapsis_radius / 1000.0, current_elements.local_gravity_magnitude);
                info_text.AppendFormat("\nEccentricity: {0:F3}\nInclination: {1:F0} deg\nLAN: {2:F0} deg", current_elements.eccentricity, 
                    current_elements.inclination * 180.0 / Math.PI, current_elements.longitude_of_ascending_node * 180.0 / Math.PI);
                info_text.AppendFormat("\nPeriod: N/A\nTime {0} periapsis: {1:F0} s\nTime to apoapsis: N/A", 
                    (current_elements.time_from_periapsis >= 0.0) ? "from" : "to", Math.Abs(current_elements.time_from_periapsis));
                info_text.AppendFormat("\nArgument of periapsis: {0:F0} deg\nTrue anomaly: {1:F0} deg\nMean anomaly: {2:F2} hrad", 
                    current_elements.argument_of_periapsis * 180.0 / Math.PI, current_elements.true_anomaly * 180.0 / Math.PI, 
                    current_elements.mean_anomaly);
            }
        }

        public static string get_current_reference(IMyCubeGrid grid)
        {
            gravity_and_physics instance;
            if (!_grid_list.TryGetValue(grid, out instance))
                return null;

            if (instance._display_reference != null)
            {
                if (instance._display_reference == instance._current_reference)
                    return "<major> " + instance._display_reference.name;
                return "<minor> " + instance._display_reference.name;
            }
            return instance._current_reference?.name;
        }

        public static string set_current_reference(IMyCubeGrid grid, string body_name)
        {
            gravity_and_physics instance;
            if (!_grid_list.TryGetValue(grid, out instance))
                return null;

            body_name = body_name.ToLower();
            foreach (var cur_body in _gravity_sources.Values)
            {
                if (cur_body.name.ToLower() == body_name)
                {
                    if (instance._display_reference != cur_body && (instance._display_reference != null || instance._current_reference != cur_body))
                        instance._refresh_elements = true;
                    instance._display_reference = cur_body;
                    return cur_body.name;
                }
            }
            if (instance._display_reference != null && instance._display_reference != instance._current_reference)
                instance._refresh_elements = true;
            instance._display_reference = null;
            if (instance._current_reference == null)
                return null;
            return instance._current_reference.name + " (auto)";
        }

        public static void clear_target_plane(IMyCubeGrid grid)
        {
            gravity_and_physics instance;
            if (_grid_list.TryGetValue(grid, out instance))
                instance._alignment_info = new orbit_plane_intersection();
        }

        public static void set_target_plane(IMyCubeGrid grid, double inclination, double LAN)
        {
            gravity_and_physics instance;
            if (!_grid_list.TryGetValue(grid, out instance))
                return;

            Vector3D reference_angular_momentum;
            double   inclination_cos  = Math.Cos(inclination);
            double   inclination_cos2 = inclination_cos * inclination_cos;

            if (inclination_cos2 > 0.9999)
            {
                reference_angular_momentum.X = reference_angular_momentum.Z = 0.0;
                reference_angular_momentum.Y = 1.0;
            }
            else
            {
                double momentum_x = Math.Sin(LAN);
                double momentum_z = Math.Cos(LAN);
                reference_angular_momentum.X = momentum_x;
                reference_angular_momentum.Y = Math.Sqrt(((momentum_x * momentum_x + momentum_z * momentum_z) * inclination_cos2) / (1.0 - inclination_cos2));
                reference_angular_momentum.Z = momentum_z;
                if (inclination_cos < 0.0)
                    reference_angular_momentum.Y = -reference_angular_momentum.Y;
                reference_angular_momentum /= reference_angular_momentum.Length();
            }
            instance._alignment_info.target_angular_momentum = reference_angular_momentum;
        }

        #endregion

        #region Trajectory vectors

        public Vector3D get_circular_orbit_velocity(bool refresh_plane)
        {
            gravity_source selected_reference = _display_reference;
            if (selected_reference == null)
                selected_reference = _current_reference;
            if (selected_reference == null)
                return Vector3D.Zero;

            Vector3D grid_vector        = _grid_position - selected_reference.centre_position;
            double   grid_vector_length = grid_vector.Length();
            if (refresh_plane || _current_angular_momentum.LengthSquared() < 1.0E-6)
                _current_angular_momentum = Vector3D.Cross(grid_vector, _absolute_linear_velocity);
            if (_current_angular_momentum.LengthSquared() < 1.0E-6)
                return Vector3D.Zero;
            return Vector3D.Normalize(Vector3D.Cross(_current_angular_momentum, grid_vector)) * Math.Sqrt(selected_reference.standard_gravitational_parameter / grid_vector_length);
        }

        public Vector3 get_maneuvre_direction(engine_control_unit.ID_maneuvres selection)
        {
            if (selection == engine_control_unit.ID_maneuvres.maneuvre_off || _display_reference == null && _current_reference == null)
                return Vector3.Zero;

            if (selection == engine_control_unit.ID_maneuvres.burn_prograde)
                return Vector3.Normalize(_absolute_linear_velocity);
            if (selection == engine_control_unit.ID_maneuvres.burn_retrograde)
                return -Vector3.Normalize(_absolute_linear_velocity);

            gravity_source selected_reference = _display_reference;
            if (selected_reference == null)
                selected_reference = _current_reference;
            Vector3D grid_vector      = _grid_position - selected_reference.centre_position;
            _current_angular_momentum = Vector3D.Cross(grid_vector, _absolute_linear_velocity);
            Vector3  normal           = (_current_angular_momentum.LengthSquared() > 0.0) ? Vector3.Normalize(_current_angular_momentum) : Vector3.Zero;
            if (selection == engine_control_unit.ID_maneuvres.burn_normal)
                return normal;
            if (selection == engine_control_unit.ID_maneuvres.burn_antinormal)
                return -normal;

            Vector3 outward = Vector3.Cross(_absolute_linear_velocity, normal);
            if (outward.LengthSquared() > 0.0f)
                outward.Normalize();
            if (selection == engine_control_unit.ID_maneuvres.burn_outward)
                return outward;
            if (selection == engine_control_unit.ID_maneuvres.burn_inward)
                return -outward;
            return Vector3.Zero;

        }

        #endregion

        #region Movement

        public void get_linear_and_angular_velocities(out Vector3D world_linear_velocity, out Vector3D world_angular_velocity)
        {
            world_linear_velocity  = _absolute_linear_velocity;
            world_angular_velocity = _absolute_angular_velocity;
        }

        private void update_grid_position_and_velocity()
        {
            Vector3D new_position     = _grid.Physics.CenterOfMassWorld;
            _absolute_linear_velocity = (new_position - _grid_position) * MyEngineConstants.UPDATE_STEPS_PER_SECOND;
            _grid_position            = new_position;

            MatrixD  grid_matrix = _grid.WorldMatrix;
            Vector3D new_forward = grid_matrix.Forward, new_right = grid_matrix.Right, new_up = grid_matrix.Up;
            Vector3D angular_pitch_yaw  = Vector3D.Cross(new_forward, (new_forward - _grid_forward) * MyEngineConstants.UPDATE_STEPS_PER_SECOND);
            Vector3D angular_pitch_roll = Vector3D.Cross(new_up     , (new_up      - _grid_up     ) * MyEngineConstants.UPDATE_STEPS_PER_SECOND);
            Vector3D angular_roll_yaw   = Vector3D.Cross(new_right  , (new_right   - _grid_right  ) * MyEngineConstants.UPDATE_STEPS_PER_SECOND);
            _absolute_angular_velocity  = (angular_pitch_yaw + angular_pitch_roll + angular_roll_yaw) / 2.0;
            _grid_forward = new_forward;
            _grid_right   = new_right;
            _grid_up      = new_up;
        }

        public void apply_torque(Vector3 absolute_torque)
        {
            _current_torque = absolute_torque;
        }

        public void simulate_gravity_and_torque()
        {
            if (_grid.IsStatic || _grid.Physics == null || !_grid.Physics.Enabled)
            {
                _absolute_linear_velocity = _absolute_angular_velocity = Vector3D.Zero;
                return;
            }

            Vector3D grid_vector;
            double   grid_vector_length, gravity_magnitude;

            update_grid_position_and_velocity();
            _current_reference                = get_reference_body(_grid_position, out grid_vector, out grid_vector_length, out gravity_magnitude);
            Vector3D gravity_vector           = (grid_vector_length >= 1.0) ? (grid_vector / grid_vector_length * (-gravity_magnitude)) : Vector3D.Zero;
            Vector3  stock_gravity_force      = _grid.Physics.Gravity;
            Vector3D gravity_correction_force = _grid.Physics.Mass * (gravity_vector - _grid.Physics.Gravity) + _accumulated_gravity;

            if (gravity_correction_force.LengthSquared() >= 1.0 || _current_torque.LengthSquared() >= 1.0f)
                _grid.Physics.AddForce(MyPhysicsForceType.APPLY_WORLD_IMPULSE_AND_WORLD_ANGULAR_IMPULSE, GRAVITY_ON ? (gravity_correction_force / MyEngineConstants.UPDATE_STEPS_PER_SECOND) : Vector3D.Zero, _grid_position, _current_torque /*Vector3.Zero*/);
            _current_torque = Vector3.Zero;
            if (GRAVITY_ON && _grid.Physics.LinearVelocity.LengthSquared() <= 0.01f && stock_gravity_force.LengthSquared() < 0.0001f)
                _accumulated_gravity += gravity_correction_force / MyEngineConstants.UPDATE_STEPS_PER_SECOND;
            else
                _accumulated_gravity = Vector3D.Zero;
        }

        #endregion

        public gravity_and_physics(IMyCubeGrid grid_ref)
        {
            _grid = (MyCubeGrid) grid_ref;
            if (_grid.Physics != null)
            {
                _grid_position      = _grid.Physics.CenterOfMassWorld;
                MatrixD grid_matrix = _grid.WorldMatrix;
                _grid_forward       = grid_matrix.Forward;
                _grid_right         = grid_matrix.Right;
                _grid_up            = grid_matrix.Up;
            }
            _grid_list[grid_ref] = this;
        }
    }
}
