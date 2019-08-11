using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Sandbox.Game.Entities;
using Sandbox.Game.Entities.Planet;
using Sandbox.Game.Entities.Cube;
using VRage.Game;
using VRage.Game.Components;
using VRage.Utils;
using VRageMath;

namespace ttdtwm
{
    class gravity_and_physics
    {
        const float REFERENCE_GRAVITY = 9.81f;

        class gravity_source
        {
            public double   surface_gravity, mean_radius2, standard_gravitational_parameter;
            public Vector3D centre_position;
        }

        private static Dictionary<MyPlanet, gravity_source> _gravity_sources = new Dictionary<MyPlanet, gravity_source>();

        private MyCubeGrid     _grid;
        private gravity_source _current_reference;

        private Vector3D _grid_position, _absolute_velocity, _accumulated_gravity = Vector3D.Zero;
        private Vector3D _eccentricity_vector, _specific_angular_momentum;
        private Vector3D _grid_forward, _grid_right, _grid_up;

        public static void register_gravity_source(MyPlanet new_planetoid)
        {
            gravity_source new_gravity_source = new gravity_source();

            new_gravity_source.mean_radius2                     = new_planetoid.AverageRadius * new_planetoid.AverageRadius;
            new_gravity_source.surface_gravity                  = REFERENCE_GRAVITY * new_planetoid.GetInitArguments.SurfaceGravity;
            new_gravity_source.standard_gravitational_parameter = new_gravity_source.surface_gravity * new_gravity_source.mean_radius2;
            new_gravity_source.centre_position                  = new_planetoid.PositionComp.WorldAABB.Center;
            _gravity_sources[new_planetoid] = new_gravity_source;
            MyLog.Default.WriteLine("register_gravity_source(): " + new_planetoid.Name);
        }

        public static void deregister_gravity_source(MyPlanet removed_planetoid)
        {
            if (_gravity_sources.ContainsKey(removed_planetoid))
                _gravity_sources.Remove(removed_planetoid);
        }
        
        #region Auxiliaries

        private static double acosh(double x)
        {
            if (x == 1.0)
                return 0.0;
            if (double.IsPositiveInfinity(x))
                return double.PositiveInfinity;
            if (x < 1.0 || double.IsNaN(x))
                return double.NaN;

            double prev_result = Math.Abs(Math.Log(2.0 * Math.Abs(x)));
            double result, diff, prev_diff = double.MaxValue;

            while (true)
            {
                result = prev_result + (x - Math.Cosh(prev_result)) / Math.Sinh(prev_result);
                diff   = Math.Abs(result - prev_result);
                if (diff >= prev_diff)
                    return result;
                prev_result = result;
                prev_diff   = diff;
            }
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
                gravity         = (distance2 >= cur_body.mean_radius2) ? (cur_body.standard_gravitational_parameter / distance2) 
                                                                       : (cur_body.surface_gravity * Math.Sqrt(distance2 / cur_body.mean_radius2));
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


        #endregion

        #region Orbit information

        public Vector3D get_circular_orbit_velocity(Vector3D manual_velocity_change, bool recalculate_plane)
        {
            Vector3D       grid_vector;
            double         grid_vector_length, dummy;
            gravity_source reference_body = get_reference_body(_grid_position, out grid_vector, out grid_vector_length, out dummy);

            if (recalculate_plane || _current_reference != reference_body || Vector3D.IsZero(_specific_angular_momentum))
            {
                _specific_angular_momentum = Vector3D.Cross(grid_vector, _absolute_velocity);
                _current_reference         = reference_body;
            }
            else
            {
                Vector3D circular_velocity = Vector3D.Normalize(Vector3D.Cross(_specific_angular_momentum, grid_vector)) * (_specific_angular_momentum.Length() / grid_vector_length);
                _specific_angular_momentum = Vector3D.Cross(grid_vector, circular_velocity + manual_velocity_change * (circular_velocity.Length() / _absolute_velocity.Length()));
            }

            return Vector3D.Normalize(Vector3D.Cross(_specific_angular_momentum, grid_vector)) * Math.Sqrt(reference_body.standard_gravitational_parameter / grid_vector_length);
        }

        private void calculate_elements()
        {
            Vector3D       grid_vector;
            double         grid_vector_length, gravity_magnitude;
            gravity_source reference_body = get_reference_body(_grid_position, out grid_vector, out grid_vector_length, out gravity_magnitude);
            double         SGP            = reference_body.standard_gravitational_parameter;

            Vector3D specific_angular_momentum = Vector3D.Cross(grid_vector, _absolute_velocity);
            double   speed                     = _absolute_velocity.Length();
            double   gravity_energy            = SGP / grid_vector_length;
            double   circular_speed            = Math.Sqrt(      gravity_energy);
            double   escape_speed              = Math.Sqrt(2.0 * gravity_energy);
            double   semi_major_axis           = SGP / ((escape_speed - speed) * (escape_speed + speed));
            Vector3D new_eccentricity_vector   = (((speed - circular_speed) * (speed + circular_speed)) * grid_vector - Vector3D.Dot(grid_vector, _absolute_velocity) * _absolute_velocity) / SGP;
            if ((new_eccentricity_vector - _eccentricity_vector).LengthSquared() >= 0.001 * 0.001)
                _eccentricity_vector = new_eccentricity_vector;
            double eccentricity = _eccentricity_vector.Length();
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

            double period2 = semi_major_axis * semi_major_axis * semi_major_axis / SGP;
            double period  = (semi_major_axis > 0.0 && !double.IsInfinity(semi_major_axis)) ? (2.0 * Math.PI * Math.Sqrt(period2)) : -1.0;
            double true_anomaly_half, true_anomaly_full;
            if (eccentricity > 0.0)
            {
                Vector3D minor_axis_vector = Vector3D.Cross(specific_angular_momentum, _eccentricity_vector);

                true_anomaly_half = Math.Acos(Vector3D.Dot(_eccentricity_vector, grid_vector) / (eccentricity * grid_vector_length));
                if (!double.IsNaN(true_anomaly_half))
                    true_anomaly_full = (Vector3D.Dot(grid_vector, minor_axis_vector) < 0.0) ? (2.0 * Math.PI - true_anomaly_half) : true_anomaly_half;
                else
                    true_anomaly_half = true_anomaly_full = 0.0;
            }
            else
            {
                Vector3D LAN_vector = new Vector3D(-specific_angular_momentum.Y, specific_angular_momentum.X, 0.0);

                if (LAN_vector.Length() > 0.0)
                {
                    true_anomaly_half = Math.Acos(Vector3D.Dot(LAN_vector, grid_vector) / (LAN_vector.Length() * grid_vector_length));
                    if (!double.IsNaN(true_anomaly_half))
                        true_anomaly_full = (Vector3D.Dot(LAN_vector, _absolute_velocity) > 0.0) ? (2.0 * Math.PI - true_anomaly_half) : true_anomaly_half;
                    else
                        true_anomaly_half = true_anomaly_full = 0.0;
                }
                else
                {
                    true_anomaly_half = Math.Acos(grid_vector.X / grid_vector_length);
                    if (!double.IsNaN(true_anomaly_half))
                        true_anomaly_full = (_absolute_velocity.X > 0.0) ? (2.0 * Math.PI - true_anomaly_half) : true_anomaly_half;
                    else
                        true_anomaly_half = true_anomaly_full = 0.0;
                }
            }

            double eccentric_anomaly, true_anomaly_cos = Math.Cos(true_anomaly_half), mean_anomaly, time_from_periapsis;
            if (eccentricity < 1.0)
            {
                eccentric_anomaly = Math.Acos((eccentricity + true_anomaly_cos) / (1.0 + eccentricity * true_anomaly_cos));
                if (true_anomaly_full > Math.PI)
                    eccentric_anomaly = 2.0 * Math.PI - eccentric_anomaly;
                mean_anomaly = eccentric_anomaly - eccentricity * Math.Sin(eccentric_anomaly);
                time_from_periapsis = mean_anomaly * Math.Sqrt(period2);
            }
            else if (eccentricity > 1.0)
            {
                eccentric_anomaly = acosh((eccentricity + true_anomaly_cos) / (1.0 + eccentricity * true_anomaly_cos));
                if (eccentric_anomaly < 0.0)
                    mean_anomaly = time_from_periapsis = 0.0;
                else
                {
                    if (true_anomaly_full > Math.PI)
                        eccentric_anomaly = -eccentric_anomaly;
                    mean_anomaly = eccentricity * Math.Sinh(eccentric_anomaly) - eccentric_anomaly;
                    time_from_periapsis = mean_anomaly * Math.Sqrt(-period2);
                }
            }
            else
            {
                eccentric_anomaly = Math.Atan(true_anomaly_half / 2.0);
                if (true_anomaly_full > Math.PI)
                    eccentric_anomaly = -eccentric_anomaly;
                mean_anomaly = eccentric_anomaly * (1.0 + eccentric_anomaly * eccentric_anomaly / 3.0);
                time_from_periapsis = mean_anomaly * Math.Sqrt(2.0 * periapsis_radius * periapsis_radius * periapsis_radius / SGP);
            }


            float v1 = (float) Math.Sqrt(gravity_energy);
            //float v1 = (float) Math.Sqrt(grid.Physics.Gravity.Length() * grid_vector_length);

            screen_info.screen_text(_grid, "", string.Format("R = {0:F1} km, g = {3:F2} m/s^2, v1 = {1:F1} m/s, v2 = {2:F1} m/s", grid_vector_length / 1000.0, v1, v1 * Math.Sqrt(2.0), gravity_magnitude), 16, controlled_only: true);
            screen_info.screen_text(_grid, "", string.Format("SMA = {0:F1} km, e = {3:F3}, PR = {1:F1} km, AR = {2:F1} km", semi_major_axis / 1000.0, periapsis_radius / 1000.0, apoapsis_radius / 1000.0, eccentricity), 16, controlled_only: true);
            screen_info.screen_text(_grid, "", string.Format("T = {0:F0} s, TA = {1:F0}, MA = {2:F0}, TtCA = {3:F0} s", period, true_anomaly_full * 180.0 / Math.PI, mean_anomaly * 180.0 / Math.PI, (true_anomaly_full > Math.PI) ?  (period - time_from_periapsis) : (period / 2.0 - time_from_periapsis)), 16, controlled_only: true);
        }

        #endregion

        #region Movement

        public void update_grid_position_and_velocity()
        {
            Vector3D dummy;
            get_linear_and_angular_velocities(out dummy, out dummy);
        }

        public void get_linear_and_angular_velocities(out Vector3D world_linear_velocity, out Vector3D world_angular_velocity)
        {
            if (_grid.Physics == null)
            {
                world_linear_velocity = _absolute_velocity = world_angular_velocity = Vector3D.Zero;
                return;
            }

            Vector3D new_position = _grid.Physics.CenterOfMassWorld;
            world_linear_velocity = _absolute_velocity = (new_position - _grid_position) * MyEngineConstants.UPDATE_STEPS_PER_SECOND;
            _grid_position        = new_position;

            MatrixD  grid_matrix = _grid.WorldMatrix;
            Vector3D new_forward = grid_matrix.Forward, new_right = grid_matrix.Right, new_up = grid_matrix.Up;
            Vector3D angular_pitch_yaw  = Vector3D.Cross(new_forward, (new_forward - _grid_forward) * MyEngineConstants.UPDATE_STEPS_PER_SECOND);
            Vector3D angular_pitch_roll = Vector3D.Cross(new_up     , (new_up      - _grid_up     ) * MyEngineConstants.UPDATE_STEPS_PER_SECOND);
            Vector3D angular_roll_yaw   = Vector3D.Cross(new_right  , (new_right   - _grid_right  ) * MyEngineConstants.UPDATE_STEPS_PER_SECOND);
            world_angular_velocity      = (angular_pitch_yaw + angular_pitch_roll + angular_roll_yaw) / 2.0;
            _grid_forward = new_forward;
            _grid_right   = new_right;
            _grid_up      = new_up;
        }

        public void apply_gravity_and_torque(Vector3 absolute_torque, bool linear_damping_on)
        {
            Vector3D       grid_vector;
            double         grid_vector_length, gravity_magnitude;
            gravity_source reference_body           = get_reference_body(_grid_position, out grid_vector, out grid_vector_length, out gravity_magnitude);
            Vector3D       gravity_vector           = (grid_vector_length >= 1.0) ? (grid_vector / grid_vector_length * (-gravity_magnitude)) : Vector3D.Zero;
            Vector3D       gravity_correction_force = _grid.Physics.Mass * (gravity_vector - _grid.Physics.Gravity) + _accumulated_gravity;

            if (gravity_correction_force.LengthSquared() >= 1.0 || absolute_torque.LengthSquared() >= 1.0f)
                _grid.Physics.AddForce(MyPhysicsForceType.APPLY_WORLD_IMPULSE_AND_WORLD_ANGULAR_IMPULSE, gravity_correction_force / MyEngineConstants.UPDATE_STEPS_PER_SECOND /*Vector3.Zero*/, _grid_position, absolute_torque /*Vector3.Zero*/);
            if (!linear_damping_on && _grid.Physics.LinearVelocity.LengthSquared() <= 0.01f)
                _accumulated_gravity += gravity_correction_force;
            else
                _accumulated_gravity = Vector3D.Zero;
            calculate_elements();
        }

        #endregion

        public gravity_and_physics(MyCubeGrid grid)
        {
            _grid = grid;
            if (_grid.Physics != null)
            {
                _grid_position      = _grid.Physics.CenterOfMassWorld;
                MatrixD grid_matrix = _grid.WorldMatrix;
                _grid_forward       = grid_matrix.Forward;
                _grid_right         = grid_matrix.Right;
                _grid_up            = grid_matrix.Up;
            }
        }
    }
}
