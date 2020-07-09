using System;
using System.Collections.Generic;
using System.Text;

using Sandbox.Game.Entities;
using Sandbox.ModAPI;
using VRage.Game;
using VRage.Game.Entity;
using VRage.Game.ModAPI;
using VRage.Game.Components;
using VRage.Utils;
using VRageMath;

namespace orbiter_SE
{
    sealed class orbit_elements
    {
        private static readonly double sqrt2 = Math.Sqrt(2.0);
        
        private double _current_SGP;

        private Vector3D _specific_angular_momentum, _AN_vector = Vector3D.Zero, _eccentricity_vector, _anomaly90;

        private static double acosh(double x)
        {
            if (double.IsPositiveInfinity(x))
                return double.PositiveInfinity;
            return (x < 1.0 || double.IsNaN(x)) ? double.NaN : Math.Log(x + Math.Sqrt(x * x - 1.0));
        }

        #region Primary elements

        public string name { get; private set; }
        public bool foreign_reference { get; private set; }

        // Vector elements
        public Vector3D specific_angular_momentum => _specific_angular_momentum;
        public Vector3D ascending_node_vector     => _AN_vector;
        public Vector3D eccentricity_vector       => _eccentricity_vector;

        public Vector3D local_gravity   { get; private set; }
        public Vector3D radius_vector   { get; private set; }
        public Vector3D linear_velocity { get; private set; }

        // Size and shape elements
        public double reference_radius { get; private set; }
        public double semi_major_axis  { get; private set; }
        public double eccentricity     { get; private set; }

        // Orientation elements
        public double inclination                 { get; private set; }
        public double longitude_of_ascending_node { get; private set; }
        public double argument_of_periapsis       { get; private set; }

        // Current position
        public double true_anomaly { get; private set; }

        #endregion

        #region Derived elements

        public double semi_latus_rectum { get; private set; }
        public double periapsis_radius  { get; private set; }
        public double apoapsis_radius   { get; private set; }
        public double mean_motion       { get; private set; }
        public double orbit_period      { get; private set; }

        #endregion

        #region Positional elements

        public double mean_anomaly        { get; private set; }
        public double time_from_periapsis { get; private set; }
        public double circular_speed      { get; private set; }
        public double escape_speed        { get; private set; }
        public double predicted_speed     { get; private set; }
        public double angle_of_velocity   { get; private set; }
        public double predicted_distance  { get; private set; }

        #endregion

        #region Element calculation

        public void calculate_primary_elements(double SGP, Vector3D radius_vector, Vector3D linear_velocity, Vector3D local_gravity, string body_name, double body_radius, bool minor_body)
        {
            const double EPSILON = 1.0E-4, MAX_SEMI_MAJOR_AXIS = double.MaxValue / 1024.0;
            
            _current_SGP         = SGP;
            name                 = body_name;
            foreign_reference    = minor_body;
            this.local_gravity   = local_gravity;
            this.radius_vector   = radius_vector;
            this.linear_velocity = linear_velocity;
            reference_radius     = body_radius;

            Vector3D specific_angular_momentum;
            specific_angular_momentum = _specific_angular_momentum = Vector3D.Cross(radius_vector, linear_velocity);
            double areal_velocity     = specific_angular_momentum.Length();

            var    AN_vector        = new Vector3D(specific_angular_momentum.Z, 0.0, -specific_angular_momentum.X);
            double AN_vector_length = AN_vector.Length();
            if (AN_vector_length >= EPSILON)
                AN_vector /= AN_vector.Length();
            else
                AN_vector = Vector3D.Right;
            _AN_vector = AN_vector;

            double speed          = linear_velocity.Length();
            double circular_speed = Math.Sqrt(SGP / radius_vector.Length());
            double escape_speed   = circular_speed * sqrt2;
            Vector3D eccentricity_vector     = _eccentricity_vector;
            Vector3D new_eccentricity_vector = (((speed - circular_speed) * (speed + circular_speed)) * radius_vector - Vector3D.Dot(radius_vector, linear_velocity) * linear_velocity) / SGP;
            if ((new_eccentricity_vector - eccentricity_vector).LengthSquared() >= 0.001 * 0.001)
                eccentricity_vector = new_eccentricity_vector;
            else
                eccentricity_vector = 0.99 * eccentricity_vector + 0.01 * new_eccentricity_vector;

            double divider = (escape_speed - speed) * (escape_speed + speed);
            if (divider >= 0.0)
                semi_major_axis = (MAX_SEMI_MAJOR_AXIS * divider <= SGP) ? MAX_SEMI_MAJOR_AXIS : (SGP / divider);
            else
                semi_major_axis = (-MAX_SEMI_MAJOR_AXIS * divider <= SGP) ? (-MAX_SEMI_MAJOR_AXIS) : (SGP / divider);
            double eccentricity = eccentricity_vector.Length();
            if (semi_major_axis >= 0.0)
            {
                if (eccentricity >= 1.0)
                    eccentricity = 1.0;
            }
            else if (eccentricity <= 1.0001)
                eccentricity = 1.0001;
            if (eccentricity >= EPSILON)
                this.eccentricity = eccentricity;
            else
            {
                this.eccentricity = eccentricity = 0.0;
                eccentricity_vector = Vector3D.Zero;
            }

            inclination = (areal_velocity > 0.0) ? Math.Acos(specific_angular_momentum.Y / areal_velocity) : 0.0;
            if (inclination > -EPSILON && inclination < EPSILON)
                inclination = 0.0;
            if (inclination != 0.0)
            {
                longitude_of_ascending_node = Math.Acos(AN_vector.X);
                argument_of_periapsis       = (eccentricity > 0.0) ? Math.Acos(MathHelperD.Clamp(Vector3D.Dot(AN_vector, eccentricity_vector) / eccentricity, -1.0, 1.0)) : 0.0;
                if (AN_vector.Z > 0.0)
                    longitude_of_ascending_node = 2.0 * Math.PI - longitude_of_ascending_node;
                if (eccentricity_vector.Y < 0.0)
                    argument_of_periapsis = 2.0 * Math.PI - argument_of_periapsis;
            }
            else
            {
                longitude_of_ascending_node = 0.0;
                argument_of_periapsis       = (eccentricity > 0.0) ? Math.Acos(eccentricity_vector.X / eccentricity) : 0.0;
                if (eccentricity_vector.Z > 0.0)
                    argument_of_periapsis = 2.0 * Math.PI - argument_of_periapsis;
            }

            _anomaly90           = Vector3D.Cross(specific_angular_momentum, (eccentricity > 0.0) ? eccentricity_vector : AN_vector);
            _eccentricity_vector = eccentricity * Vector3D.Normalize(Vector3D.Cross(_anomaly90, specific_angular_momentum));

            true_anomaly = get_true_anomaly(radius_vector);
        }

        public void calculate_derived_elements()
        {
            double semi_major_axis = this.semi_major_axis, eccentricity = this.eccentricity;
            
            periapsis_radius  = semi_major_axis  * (1.0 - eccentricity);
            apoapsis_radius   = semi_major_axis  * (1.0 + eccentricity);
            semi_latus_rectum = periapsis_radius * (1.0 + eccentricity);
            mean_motion       = Math.Sqrt(Math.Abs(semi_major_axis * semi_major_axis * semi_major_axis / _current_SGP));
            orbit_period      = (semi_major_axis > 0.0) ? (2.0 * Math.PI * mean_motion) : -1.0;
        }

        public void calculate_positional_elements(double? specified_true_anomaly = null)
        {
            double true_anomaly = specified_true_anomaly ?? this.true_anomaly;
            double eccentricity = this.eccentricity;
            double divider      = 1.0 + eccentricity * Math.Cos(true_anomaly);

            double predicted_distance;
            if (divider == 0.0)
            {
                predicted_distance = this.predicted_distance = apoapsis_radius;
                angle_of_velocity  = 0.0;
            }
            else
            {
                predicted_distance = this.predicted_distance = semi_latus_rectum / divider;
                angle_of_velocity  = Math.Atan(eccentricity * Math.Sin(true_anomaly) / divider);
            }
            predicted_speed = Math.Sqrt(_current_SGP * (2.0 / predicted_distance - 1.0 / semi_major_axis));

            mean_anomaly        = convert_true_anomaly_to_mean(eccentricity, true_anomaly);
            time_from_periapsis = mean_anomaly * mean_motion;

            circular_speed = Math.Sqrt(_current_SGP / predicted_distance);
            escape_speed   = circular_speed * sqrt2;
        }

        public static double convert_true_anomaly_to_mean(double eccentricity, double true_anomaly)
        {
            double eccentric_anomaly, true_anomaly_cos = Math.Cos(true_anomaly);
            if (eccentricity < 1.0)
            {
                eccentric_anomaly = Math.Acos(MathHelperD.Clamp((eccentricity + true_anomaly_cos) / (1.0 + eccentricity * true_anomaly_cos), -1.0, 1.0));
                if (true_anomaly > Math.PI)
                    eccentric_anomaly = 2.0 * Math.PI - eccentric_anomaly;
                return eccentric_anomaly - eccentricity * Math.Sin(eccentric_anomaly);
            }
            
            eccentric_anomaly = acosh((eccentricity + true_anomaly_cos) / (1.0 + eccentricity * true_anomaly_cos));
            if (double.IsInfinity(eccentric_anomaly) || double.IsNaN(eccentric_anomaly))
                return 0.0;
            if (true_anomaly > Math.PI)
                eccentric_anomaly = -eccentric_anomaly;
            return eccentricity * Math.Sinh(eccentric_anomaly) - eccentric_anomaly;
        }

        public static double convert_mean_anomaly_to_true(double eccentricity, double mean_anomaly)
        {
            const int MAX_ITERATIONS = 20;
            
            int    remaining_iterations = MAX_ITERATIONS;
            double eccentric_anomaly, new_eccentric_anomaly, diff, prev_diff = double.MaxValue, true_anomaly;

            if (eccentricity < 1.0)
            {
                eccentric_anomaly = (mean_anomaly + Math.PI) / 2.0;
                while (true)
                {
                    new_eccentric_anomaly = eccentric_anomaly - (eccentric_anomaly - eccentricity * Math.Sin(eccentric_anomaly) - mean_anomaly)
                                                              / (1.0               - eccentricity * Math.Cos(eccentric_anomaly));
                    diff                  = Math.Abs(eccentric_anomaly - new_eccentric_anomaly);
                    eccentric_anomaly     = new_eccentric_anomaly;
                    if (diff >= prev_diff || remaining_iterations-- <= 0)
                        break;
                    prev_diff = diff;
                }
                true_anomaly = 2.0 * Math.Atan(Math.Sqrt((1.0 + eccentricity) / (1.0 - eccentricity)) * Math.Tan(eccentric_anomaly / 2.0));
            }
            else
            {
                double high, low, true_anomaly_cos;

                if (mean_anomaly >= 0.0)
                {
                    low  = 0.0;
                    high = Math.Acos(-1.0 / eccentricity);
                }
                else
                {
                    low  = -Math.Acos(-1.0 / eccentricity);
                    high = 0.0;
                }
                do
                {
                    true_anomaly      = (low + high) / 2.0;
                    true_anomaly_cos  = Math.Cos(true_anomaly);
                    eccentric_anomaly = acosh((eccentricity + true_anomaly_cos) / (1.0 + eccentricity * true_anomaly_cos));
                    if (mean_anomaly < 0.0)
                        eccentric_anomaly = -eccentric_anomaly;
                    double mean_anomaly_test = eccentricity * Math.Sinh(eccentric_anomaly) - eccentric_anomaly;
                    if (mean_anomaly_test > mean_anomaly)
                        high = true_anomaly;
                    else
                        low  = true_anomaly;
                }
                while (high - low > 0.01);

                while (true)
                {
                    new_eccentric_anomaly = eccentric_anomaly - (eccentricity * Math.Sinh(eccentric_anomaly) - eccentric_anomaly - mean_anomaly)
                                                              / (eccentricity * Math.Cosh(eccentric_anomaly) - 1.0);
                    diff                  = Math.Abs(eccentric_anomaly - new_eccentric_anomaly);
                    eccentric_anomaly     = new_eccentric_anomaly;
                    if (diff >= prev_diff || remaining_iterations-- <= 0)
                        break;
                    prev_diff = diff;
                }
                true_anomaly = 2.0 * Math.Atan(Math.Sqrt((eccentricity + 1.0) / (eccentricity - 1.0)) * Math.Tanh(eccentric_anomaly / 2.0));
            }
            if (true_anomaly < 0.0)
                true_anomaly += 2.0 * Math.PI;
            return true_anomaly;
        }

        public double get_true_anomaly(Vector3D direction)
        {
            double true_anomaly;

            if (eccentricity > 0.0)
            {
                true_anomaly = Math.Acos(MathHelperD.Clamp(Vector3D.Dot(_eccentricity_vector, direction) / (eccentricity * direction.Length()), -1.0, 1.0));
                if (Vector3D.Dot(direction, _anomaly90) < 0.0)
                    true_anomaly = 2.0 * Math.PI - true_anomaly;
            }
            else if (inclination != 0.0)
            {
                true_anomaly = Math.Acos(MathHelperD.Clamp(Vector3D.Dot(_AN_vector, direction) / direction.Length(), -1.0, 1.0));
                if (Vector3D.Dot(direction, _anomaly90) < 0.0)
                    true_anomaly = 2.0 * Math.PI - true_anomaly;
            }
            else
            {
                true_anomaly = Math.Acos(direction.X / direction.Length());
                if (direction.Z > 0.0)
                    true_anomaly = 2.0 * Math.PI - true_anomaly;
            }

            return true_anomaly;
        }

        public static Vector3D calculate_orbit_normal(double inclination, double LAN)
        {
            var AN_direction    = new Vector3(Math.Cos(LAN), 0.0, -Math.Sin(LAN));
            var normal_rotation = Quaternion.CreateFromAxisAngle(AN_direction, (float) inclination);
            return Vector3D.Transform(Vector3D.Up, normal_rotation);
        }

        #endregion
    }

    sealed class orbit_plane_intersection
    {
        public Vector3D target_normal, ascending_node_vector;
        public double   relative_inclination, time_to_ascending_node, time_to_descending_node;
    }

    interface gravity_simulation: IDisposable
    {
        orbit_elements           current_elements_reader();
        orbit_plane_intersection plane_alignment_reader();

        void simulate_gravity_and_torque();
        void update_current_reference();
    }

    interface torque_and_orbit_control
    {
        bool suppress_stabilisation { get; set; }
        
        void     apply_torque(Vector3 absolute_torque);
        void     get_linear_and_angular_velocities(out Vector3D world_linear_velocity, out Vector3D world_angular_velocity);
        void     refresh_orbit_plane(bool no_plane_change);
        Vector3D get_circular_orbit_velocity();
        Vector3  get_manoeuvre_direction(engine_control_unit.ID_manoeuvres selection);
    }

    sealed class gravity_and_physics: gravity_simulation, torque_and_orbit_control
    {
        const float REFERENCE_GRAVITY = 9.81f;
        const bool  GRAVITY_ON        = true;

        class gravity_source
        {
            public string   name;
            public double   surface_gravity, radius, radius2, standard_gravitational_parameter;
            public Vector3D centre_position;
        }

        private static readonly Dictionary<     string, gravity_source     > _gravity_source_names = new Dictionary<     string, gravity_source     >();
        private static readonly Dictionary<   MyPlanet, gravity_source     > _gravity_sources      = new Dictionary<   MyPlanet, gravity_source     >();
        private static readonly Dictionary<     string, gravity_and_physics> _grid_names           = new Dictionary<     string, gravity_and_physics>();
        private static readonly Dictionary<IMyCubeGrid, gravity_and_physics> _grid_list            = new Dictionary<IMyCubeGrid, gravity_and_physics>();

        private static readonly Dictionary<IMyTerminalBlock, orbit_elements> _PB_elements = new Dictionary<IMyTerminalBlock, orbit_elements>();

        private readonly MyCubeGrid _grid;
        private gravity_source _current_reference, _display_reference = null;

        private Vector3D _grid_position, _absolute_linear_velocity, _absolute_angular_velocity;
        private Vector3D  _current_gravity_vector = Vector3D.Zero, _accumulated_gravity = Vector3D.Zero;
        private Vector3D _current_angular_momentum = Vector3D.Up;
        private Vector3D _grid_forward, _grid_right, _grid_up;

        private Vector3 _current_torque = Vector3.Zero;

        private orbit_elements           _current_elements = new orbit_elements(), _displayed_elements;
        private orbit_plane_intersection _alignment_info   = new orbit_plane_intersection();

        private static readonly Dictionary<IMyCharacter, gravity_source> _PC_current_source = new Dictionary<IMyCharacter, gravity_source>();
        private static readonly Dictionary<IMyCharacter, gravity_source> _PC_new_source     = new Dictionary<IMyCharacter, gravity_source>();

        double _reference_energy = 0.0;
        bool   _energy_changed   = false, _suppress_stabilisation = true, _energy_received = false;

        private static byte[] _message = new byte[9];

        public static bool world_has_gravity => _gravity_sources.Count > 0;
        
        public bool suppress_stabilisation 
        { 
            get
            {
                return _suppress_stabilisation || _current_reference == null || _absolute_linear_velocity.LengthSquared() < 1.0;
            }
            set
            {
                _suppress_stabilisation = value;
            }
        }

        #region Auxiliaries

        private void log_physics_action(string method_name, string message)
        {
            MyLog.Default.WriteLine($"OSE\t{method_name}(\"{_grid.DisplayName}\"): {message}");
        }


        public orbit_elements current_elements_reader()
        {
            if (_displayed_elements == null)
                _displayed_elements = new orbit_elements();
            calculate_elements(_display_reference, ref _displayed_elements, primary_only: false);
            return _displayed_elements;
        }

        public orbit_plane_intersection plane_alignment_reader()
        {
            calculate_elements(_current_reference, ref _current_elements, primary_only: false);
            calculate_plane_intersection();
            return _alignment_info;
        }


        public static void register_gravity_source(MyPlanet new_planetoid)
        {
            var new_gravity_source = new gravity_source
            {
                name            = new_planetoid.Name,
                radius          = new_planetoid.MaximumRadius,
                radius2         = new_planetoid.MaximumRadius * new_planetoid.MaximumRadius,
                surface_gravity = REFERENCE_GRAVITY * new_planetoid.GetInitArguments.SurfaceGravity
            };
            new_gravity_source.standard_gravitational_parameter = new_gravity_source.surface_gravity * new_gravity_source.radius2;
            new_gravity_source.centre_position                  = new_planetoid.PositionComp.WorldAABB.Center;
            _gravity_sources[new_planetoid] = _gravity_source_names[new_planetoid.Name] = new_gravity_source;
            foreach (gravity_and_physics cur_physics_object in _grid_list.Values)
                cur_physics_object.update_current_reference();
        }

        public static void deregister_gravity_source(MyPlanet removed_planetoid)
        {
            if (_gravity_sources.ContainsKey(removed_planetoid))
            {
                _gravity_sources.Remove(removed_planetoid);
                _gravity_source_names.Remove(removed_planetoid.Name);
            }
            foreach (gravity_and_physics cur_physics_object in _grid_list.Values)
                cur_physics_object.update_current_reference();
        }

        private static double floor_mod(double dividend, double divisor)
        {
            return dividend - divisor * Math.Floor(dividend / divisor);
        }

        private static gravity_source get_reference_body(Vector3D entity_position)
        {
            double         gravity = 0.0, cur_gravity, distance2;
            gravity_source reference_body = null;
            Vector3D       cur_body_vector;

            foreach (gravity_source cur_body in _gravity_sources.Values)
            {
                cur_body_vector = entity_position - cur_body.centre_position;
                distance2       = cur_body_vector.LengthSquared();
                if (distance2 <= cur_body.radius2)
                    return cur_body;

                cur_gravity = cur_body.standard_gravitational_parameter / distance2;
                if (gravity < cur_gravity)
                {
                    gravity        = cur_gravity;
                    reference_body = cur_body;
                }
            }
            return reference_body;
        }

        private static Vector3D calculate_gravity_vector(gravity_source reference_body, Vector3D entity_position)
        {
            if (reference_body == null)
                return Vector3D.Zero;

            Vector3D radius_vector         = entity_position - reference_body.centre_position;
            double   radius_vector_length2 = radius_vector.LengthSquared();
            if (radius_vector_length2 < 1.0)
                return Vector3D.Zero;

            double gravity_magnitude;
            if (radius_vector_length2 > reference_body.radius2)
                gravity_magnitude = reference_body.standard_gravitational_parameter / radius_vector_length2;
            else
                gravity_magnitude = reference_body.surface_gravity * Math.Sqrt(radius_vector_length2 / reference_body.radius2);
            return radius_vector * ((-gravity_magnitude) / Math.Sqrt(radius_vector_length2));
        }

        public static void orbit_energy_handler(sync_helper.message_types message, object entity, byte[] argument, int length)
        {
            var instance = entity as gravity_and_physics;
            if (instance == null)
                return;

            if (sync_helper.running_on_server)
            {
                if (length == 8)
                {
                    ulong recipient = sync_helper.decode_unsigned(8, argument, 0);
                    instance.sync_orbit_energy(recipient);
                }
            }
            else if (length == 9)
            {
                instance._reference_energy = sync_helper.decode_double(argument, 0);
                instance._energy_received = true;
            }
        }

        public void sync_orbit_energy(ulong? recipient = null)
        {
            if (!sync_helper.running_on_server)
            {
                if (!_energy_received)
                {
                    sync_helper.encode_unsigned(screen_info.local_player.SteamUserId, 8, _message, 0);
                    sync_helper.send_message_to_server(sync_helper.message_types.ORBIT_ENERGY, this, _message, 8);
                }
                _energy_changed = false;
            }
            else if (_energy_changed && !suppress_stabilisation || recipient != null)
            {
                sync_helper.encode_double(_reference_energy, _message, 0);
                if (recipient != null)
                    sync_helper.send_message_to((ulong) recipient, sync_helper.message_types.ORBIT_ENERGY, this, _message, 9);
                else
                    sync_helper.send_message_to_others(sync_helper.message_types.ORBIT_ENERGY, this, _message, 9);
                _energy_changed = false;
            }
        }

        public void Dispose()
        {
            sync_helper.deregister_entity(sync_helper.message_types.ORBIT_ENERGY, _grid.EntityId);
            if (_grid_list.ContainsKey(_grid))
            {
                _grid_list.Remove(_grid);
                _grid_names.Remove(_grid.DisplayName);
            }
        }

        #endregion

        #region Orbit calculation

        private void zero_all_elements(ref orbit_elements elements_to_clear)
        {
            if (elements_to_clear.name != null)
                elements_to_clear = new orbit_elements();
        }

        private void calculate_elements(gravity_source selected_reference, ref orbit_elements new_elements, bool primary_only, 
            Vector3D? custom_radius = null, Vector3D? custom_velocity = null)
        {
            if (selected_reference == null)
            {
                if (_current_reference == null)
                {
                    zero_all_elements(ref new_elements);
                    return;
                }
                selected_reference = _current_reference;
            }
            Vector3D grid_vector   = custom_radius ?? (_grid_position - selected_reference.centre_position);
            double   SGP           = GRAVITY_ON ? selected_reference.standard_gravitational_parameter : (_grid.Physics.Gravity.Length() * grid_vector.LengthSquared());
            Vector3D local_gravity = calculate_gravity_vector(selected_reference, _grid_position);
            new_elements.calculate_primary_elements(SGP, grid_vector, custom_velocity ?? _absolute_linear_velocity, 
                local_gravity, selected_reference.name, selected_reference.radius, selected_reference != _current_reference);

            if (!primary_only)
            {
                new_elements.calculate_derived_elements();
                new_elements.calculate_positional_elements();
            }
        }

        private void calculate_plane_intersection()
        {
            Vector3D                 specific_angular_momentum = _current_elements.specific_angular_momentum;
            orbit_plane_intersection alignment_info            = _alignment_info;
            alignment_info.relative_inclination  = Math.Acos(MathHelperD.Clamp(Vector3D.Dot(specific_angular_momentum, alignment_info.target_normal) / specific_angular_momentum.Length(), -1.0, 1.0));
            Vector3D intersection_ascending_node = Vector3D.Cross(alignment_info.target_normal, specific_angular_momentum);
            alignment_info.ascending_node_vector = intersection_ascending_node;

            double eccentricity = _current_elements.eccentricity, mean_motion = _current_elements.mean_motion, orbit_period = _current_elements.orbit_period;
            double mean_anomaly = _current_elements.mean_anomaly;
            double intersection_true_anomaly = _current_elements.get_true_anomaly(intersection_ascending_node);
            alignment_info.time_to_ascending_node = floor_mod((orbit_elements.convert_true_anomaly_to_mean(eccentricity, intersection_true_anomaly) - mean_anomaly) * mean_motion, orbit_period);
            intersection_true_anomaly = (intersection_true_anomaly + Math.PI) % (2.0 * Math.PI);
            alignment_info.time_to_descending_node = floor_mod((orbit_elements.convert_true_anomaly_to_mean(eccentricity, intersection_true_anomaly) - mean_anomaly) * mean_motion, orbit_period);
        }

        #endregion

        #region Orbit information

        public static void dispose_PB(IMyTerminalBlock PB)
        {
            if (_PB_elements.ContainsKey(PB))
                _PB_elements.Remove(PB);
        }
        
        public static bool calculate_elements_for_PB(IMyTerminalBlock PB, string reference_name, string grid_name)
        {
            if (!world_has_gravity)
                return false;
            
            gravity_and_physics instance;
            if (grid_name == null)
                instance = _grid_list[PB.CubeGrid];
            else if (!_grid_names.TryGetValue(grid_name, out instance))
                return false;

            if (!screen_info.scripts_can_inspect_orbit_of_any_ship)
            {
                List<long> grid_owner_list = instance._grid.SmallOwners;
                if (grid_owner_list.Count > 0)
                {
                    long PB_owner      = PB.OwnerId;
                    bool access_denied = true;

                    foreach (long cur_owner in grid_owner_list)
                    {
                        if (MyIDModule.GetRelationPlayerPlayer(cur_owner, PB_owner) != MyRelationsBetweenPlayers.Enemies)
                        {
                            access_denied = false;
                            break;
                        }
                    }
                    if (access_denied)
                        return false;
                }
            }

            gravity_source selected_reference;
            if (reference_name == null)
            {
                if (instance._current_reference == null)
                    return false;
                selected_reference = instance._current_reference;
            }
            else if (!_gravity_source_names.TryGetValue(reference_name, out selected_reference))
                return false;
            
            orbit_elements PB_elements;
            if (!_PB_elements.TryGetValue(PB, out PB_elements))
                PB_elements = new orbit_elements();

            instance.calculate_elements(selected_reference, ref PB_elements, primary_only: true);
            _PB_elements[PB] = PB_elements;
            return true;
        }

        public static bool calculate_elements_for_PB_from_vectors(IMyTerminalBlock PB, string reference_name, Vector3D radius, Vector3D velocity)
        {
            if (!world_has_gravity)
                return false;

            gravity_and_physics instance = _grid_list[PB.CubeGrid];

            gravity_source selected_reference;
            if (reference_name == null)
            {
                if (instance._current_reference == null)
                    return false;
                selected_reference = instance._current_reference;
            }
            else if (!_gravity_source_names.TryGetValue(reference_name, out selected_reference))
                return false;

            orbit_elements PB_elements;
            if (!_PB_elements.TryGetValue(PB, out PB_elements))
                PB_elements = new orbit_elements();

            instance.calculate_elements(selected_reference, ref PB_elements, primary_only: true, custom_radius: radius, custom_velocity: velocity);
            _PB_elements[PB] = PB_elements;
            return true;
        }

        public static string retrieve_reference_name(IMyTerminalBlock PB)
        {
            orbit_elements PB_elements;
            if (!_PB_elements.TryGetValue(PB, out PB_elements))
                return null;
            return PB_elements.name;
        }

        public static void retrieve_primary_vectors(IMyTerminalBlock PB, Dictionary<string, Vector3D> output)
        {
            orbit_elements PB_elements;
            if (output == null || !_PB_elements.TryGetValue(PB, out PB_elements))
                return;

            output["SAM"] = PB_elements.specific_angular_momentum;
            output["ANV"] = PB_elements.ascending_node_vector;
            output["EcV"] = PB_elements.eccentricity_vector;
            output["LGV"] = PB_elements.local_gravity;
            output["Rad"] = PB_elements.radius_vector;
            output["Vel"] = PB_elements.linear_velocity;
        }

        public static void retrieve_primary_scalars(IMyTerminalBlock PB, Dictionary<string, double> output)
        {
            orbit_elements PB_elements;
            if (output == null || !_PB_elements.TryGetValue(PB, out PB_elements))
                return;

            output["RBR"] = PB_elements.reference_radius;
            output["SMA"] = PB_elements.semi_major_axis;
            output["Ecc"] = PB_elements.eccentricity;
            output["Inc"] = PB_elements.inclination;
            output["LAN"] = PB_elements.longitude_of_ascending_node;
            output["AoP"] = PB_elements.argument_of_periapsis;
            output["TrA"] = PB_elements.true_anomaly;
        }

        public static void retrieve_derived_elements(IMyTerminalBlock PB, Dictionary<string, double> output)
        {
            orbit_elements PB_elements;
            if (output == null || !_PB_elements.TryGetValue(PB, out PB_elements))
                return;

            PB_elements.calculate_derived_elements();
            output["SLR"] = PB_elements.semi_latus_rectum;
            output["PeR"] = PB_elements.periapsis_radius;
            output["ApR"] = PB_elements.apoapsis_radius;
            output["MnM"] = PB_elements.mean_motion;
            output["OrP"] = PB_elements.orbit_period;
        }

        public static void retrieve_positional_elements(IMyTerminalBlock PB, double? true_anomaly, Dictionary<string, double> output)
        {
            orbit_elements PB_elements;
            if (output == null || !_PB_elements.TryGetValue(PB, out PB_elements))
                return;

            if (true_anomaly != null)
                true_anomaly = floor_mod((double) true_anomaly, 2.0 * Math.PI);
            PB_elements.calculate_positional_elements(true_anomaly);
            output["MnA"] = PB_elements.mean_anomaly;
            output["TfP"] = PB_elements.time_from_periapsis;
            output["CiV"] = PB_elements.circular_speed;
            output["EsV"] = PB_elements.escape_speed;
            output["Vel"] = PB_elements.predicted_speed;
            output["AoV"] = PB_elements.angle_of_velocity;
            output["Rad"] = PB_elements.predicted_distance;
        }

        public static double convert_true_anomaly_to_mean(double eccentricity, double true_anomaly)
        {
            if (eccentricity < 0.0)
                eccentricity = 0.0;
            else if (eccentricity > 0.9999 && eccentricity < 1.0)
                eccentricity = 0.9999;
            else if (eccentricity >= 1.0 && eccentricity < 1.0001)
                eccentricity = 1.0001;
            true_anomaly = floor_mod(true_anomaly, 2.0 * Math.PI);
            if (eccentricity > 1.0)
            {
                double min_prohibited_angle = Math.Acos(-1.0 / eccentricity);
                if (true_anomaly > min_prohibited_angle && true_anomaly < 2.0 * Math.PI - min_prohibited_angle)
                    return 0.0;
            }
            return orbit_elements.convert_true_anomaly_to_mean(eccentricity, true_anomaly);
        }

        public static double convert_mean_anomaly_to_true(double eccentricity, double mean_anomaly)
        {
            if (eccentricity < 0.0)
                eccentricity = 0.0;
            else if (eccentricity > 0.9999)
            {
                if (eccentricity < 1.0001)
                    eccentricity = 1.0001;
                return orbit_elements.convert_mean_anomaly_to_true(eccentricity, mean_anomaly);
            }
            return orbit_elements.convert_true_anomaly_to_mean(eccentricity, floor_mod(mean_anomaly, 2.0 * Math.PI));
        }

        public static double get_true_anomaly(IMyTerminalBlock PB, Vector3D direction)
        {
            orbit_elements PB_elements;
            if (!_PB_elements.TryGetValue(PB, out PB_elements))
                return -1.0;
            Vector3D aux_vector = Vector3D.Cross(PB_elements.specific_angular_momentum, direction);
            if (Vector3D.IsZero(aux_vector))
                return -1.0;
            direction = Vector3D.Cross(Vector3D.Normalize(aux_vector), PB_elements.specific_angular_momentum);
            return PB_elements.get_true_anomaly(direction);
        }

        public static void list_current_elements(IMyTerminalBlock controller, StringBuilder info_text)
        {
            gravity_and_physics instance;
            if (!_grid_list.TryGetValue(controller.CubeGrid, out instance))
                return;

            instance.calculate_elements(instance._current_reference, ref instance._current_elements, primary_only: false);
            orbit_elements current_elements = instance._current_elements;
            if (current_elements.name == null)
                return;
            if (info_text.Length > 0)
                info_text.AppendLine();
            if (current_elements.eccentricity < 1.0)
            {
                info_text.AppendFormat("Orbiting {0}\nRadius: {1:F1} km\nLocal gravity: {5:F2} m/s^2\nSemi-major axis: {2:F1} km\nPeriapsis: {3:F1} km\nApoapsis: {4:F1} km",
                    current_elements.name, current_elements.predicted_distance / 1000.0, current_elements.semi_major_axis / 1000.0, 
                    current_elements.periapsis_radius / 1000.0, current_elements.apoapsis_radius / 1000.0, instance._current_gravity_vector.Length());
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
                    current_elements.name, current_elements.predicted_distance / 1000.0, current_elements.semi_major_axis / 1000.0, 
                    current_elements.periapsis_radius / 1000.0, instance._current_gravity_vector.Length());
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
                return (instance._display_reference == instance._current_reference)
                    ? "<major> " + instance._display_reference.name
                    : "<minor> " + instance._display_reference.name;
            }
            return instance._current_reference?.name;
        }

        public static string set_current_reference(IMyCubeGrid grid, string body_name)
        {
            gravity_and_physics instance;
            if (!_grid_list.TryGetValue(grid, out instance))
                return null;

            body_name = body_name.ToLower();
            foreach (gravity_source cur_body in _gravity_sources.Values)
            {
                if (cur_body.name.ToLower() == body_name)
                {
                    instance._display_reference = cur_body;
                    return cur_body.name;
                }
            }
            instance._display_reference = null;
            return (instance._current_reference == null) ? null : instance._current_reference.name + " (auto)";
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

            instance._alignment_info.target_normal = orbit_elements.calculate_orbit_normal(inclination, LAN);
        }

        #endregion

        #region Trajectory vectors

        public void refresh_orbit_plane(bool no_plane_change)
        {
            gravity_source selected_reference = _current_reference;
            if (selected_reference == null)
            {
                _current_angular_momentum = Vector3D.Up;
                return;
            }

            Vector3D grid_vector = _grid_position - selected_reference.centre_position;
            Vector3D new_angular_momentum;
            if (_absolute_linear_velocity.LengthSquared() > 10.0)
                new_angular_momentum = Vector3D.Cross(grid_vector, _absolute_linear_velocity);
            else
            {
                Vector3D tangent     = Vector3D.Cross(Vector3D.Up, grid_vector);
                tangent              = (tangent.LengthSquared() < 1.0) ? Vector3D.Forward : Vector3D.Normalize(tangent);
                new_angular_momentum = Vector3D.Cross(grid_vector, tangent);
            }
            if (_current_angular_momentum == Vector3D.Up)
                _current_angular_momentum = new_angular_momentum;
            else
                _current_angular_momentum = no_plane_change ? (Vector3D.Normalize(_current_angular_momentum) * new_angular_momentum.Length()) : new_angular_momentum;
        }

        public Vector3D get_circular_orbit_velocity()
        {
            gravity_source selected_reference = _current_reference;
            if (selected_reference == null)
                return Vector3D.Zero;

            Vector3D grid_vector        = _grid_position - selected_reference.centre_position;
            double   grid_vector_length = grid_vector.Length();
            if (GRAVITY_ON)
                return Vector3D.Normalize(Vector3D.Cross(_current_angular_momentum, grid_vector)) * Math.Sqrt(selected_reference.standard_gravitational_parameter / grid_vector_length);
            return  Vector3D.Normalize(Vector3D.Cross(_current_angular_momentum, grid_vector)) * Math.Sqrt(_grid.Physics.Gravity.Length() * grid_vector_length);
        }

        public Vector3 get_manoeuvre_direction(engine_control_unit.ID_manoeuvres selection)
        {
            if (selection == engine_control_unit.ID_manoeuvres.manoeuvre_off || _current_reference == null)
                return Vector3.Zero;

            Vector3D normal = Vector3D.Normalize(_current_angular_momentum);
            if (selection == engine_control_unit.ID_manoeuvres.burn_normal)
                return normal;
            if (selection == engine_control_unit.ID_manoeuvres.burn_antinormal)
                return -normal;

            Vector3D prograde                      = _absolute_linear_velocity;
            Vector3D prograde_to_normal_projection = Vector3D.Dot(prograde, normal) * normal;
            prograde -= prograde_to_normal_projection;
            if (prograde.LengthSquared() < 1.0)
                prograde = Vector3D.Cross(normal, _grid_position - _current_reference.centre_position);
            if (selection == engine_control_unit.ID_manoeuvres.burn_prograde)
                return Vector3.Normalize(prograde);
            if (selection == engine_control_unit.ID_manoeuvres.burn_retrograde)
                return -Vector3.Normalize(prograde);

            Vector3 outward = Vector3.Normalize(Vector3D.Cross(prograde, normal));
            if (selection == engine_control_unit.ID_manoeuvres.burn_outward)
                return outward;
            if (selection == engine_control_unit.ID_manoeuvres.burn_inward)
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
            Vector3D new_position = _grid.Physics.CenterOfMassWorld;
            if (!sync_helper.running_on_server)
                _absolute_linear_velocity = _grid.Physics.LinearVelocity;
            else
            {
                _absolute_linear_velocity = (new_position - _grid_position) * MyEngineConstants.UPDATE_STEPS_PER_SECOND;
                if ((_absolute_linear_velocity - _grid.Physics.LinearVelocity).LengthSquared() > 0.5 * 0.5)
                    _absolute_linear_velocity = _grid.Physics.LinearVelocity;
            }
            _grid_position = new_position;

            MatrixD  grid_matrix = _grid.WorldMatrix.GetOrientation();
            
            Vector3D new_forward = grid_matrix.Forward, new_right = grid_matrix.Right, new_up = grid_matrix.Up;
            Vector3D angular_pitch_yaw         = Vector3D.Cross(new_forward, (new_forward - _grid_forward) * MyEngineConstants.UPDATE_STEPS_PER_SECOND);
            Vector3D angular_pitch_roll        = Vector3D.Cross(new_up     , (new_up      - _grid_up     ) * MyEngineConstants.UPDATE_STEPS_PER_SECOND);
            Vector3D angular_roll_yaw          = Vector3D.Cross(new_right  , (new_right   - _grid_right  ) * MyEngineConstants.UPDATE_STEPS_PER_SECOND);
            _absolute_angular_velocity = sync_helper.running_on_server ? ((angular_pitch_yaw + angular_pitch_roll + angular_roll_yaw) / 2.0) 
                : ((Vector3D) _grid.Physics.AngularVelocity);
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
            const double step = MyEngineConstants.PHYSICS_STEP_SIZE_IN_SECONDS, MAX_ENERGY_DRIFT = 100.0, MAX_STABILISATION_ACC = 0.05;

            if (_grid.IsStatic || _grid.Physics == null || !_grid.Physics.Enabled)
            {
                _absolute_linear_velocity = _absolute_angular_velocity = Vector3D.Zero;
                return;
            }

            update_grid_position_and_velocity();
            gravity_source current_reference = _current_reference;
            Vector3D       grid_position     = _grid_position;
            Vector3D       grid_velocity     = _absolute_linear_velocity;
            Vector3D       gravity_vector    = calculate_gravity_vector(current_reference, grid_position);

            if (current_reference == null)
            {
                if (sync_helper.running_on_server)
                {
                    _reference_energy = 0.0;
                    _energy_changed   = true;
                }
            }
            else
            {
                double speed2 = grid_velocity.LengthSquared();
                double orbital_energy = speed2 / 2.0 - current_reference.standard_gravitational_parameter / (grid_position - current_reference.centre_position).Length();
                if (suppress_stabilisation)
                {
                    if (sync_helper.running_on_server)
                    {
                        _reference_energy = orbital_energy;
                        _energy_changed   = true;
                    }
                }
                else
                {
                    double energy_drift = orbital_energy - _reference_energy;
                    if (energy_drift < -MAX_ENERGY_DRIFT)
                    {
                        energy_drift = -MAX_ENERGY_DRIFT;
                        if (sync_helper.running_on_server)
                        {
                            _reference_energy = orbital_energy - energy_drift;
                            _energy_changed   = true;
                        }
                    }
                    else if (energy_drift > MAX_ENERGY_DRIFT)
                    {
                        energy_drift = MAX_ENERGY_DRIFT;
                        if (sync_helper.running_on_server)
                        {
                            _reference_energy = orbital_energy - energy_drift;
                            _energy_changed   = true;
                        }
                    }
                    double stabilisation_acc = energy_drift * (-MAX_STABILISATION_ACC / MAX_ENERGY_DRIFT);
                    gravity_vector += Vector3D.Normalize(grid_velocity) * stabilisation_acc;
                }
            }

            Vector3 stock_gravity_force = _grid.Physics.Gravity;
            double  gravity_magnitude   = gravity_vector.Length(), stock_gravity_magnitude = stock_gravity_force.Length();
            if (gravity_magnitude < stock_gravity_magnitude)
                gravity_vector *= stock_gravity_magnitude / gravity_magnitude;
            Vector3D gravity_correction_force = _grid.Physics.Mass * (gravity_vector - stock_gravity_force) + _accumulated_gravity;
            if (gravity_correction_force.LengthSquared() >= 1.0 || _current_torque.LengthSquared() >= 1.0f)
            {
                Vector3 gravity_correction_impulse = GRAVITY_ON ? ((Vector3) (gravity_correction_force * step)) : Vector3.Zero;
                _grid.Physics.AddForce(MyPhysicsForceType.APPLY_WORLD_IMPULSE_AND_WORLD_ANGULAR_IMPULSE, gravity_correction_impulse, grid_position, _current_torque);
            }
            _current_torque = Vector3.Zero;
            if (GRAVITY_ON && _grid.Physics.LinearVelocity.LengthSquared() <= 0.01f && stock_gravity_force.LengthSquared() < 0.0001f)
            {
                if (_accumulated_gravity.LengthSquared() < 1.0)
                    _accumulated_gravity += gravity_correction_force / MyEngineConstants.UPDATE_STEPS_PER_SECOND;
            }
            else
                _accumulated_gravity = Vector3D.Zero;
            _current_gravity_vector = gravity_vector;
        }

        public void update_current_reference()
        {
            gravity_source prev_reference = _current_reference;

            _current_reference = get_reference_body(_grid_position);
            if (_current_reference != null)
            {
                if (sync_helper.running_on_server && _current_reference != prev_reference)
                {
                    _reference_energy = _grid.Physics.LinearVelocity.LengthSquared() / 2.0
                        - _current_reference.standard_gravitational_parameter / (_grid_position - _current_reference.centre_position).Length();
                    _energy_changed = true;
                }
                sync_orbit_energy();
            }
            //log_physics_action("update_current_reference", $"{(_grid_position - _current_reference.centre_position).Length()} {_absolute_linear_velocity.Length()}");
        }

        #endregion

        #region Player character movement

        public static void register_player(IMyCharacter player)
        {
            _PC_current_source.Add(player, null);
        }

        public static void deregister_player(IMyCharacter player)
        {
            if (_PC_current_source.ContainsKey(player))
                _PC_current_source.Remove(player);
        }

        public static void apply_gravity_to_players()
        {
            IMyCharacter   player;
            gravity_source current_source;
            Vector3D       player_position, gravity_vector, stock_gravity_vector;

            foreach (KeyValuePair<IMyCharacter, gravity_source> player_entry in _PC_current_source)
            {
                player         = player_entry.Key;
                current_source = player_entry.Value;
                if (current_source == null || !player.EnabledThrusts || player.EnabledDamping || player.Physics == null || !player.Physics.Enabled)
                    continue;

                player_position      = player.PositionComp.GetPosition();
                gravity_vector       = calculate_gravity_vector(current_source, player_position);
                stock_gravity_vector = player.Physics.Gravity;
                player.Physics.AddForce(MyPhysicsForceType.APPLY_WORLD_IMPULSE_AND_WORLD_ANGULAR_IMPULSE, (gravity_vector - stock_gravity_vector) * player.Physics.Mass / MyEngineConstants.UPDATE_STEPS_PER_SECOND, player_position, Vector3.Zero);
            }
        }

        public static void update_player_reference_bodies()
        {
            IMyCharacter   player;
            gravity_source new_source;

            _PC_new_source.Clear();
            foreach (KeyValuePair<IMyCharacter, gravity_source> player_entry in _PC_current_source)
            {
                player     = player_entry.Key;
                new_source = get_reference_body(player.PositionComp.GetPosition());
                if (player_entry.Value != new_source)
                    _PC_new_source.Add(player, new_source);
            }
            foreach (KeyValuePair<IMyCharacter, gravity_source> changed_player_entry in _PC_new_source)
                _PC_current_source[changed_player_entry.Key] = changed_player_entry.Value;
        }

        #endregion

        public gravity_and_physics(IMyCubeGrid grid_ref)
        {
            _grid = (MyCubeGrid) grid_ref;
            sync_helper.register_entity(sync_helper.message_types.ORBIT_ENERGY, this, _grid.EntityId);
            if (_grid.Physics != null)
            {
                _grid_position      = _grid.Physics.CenterOfMassWorld;
                MatrixD grid_matrix = _grid.WorldMatrix;
                _grid_forward       = grid_matrix.Forward;
                _grid_right         = grid_matrix.Right;
                _grid_up            = grid_matrix.Up;
                update_current_reference();
            }
            _grid_list[grid_ref] = _grid_names[grid_ref.DisplayName] = this;
        }
    }
}
