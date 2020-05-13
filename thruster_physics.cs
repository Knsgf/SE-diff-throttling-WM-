using Sandbox.Game.Entities;
using VRage.Game;
using VRage.Game.ModAPI;
using VRage.Game.Components;
using VRageMath;

namespace ttdtwm
{
    interface torque_simulation
    {
        void simulate_torque();
    }

    interface torque_control
    {
        void     apply_torque(Vector3 absolute_torque);
        void     get_linear_and_angular_velocities(out Vector3D world_linear_velocity, out Vector3D world_angular_velocity);
    }

    sealed class thruster_physics: torque_simulation, torque_control
    {
        private readonly MyCubeGrid _grid;

        private Vector3D _grid_position, _absolute_linear_velocity, _absolute_angular_velocity;
        private Vector3D _grid_forward, _grid_right, _grid_up;

        private Vector3 _current_torque = Vector3.Zero;

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

            /*
            MatrixD  grid_matrix = _grid.WorldMatrix;
            Vector3D new_forward = grid_matrix.Forward, new_right = grid_matrix.Right, new_up = grid_matrix.Up;
            Vector3D angular_pitch_yaw  = Vector3D.Cross(new_forward, (new_forward - _grid_forward) * MyEngineConstants.UPDATE_STEPS_PER_SECOND);
            Vector3D angular_pitch_roll = Vector3D.Cross(new_up     , (new_up      - _grid_up     ) * MyEngineConstants.UPDATE_STEPS_PER_SECOND);
            Vector3D angular_roll_yaw   = Vector3D.Cross(new_right  , (new_right   - _grid_right  ) * MyEngineConstants.UPDATE_STEPS_PER_SECOND);
            */
            _absolute_angular_velocity  = _grid.Physics.AngularVelocity;
            /*
            _grid_forward = new_forward;
            _grid_right   = new_right;
            _grid_up      = new_up;
            */
        }

        public void apply_torque(Vector3 absolute_torque)
        {
            _current_torque = absolute_torque;
        }

        public void simulate_torque()
        {
            if (_grid.IsStatic || _grid.Physics == null || !_grid.Physics.Enabled)
            {
                _absolute_linear_velocity = _absolute_angular_velocity = Vector3D.Zero;
                return;
            }

            update_grid_position_and_velocity();
            if (_current_torque.LengthSquared() >= 1.0f)
                _grid.Physics.AddForce(MyPhysicsForceType.APPLY_WORLD_IMPULSE_AND_WORLD_ANGULAR_IMPULSE, Vector3D.Zero, _grid_position, _current_torque);
            _current_torque = Vector3.Zero;
        }

        #endregion
        public thruster_physics(IMyCubeGrid grid_ref)
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
        }
    }
}
