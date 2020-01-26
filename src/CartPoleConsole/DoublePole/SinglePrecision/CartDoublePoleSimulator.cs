using CartPolePhysics.DoublePole.SinglePrecision;

namespace CartPoleConsole.DoublePole.SinglePrecision
{
    internal class CartDoublePoleSimulator
    {
        #region Instance Fields

        readonly float _tau;
        readonly float _durationSecs;
        readonly int _timesteps;
        readonly CartDoublePolePhysics _cartPolePhysics;
        readonly float[] _t_series;
        readonly float[] _x_series;
        readonly float[] _xv_series;
        readonly float[] _theta1_series;
        readonly float[] _theta2_series;

        #endregion

        #region Constructor

        public CartDoublePoleSimulator(
          
            float durationSecs,
            CartDoublePolePhysics cartPolePhysics)
        {
            _tau = cartPolePhysics.Tau;
            _durationSecs = durationSecs;
            _timesteps = (int)(durationSecs / _tau);
            _cartPolePhysics = cartPolePhysics;

            _t_series = new float[_timesteps];
            _x_series = new float[_timesteps];
            _xv_series = new float[_timesteps];
            _theta1_series = new float[_timesteps];
            _theta2_series = new float[_timesteps];
        }

        #endregion

        #region Properties

        /// <summary>
        /// The clock time at each timestep (number of seconds from simulation start).
        /// </summary>
        public float[] TimeSeries => _t_series;

        /// <summary>
        /// x-axis coordinate of the cart (metres) at each timestep.
        /// </summary>
        public float[] XSeries => _x_series;

        /// <summary>
        /// Pole 1 angle (radians) at each timestep.
        /// </summary>
        public float[] Theta1Series => _theta1_series;

        /// <summary>
        /// Pole 2 angle (radians) at each timestep.
        /// </summary>
        public float[] Theta2Series => _theta2_series;

        #endregion

        #region Public Methods

        /// <summary>
        /// Run the simulation.
        /// </summary>
        public void Run()
        {
            float t = 0f;

            // Run the simulation for the required number of timesteps, and record state at each timestep.
            for(int timestep=0; timestep < _timesteps; timestep++, t += _tau)
            {
                // Record state.
                _t_series[timestep] = t;
                _x_series[timestep] = _cartPolePhysics.State[0];
                _xv_series[timestep] = _cartPolePhysics.State[1];
                _theta1_series[timestep] = _cartPolePhysics.State[2];
                _theta2_series[timestep] = _cartPolePhysics.State[4];

                // Update model state.
                _cartPolePhysics.Update(0f);
            }
        }

        #endregion
    }
}
