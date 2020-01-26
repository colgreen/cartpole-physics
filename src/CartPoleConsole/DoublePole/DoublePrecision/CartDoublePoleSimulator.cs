using CartPolePhysics.DoublePole.DoublePrecision;

namespace CartPoleConsole.DoublePole.DoublePrecision
{
    internal class CartDoublePoleSimulator
    {
        #region Instance Fields

        readonly double _tau;
        readonly double _durationSecs;
        readonly int _timesteps;
        readonly CartDoublePolePhysics _cartPolePhysics;
        readonly double[] _t_series;
        readonly double[] _x_series;
        readonly double[] _xv_series;
        readonly double[] _theta1_series;
        readonly double[] _theta2_series;

        #endregion

        #region Constructor

        public CartDoublePoleSimulator(
          
            double durationSecs,
            CartDoublePolePhysics cartPolePhysics)
        {
            _tau = cartPolePhysics.Tau;
            _durationSecs = durationSecs;
            _timesteps = (int)(durationSecs / _tau);
            _cartPolePhysics = cartPolePhysics;

            _t_series = new double[_timesteps];
            _x_series = new double[_timesteps];
            _xv_series = new double[_timesteps];
            _theta1_series = new double[_timesteps];
            _theta2_series = new double[_timesteps];
        }

        #endregion

        #region Properties

        /// <summary>
        /// The clock time at each timestep (number of seconds from simulation start).
        /// </summary>
        public double[] TimeSeries => _t_series;

        /// <summary>
        /// x-axis coordinate of the cart (metres) at each timestep.
        /// </summary>
        public double[] XSeries => _x_series;

        /// <summary>
        /// Pole 1 angle (radians) at each timestep.
        /// </summary>
        public double[] Theta1Series => _theta1_series;

        /// <summary>
        /// Pole 2 angle (radians) at each timestep.
        /// </summary>
        public double[] Theta2Series => _theta2_series;

        #endregion

        #region Public Methods

        /// <summary>
        /// Run the simulation.
        /// </summary>
        public void Run()
        {
            double t = 0.0;

            // Record initial state.
            _t_series[0] = t;
            _x_series[0] = _cartPolePhysics.State[0];
            _xv_series[0] = _cartPolePhysics.State[1];
            _theta1_series[0] = _cartPolePhysics.State[2];
            _theta2_series[0] = _cartPolePhysics.State[4];

            // Run the simulation for the required number of timesteps, and record state at each timestep.
            for(int timestep=0; timestep < _timesteps; timestep++, t += _tau)
            {
                // Update model state.
                _cartPolePhysics.Update(0.0);

                // Record state.
                _t_series[timestep] = t;
                _x_series[timestep] = _cartPolePhysics.State[0];
                _xv_series[timestep] = _cartPolePhysics.State[1];
                _theta1_series[timestep] = _cartPolePhysics.State[2];
                _theta2_series[timestep] = _cartPolePhysics.State[4];
            }
        }

        #endregion
    }
}
