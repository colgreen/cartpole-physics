using CartPolePhysics.Double;

namespace CartPoleConsole.Double
{
    internal class CartSinglePoleSimulator
    {
        #region Instance Fields

        readonly double _tau;
        readonly double _durationSecs;
        readonly int _timesteps;
        readonly CartSinglePolePhysics _cartPolePhysics;
        readonly double[] _t_series;
        readonly double[] _x_series;
        readonly double[] _theta_series;
        readonly double[] _xv_series;

        #endregion

        #region Constructor

        public CartSinglePoleSimulator(
          
            double durationSecs,
            CartSinglePolePhysics cartPolePhysics)
        {
            _tau = cartPolePhysics.Tau;
            _durationSecs = durationSecs;
            _timesteps = (int)(durationSecs / _tau);
            _cartPolePhysics = cartPolePhysics;

            _t_series = new double[_timesteps];
            _x_series = new double[_timesteps];
            _theta_series = new double[_timesteps];
            _xv_series = new double[_timesteps];
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
        /// Pole angle (radians) at each timestep.
        /// </summary>
        public double[] ThetaSeries => _theta_series;

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
            _theta_series[0] = _cartPolePhysics.State[1];
            _xv_series[0] = _cartPolePhysics.State[2];

            // Run the simulation for the required number of timesteps, and record state at each timestep.
            for(int timestep=0; timestep < _timesteps; timestep++, t += _tau)
            {
                // Update model state.
                _cartPolePhysics.Update(0.0);

                // Record state.
                _t_series[timestep] = t;
                _x_series[timestep] = _cartPolePhysics.State[0];
                _theta_series[timestep] = _cartPolePhysics.State[1];
                _xv_series[timestep] = _cartPolePhysics.State[2];
            }
        }

        #endregion
    }
}
