using CartPolePhysics.SinglePole.SinglePrecision;

namespace CartPoleConsole.SinglePole.SinglePrecision
{
    internal class CartSinglePoleSimulator
    {
        readonly float _tau;
        readonly int _timesteps;
        readonly CartSinglePolePhysics _cartPolePhysics;
        readonly float[] _t_series;
        readonly float[] _x_series;
        readonly float[] _xv_series;
        readonly float[] _theta_series;

        public CartSinglePoleSimulator(
          
            float durationSecs,
            CartSinglePolePhysics cartPolePhysics)
        {
            _tau = cartPolePhysics.Tau;
            _timesteps = (int)(durationSecs / _tau);
            _cartPolePhysics = cartPolePhysics;

            _t_series = new float[_timesteps];
            _x_series = new float[_timesteps];
            _xv_series = new float[_timesteps];
            _theta_series = new float[_timesteps];
        }

        /// <summary>
        /// The clock time at each timestep (number of seconds from simulation start).
        /// </summary>
        public float[] TimeSeries => _t_series;

        /// <summary>
        /// x-axis coordinate of the cart (metres) at each timestep.
        /// </summary>
        public float[] XSeries => _x_series;

        /// <summary>
        /// Pole angle (radians) at each timestep.
        /// </summary>
        public float[] ThetaSeries => _theta_series;

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
                _theta_series[timestep] = _cartPolePhysics.State[2];

                // Update model state.
                _cartPolePhysics.Update(0f);
            }
        }
    }
}
