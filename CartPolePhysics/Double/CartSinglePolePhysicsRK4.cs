﻿using static CartPolePhysics.Double.ArrayMaths;

namespace CartPolePhysics.Double
{
    /// <summary>
    /// Represents the cart-pole physical model (with a single pole); providing a model state update method
    /// that employs a classic 4th order order Runge-Kutta to project to the state at the next timestep.
    /// </summary>
    public sealed class CartSinglePolePhysicsRK4 : CartSinglePolePhysics
    {
        #region Instance Fields

        readonly double _tau_half;

        // Working memory. The k1 to k4 gradients as defined by the Runge-Kutta 4th order method; and a
        // working intermediate model state s.
        readonly double[] _k1 = new double[4];
        readonly double[] _k2 = new double[4];
        readonly double[] _k3 = new double[4];
        readonly double[] _k4 = new double[4];
        readonly double[] _s = new double[4];

        #endregion

        #region Constructors

        /// <summary>
        /// Construct with the model defaults.
        /// </summary>
        public CartSinglePolePhysicsRK4() 
            : base()
        {
            _tau_half = _tau / 2.0;
        }

        /// <summary>
        /// Construct with the provided model update timestep increment (tau).
        /// </summary>
        /// <param name="tau">The timestep increment, e.g. 0.01 for 10 millisecond increments.</param>
        public CartSinglePolePhysicsRK4(double tau) 
            : base(tau)
        {
            _tau_half = _tau / 2.0;
        }

        /// <summary>
        /// Construct with the provided initial model state.
        /// </summary>
        /// <param name="state">The cart-pole model state variables.</param>
        public CartSinglePolePhysicsRK4(double[] state) 
            : base(state)
        {
            _tau_half = _tau / 2.0;
        }

        /// <summary>
        /// Construct with the provided model update timestep increment (tau), and initial model state.
        /// </summary>
        /// <param name="tau">The timestep increment, e.g. 0.01 for 10 millisecond increments.</param>
        /// <param name="state">The cart-pole model state variables.</param>
        public CartSinglePolePhysicsRK4(double tau, double[] state) 
            : base(tau, state)
        {
            _tau_half = _tau / 2.0;
        }

        /// <summary>
        /// Construct with the provided model update timestep increment (tau), initial model state, and equations of motion and parameters.
        /// </summary>
        /// <param name="tau">The timestep increment, e.g. 0.01 for 10 millisecond increments.</param>
        /// <param name="state">The cart-pole model state variables.</param>
        /// <param name="equations">The model equations of motion, and parameters.</param>
        public CartSinglePolePhysicsRK4(
            double tau,
            double[] state, 
            CartSinglePoleEquations equations) 
            : base(tau, state, equations)
        {
            _tau_half = _tau / 2.0;
        }

        #endregion

        #region Public Methods

        /// <summary>
        /// Update the model state. I.e. move the state forward by one timestep.
        /// </summary>
        /// <param name="f">The external horizontal force applied to the cart.</param>
        /// <remarks>This implementation of Update() uses classic 4th order Runge-Kutta;  this is considerably more
        /// accurate that Euler's method or 2nd order Runge-Kutta for a given timestep size.</remarks>
        public override void Update(double f)
        {
            // Calc the cart and pole accelerations for the current/initial state, and store the k1 gradients
            _equations.CalcAccelerations(_state, f, out double xa, out double thetaa);
            _k1[0] = _state[2];
            _k1[1] = _state[3];
            _k1[2] = xa;
            _k1[3] = thetaa;

            // Project the initial state to new state s2, using the k1 gradients.
            MultiplyAdd(_s, _state, _k1, _tau_half);

            // Calc the cart and pole accelerations for the s2 state, and store the k2 gradients
            _equations.CalcAccelerations(_s, f, out xa, out thetaa);
            _k2[0] = _s[2];
            _k2[1] = _s[3];
            _k2[2] = xa;
            _k2[3] = thetaa;

            // Project the initial state to new state s3, using the k2 gradients.
            MultiplyAdd(_s, _state, _k2, _tau_half);

            // Calc the cart and pole accelerations for the s3 state, and store the k3 gradients
            _equations.CalcAccelerations(_s, f, out xa, out thetaa);
            _k3[0] = _s[2];
            _k3[1] = _s[3];
            _k3[2] = xa;
            _k3[3] = thetaa;

            // Project the initial state to new state s4, using the k3 gradients.
            MultiplyAdd(_s, _state, _k3, _tau);

            // Calc the cart and pole accelerations for the s4 state, and store the k4 gradients
            _equations.CalcAccelerations(_s, f, out xa, out thetaa);
            _k4[0] = _s[2];
            _k4[1] = _s[3];
            _k4[2] = xa;
            _k4[3] = thetaa;

            // Project _state to its new state, using a weighted sum over k1,k2,k3,k4.
            for(int i=0; i < _state.Length; i++)
            {
                _state[i] += (_k1[i] + 2.0*_k2[i] + 2.0*_k3[i] + _k4[i]) * (_tau / 6.0);
            }
        }

        #endregion
    }
}
