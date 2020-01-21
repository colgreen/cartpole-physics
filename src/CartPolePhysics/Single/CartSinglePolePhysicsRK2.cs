using static CartPolePhysics.Single.ArrayMaths;

namespace CartPolePhysics.Single
{
    /// <summary>
    /// Represents the cart-pole physical model (with a single pole); providing a model state update method
    /// that employs a 2nd order Runge-Kutta method to project to the state at the next timestep.
    /// </summary>
    public sealed class CartSinglePolePhysicsRK2 : CartSinglePolePhysics
    {
        #region Instance Fields

        // Working memory. The k1 and k2 gradients as defined by the Runge-Kutta 2nd order method; and the
        // intermediate model state s2.
        readonly float[] _k1 = new float[4];
        readonly float[] _k2 = new float[4];
        readonly float[] _s2 = new float[4];

        #endregion

        #region Constructors

        /// <summary>
        /// Construct with the model defaults.
        /// </summary>
        public CartSinglePolePhysicsRK2() 
            : base()
        {}

        /// <summary>
        /// Construct with the provided model update timestep increment (tau).
        /// </summary>
        /// <param name="tau">The timestep increment, e.g. 0.01 for 10 millisecond increments.</param>
        public CartSinglePolePhysicsRK2(float tau) 
            : base(tau)
        {}

        /// <summary>
        /// Construct with the provided initial model state.
        /// </summary>
        /// <param name="state">The cart-pole model state variables.</param>
        public CartSinglePolePhysicsRK2(float[] state) 
            : base(state)
        {}

        /// <summary>
        /// Construct with the provided model update timestep increment (tau), and initial model state.
        /// </summary>
        /// <param name="tau">The timestep increment, e.g. 0.01 for 10 millisecond increments.</param>
        /// <param name="state">The cart-pole model state variables.</param>
        public CartSinglePolePhysicsRK2(float tau, float[] state) 
            : base(tau, state)
        {}

        /// <summary>
        /// Construct with the provided model update timestep increment (tau), initial model state, and equations of motion and parameters.
        /// </summary>
        /// <param name="tau">The timestep increment, e.g. 0.01 for 10 millisecond increments.</param>
        /// <param name="state">The cart-pole model state variables.</param>
        /// <param name="equations">The model equations of motion, and parameters.</param>
        public CartSinglePolePhysicsRK2(
            float tau,
            float[] state, 
            CartSinglePoleEquations equations) 
            : base(tau, state, equations)
        {}

        #endregion

        #region Public Methods

        /// <summary>
        /// Update the model state. I.e. move the state forward by one timestep.
        /// </summary>
        /// <param name="f">The external horizontal force applied to the cart.</param>
        /// <remarks>This implementation of Update() uses a 2nd order Runge-Kutta method, specifically Heune's method; 
        /// this is considerably more accurate that Euler's method for a given timestep size.</remarks>
        public override void Update(float f)
        {
            // Calc the cart and pole accelerations for the current/initial state, and store the k1 gradients
            _equations.CalcAccelerations(_state, f, out float xa, out float thetaa);
            _k1[0] = _state[1];
            _k1[1] = _state[3];
            _k1[2] = xa;
            _k1[3] = thetaa;

            // Project the initial state to new state s2, using the k1 gradients.
            MultiplyAdd(_s2, _state, _k1, _tau);

            // Calc the cart and pole accelerations for the s2 state, and store the k2 gradients
            _equations.CalcAccelerations(_s2, f, out xa, out thetaa);
            _k2[0] = _s2[1];
            _k2[1] = _s2[3];
            _k2[2] = xa;
            _k2[3] = thetaa;

            // Project _state to its new state, using the mean gradients given by (k1 + k2) / 2.
            for(int i=0; i < _state.Length; i++) 
            {
                _state[i] += (_k1[i] + _k2[i]) * _tau * 0.5f;
            }
        }

        #endregion
    }
}
