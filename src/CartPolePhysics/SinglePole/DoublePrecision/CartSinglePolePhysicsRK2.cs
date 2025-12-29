using static CartPolePhysics.ElementwiseMath;

namespace CartPolePhysics.SinglePole.DoublePrecision;

/// <summary>
/// Represents the cart-pole physical model (with a single pole); providing a model state update method
/// that employs a 2nd order Runge-Kutta method to project to the state at the next timestep.
/// </summary>
public sealed class CartSinglePolePhysicsRK2 : CartSinglePolePhysics
{
    // Allocate re-usable working arrays to avoid memory allocation and garbage collection overhead.
    // These are the k1 and k2 gradients as defined by the Runge-Kutta 2nd order method; and an 
    // intermediate model state s2.
    readonly double[] _k1 = new double[4];
    readonly double[] _k2 = new double[4];
    readonly double[] _s2 = new double[4];

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
    public CartSinglePolePhysicsRK2(double tau) 
        : base(tau)
    {}

    /// <summary>
    /// Construct with the provided initial model state.
    /// </summary>
    /// <param name="state">The cart-pole model state variables.</param>
    public CartSinglePolePhysicsRK2(double[] state) 
        : base(state)
    {}

    /// <summary>
    /// Construct with the provided model update timestep increment (tau), and initial model state.
    /// </summary>
    /// <param name="tau">The timestep increment, e.g. 0.01 for 10 millisecond increments.</param>
    /// <param name="state">The cart-pole model state variables.</param>
    public CartSinglePolePhysicsRK2(double tau, double[] state) 
        : base(tau, state)
    {}

    /// <summary>
    /// Construct with the provided model update timestep increment (tau), initial model state, and equations of motion and parameters.
    /// </summary>
    /// <param name="tau">The timestep increment, e.g. 0.01 for 10 millisecond increments.</param>
    /// <param name="state">The cart-pole model state variables.</param>
    /// <param name="equations">The model equations of motion, and parameters.</param>
    public CartSinglePolePhysicsRK2(
        double tau,
        double[] state, 
        CartSinglePoleEquations equations) 
        : base(tau, state, equations)
    {}

    /// <summary>
    /// Update the model state. I.e. move the state forward by one timestep.
    /// </summary>
    /// <param name="f">The external horizontal force applied to the cart.</param>
    /// <remarks>This implementation of Update() uses a 2nd order Runge-Kutta method, specifically Heune's method; 
    /// this is considerably more accurate that Euler's method for a given timestep size.</remarks>
    public override void Update(double f)
    {
        // Calc the cart and pole accelerations for the current/initial state.
        _equations.CalcAccelerations(_state, f, out double xa, out double thetaa);

        // Store a set of model state gradients, e.g. state[0] is the cart x position, therefore gradient[0] is 
        // cart x-axis velocity; and state[1] is cart x-axis velocity, therefore gradient[1] is cart x-axis acceleration, etc.
        _k1[0] = _state[1]; // Cart velocity.
        _k1[1] = xa;        // Cart acceleration.
        _k1[2] = _state[3]; // Pole angular velocity.
        _k1[3] = thetaa;    // Pole angular acceleration.

        // Project the initial state to new state s2, using the k1 gradients.
        // I.e. multiply each gradient (which is a rate of change) by a time increment (tau), to give a model state increment;
        // and then add the increments to the initial state to get a new state for time t + tau.
        MultiplyAdd(_s2, _state, _k1, _tau);

        // Calc the cart and pole accelerations for the s2 state, and store the k2 gradients.
        _equations.CalcAccelerations(_s2, f, out xa, out thetaa);
        _k2[0] = _s2[1];
        _k2[1] = xa;
        _k2[2] = _s2[3];
        _k2[3] = thetaa;

        // Project _state to its new state, using the mean gradients given by (k1 + k2) / 2.
        for(int i=0; i < 4; i++) 
        {
            _state[i] += (_k1[i] + _k2[i]) * _tau * 0.5;
        }
    }
}
