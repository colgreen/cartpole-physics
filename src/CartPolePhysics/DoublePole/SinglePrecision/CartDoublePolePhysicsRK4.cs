using static CartPolePhysics.ElementwiseMath;

namespace CartPolePhysics.DoublePole.SinglePrecision;

/// <summary>
/// Represents the cart-pole physical model (with two poles); providing a model state update method
/// that employs a classic 4th order Runge-Kutta to project to the state at the next timestep.
/// </summary>
public sealed class CartDoublePolePhysicsRK4 : CartDoublePolePhysics
{
    readonly float _tau_half;

    // Allocate re-usable working arrays to avoid memory allocation and garbage collection overhead.
    // These are the k1 to k4 gradients as defined by the Runge-Kutta 4th order method; and an 
    // intermediate model state s.
    readonly float[] _k1 = new float[6];
    readonly float[] _k2 = new float[6];
    readonly float[] _k3 = new float[6];
    readonly float[] _k4 = new float[6];
    readonly float[] _s = new float[6];

    /// <summary>
    /// Construct with the model defaults.
    /// </summary>
    public CartDoublePolePhysicsRK4() 
        : base()
    {
        _tau_half = _tau / 2f;
    }

    /// <summary>
    /// Construct with the provided model update timestep increment (tau).
    /// </summary>
    /// <param name="tau">The timestep increment, e.g. 0.01 for 10 millisecond increments.</param>
    public CartDoublePolePhysicsRK4(float tau) 
        : base(tau)
    {
        _tau_half = _tau / 2f;
    }

    /// <summary>
    /// Construct with the provided initial model state.
    /// </summary>
    /// <param name="state">The cart-pole model state variables.</param>
    public CartDoublePolePhysicsRK4(float[] state) 
        : base(state)
    {
        _tau_half = _tau / 2f;
    }

    /// <summary>
    /// Construct with the provided model update timestep increment (tau), and initial model state.
    /// </summary>
    /// <param name="tau">The timestep increment, e.g. 0.01 for 10 millisecond increments.</param>
    /// <param name="state">The cart-pole model state variables.</param>
    public CartDoublePolePhysicsRK4(float tau, float[] state) 
        : base(tau, state)
    {
        _tau_half = _tau / 2f;
    }

    /// <summary>
    /// Construct with the provided model update timestep increment (tau), initial model state, and equations of motion and parameters.
    /// </summary>
    /// <param name="tau">The timestep increment, e.g. 0.01 for 10 millisecond increments.</param>
    /// <param name="state">The cart-pole model state variables.</param>
    /// <param name="equations">The model equations of motion, and parameters.</param>
    public CartDoublePolePhysicsRK4(
        float tau,
        float[] state, 
        CartDoublePoleEquations equations) 
        : base(tau, state, equations)
    {
        _tau_half = _tau / 2f;
    }

    /// <summary>
    /// Update the model state. I.e. move the state forward by one timestep.
    /// </summary>
    /// <param name="f">The external horizontal force applied to the cart.</param>
    /// <remarks>This implementation of Update() uses classic 4th order Runge-Kutta;  this is considerably more
    /// accurate that Euler's method or 2nd order Runge-Kutta for a given timestep size.</remarks>
    public override void Update(float f)
    {
        // Calc the cart and pole accelerations for the current/initial model state.
        _equations.CalcAccelerations(_state, f, out float xa, out float thetaa1, out float thetaa2);
        // Store a set of model state gradients, e.g. state[0] is the cart x position, therefore gradient[0] is 
        // cart x-axis velocity; and state[1] is cart x-axis velocity, therefore gradient[1] is cart x-axis acceleration, etc.
        _k1[0] = _state[1]; // Cart velocity.
        _k1[1] = xa;        // Cart acceleration.
        _k1[2] = _state[3]; // Pole angular velocity.
        _k1[3] = thetaa1;   // Pole angular acceleration.
        _k1[4] = _state[5]; // Pole 2 angular velocity.
        _k1[5] = thetaa2;   // Pole 2 angular acceleration.

        // Project the initial state to new state s2, using the k1 gradients.
        // I.e. multiply each gradient (which is a rate of change) by a time increment (half tau), to give a model state increment;
        // and then add the increments to the initial state to get a new state for time t + tau/2.
        MultiplyAdd(_s, _state, _k1, _tau_half);

        // Calc the cart and pole accelerations for the s2 state, and store the k2 gradients.
        _equations.CalcAccelerations(_s, f, out xa, out thetaa1, out thetaa2);
        _k2[0] = _s[1];
        _k2[1] = xa;
        _k2[2] = _s[3];
        _k2[3] = thetaa1;
        _k2[4] = _s[5];
        _k2[5] = thetaa2;

        // Project the initial state to new state s3, using the k2 gradients.
        MultiplyAdd(_s, _state, _k2, _tau_half);

        // Calc the cart and pole accelerations for the s3 state, and store the k3 gradients.
        _equations.CalcAccelerations(_s, f, out xa, out thetaa1, out thetaa2);
        _k3[0] = _s[1];
        _k3[1] = xa;
        _k3[2] = _s[3];
        _k3[3] = thetaa1;
        _k3[4] = _s[5];
        _k3[5] = thetaa2;

        // Project the initial state to new state s4, using the k3 gradients.
        MultiplyAdd(_s, _state, _k3, _tau);

        // Calc the cart and pole accelerations for the s4 state, and store the k4 gradients
        _equations.CalcAccelerations(_s, f, out xa, out thetaa1, out thetaa2);
        _k4[0] = _s[1];
        _k4[1] = xa;
        _k4[2] = _s[3];
        _k4[3] = thetaa1;
        _k4[4] = _s[5];
        _k4[5] = thetaa2;

        // Project _state to its new state, using a weighted sum over gradients k1, k2, k3, k4.
        for(int i=0; i < 6; i++)
        {
            _state[i] += (_k1[i] + 2f*_k2[i] + 2f*_k3[i] + _k4[i]) * (_tau / 6f);
        }
    }
}
