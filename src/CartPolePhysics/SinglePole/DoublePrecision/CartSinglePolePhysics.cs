using System;
using System.Diagnostics;

namespace CartPolePhysics.SinglePole.DoublePrecision;

/// <summary>
/// Represents the cart-pole physical model (with a single pole); providing a model state update method
/// that employs Euler's method to project to the state at the next timestep.
/// </summary>
public class CartSinglePolePhysics
{
    /// <summary>
    /// The timestep increment, e.g. 0.01 for 10 millisecond increments.
    /// </summary>
    protected readonly double _tau = 0.01;

    /// <summary>
    /// The model state variables are:
    ///  [0] x-axis coordinate of the cart (metres).
    ///  [1] x-axis velocity of the cart (m/s).
    ///  [2] Pole angle (radians); deviation from the vertical. Positive is clockwise.
    ///  [3] Pole angular velocity (radians/s). Positive is clockwise.
    /// </summary>
    protected double[] _state;

    /// <summary>
    /// The differential equations for the cart and single pole model.
    /// </summary>
    protected readonly CartSinglePoleEquations _equations;

    /// <summary>
    /// The cart-pole model state variables.
    /// </summary>
    /// <remarks>
    /// The model state variables are:
    ///  [0] x-axis coordinate of the cart (metres).
    ///  [1] x-axis velocity of the cart (m/s).
    ///  [2] Pole angle (radians); deviation from the vertical. Positive is clockwise.
    ///  [3] Pole angular velocity (radians/s). Positive is clockwise.
    /// </remarks>
    public double[] State => _state;

    /// <summary>
    /// The timestep increment, e.g. 0.01 for 10 millisecond increments.
    /// </summary>
    public double Tau => _tau;

    /// <summary>
    /// Construct with the model defaults.
    /// </summary>
    public CartSinglePolePhysics()
    {
        _state = new double[4];
        _equations = new CartSinglePoleEquations();
    }

    /// <summary>
    /// Construct with the provided model update timestep increment (tau).
    /// </summary>
    /// <param name="tau">The timestep increment, e.g. 0.01 for 10 millisecond increments.</param>
    public CartSinglePolePhysics(double tau)
    {
        _tau = tau;
        _state = new double[4];
        _equations = new CartSinglePoleEquations();
    }

    /// <summary>
    /// Construct with the provided initial model state.
    /// </summary>
    /// <param name="state">The cart-pole model state variables.</param>
    public CartSinglePolePhysics(double[] state)
    {
        Debug.Assert(state.Length == 4);
        _state = state;
        _equations = new CartSinglePoleEquations();
    }

    /// <summary>
    /// Construct with the provided model update timestep increment (tau), and initial model state.
    /// </summary>
    /// <param name="tau">The timestep increment, e.g. 0.01 for 10 millisecond increments.</param>
    /// <param name="state">The cart-pole model state variables.</param>
    public CartSinglePolePhysics(double tau, double[] state)
    {
        Debug.Assert(state.Length == 4);
        _tau = tau;
        _state = state;
        _equations = new CartSinglePoleEquations();
    }

    /// <summary>
    /// Construct with the provided model update timestep increment (tau), initial model state, and equations of motion and parameters.
    /// </summary>
    /// <param name="tau">The timestep increment, e.g. 0.01 for 10 millisecond increments.</param>
    /// <param name="state">The cart-pole model state variables.</param>
    /// <param name="equations">The model equations of motion, and parameters.</param>
    public CartSinglePolePhysics(
        double tau,
        double[] state,
        CartSinglePoleEquations equations)
    {
        Debug.Assert(state.Length == 4);
        _tau = tau;
        _state = state;
        _equations = equations;
    }

    /// <summary>
    /// Update the model state. I.e. move the state forward by one timestep.
    /// </summary>
    /// <param name="f">The external horizontal force applied to the cart.</param>
    /// <remarks>This implementation of Update() uses Euler's method, this is somewhat inaccurate especially for larger timesteps.</remarks>
    public virtual void Update(double f)
    {
        _equations.CalcAccelerations(_state, f, out double xa, out double thetaa);

        // Update cart and pole positions based on current cart and pole velocities.
        _state[0] = Math.FusedMultiplyAdd(_state[1], _tau, _state[0]);
        _state[2] = Math.FusedMultiplyAdd(_state[3], _tau, _state[2]);

        // Update cart and pole velocities at next timestep based on current cart and pole accelerations.
        _state[1] = Math.FusedMultiplyAdd(xa, _tau, _state[1]);
        _state[3] = Math.FusedMultiplyAdd(thetaa, _tau, _state[3]);
    }
}
