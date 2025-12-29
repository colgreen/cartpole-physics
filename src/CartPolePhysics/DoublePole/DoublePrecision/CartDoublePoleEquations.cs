using System;

namespace CartPolePhysics.DoublePole.DoublePrecision;

/// <summary>
/// Encapsulates the equations of motion for the cart-pole model with two poles.
/// See https://sharpneat.sourceforge.io/research/cart-pole/cart-pole-equations.html for a derivation and exploration of these equations.
/// </summary>
public class CartDoublePoleEquations
{
    /// <summary>
    /// Gravitational acceleration (in m/s^2). Here g is taken to be the directionless magnitude of the acceleration
    /// caused by gravity (i.e. approximately 9.8 m/s^2 for gravity on Earth). The direction of gravitational acceleration
    /// is taken into account in the formulation of the equations, therefore the sign of g is positive.
    /// </summary>
    readonly double g = 9.8;
    /// <summary>
    /// Mass of pole 1 (in kilograms).
    /// </summary>
    readonly double m = 0.1;
    /// <summary>
    /// Mass of pole 2 (in kilograms).
    /// </summary>
    readonly double m2 = 0.01;
    /// <summary>
    /// Mass of the cart (in kilograms).
    /// </summary>
    readonly double m_c = 1.0;
    /// <summary>
    /// Length of pole 1 (in metres). This is the full length of the pole, and not the half length as used widely 
    /// elsewhere in the literature.
    /// </summary>
    readonly double l = 1.0;
    /// <summary>
    /// Half length of pole 1.
    /// </summary>
    readonly double l_hat;
    /// <summary>
    /// Length of pole 2 (in metres). This is the full length of the pole, and not the half length as used widely 
    /// elsewhere in the literature.
    /// </summary>
    readonly double l2 = 0.1;
    /// <summary>
    /// Half length of pole 2.
    /// </summary>
    readonly double l2_hat;
    /// <summary>
    /// Coefficient of friction between the pole and the cart, i.e. friction at the pole's pivot joint.
    /// </summary>
    readonly double mu_p = 0.001;
    /// <summary>
    /// Coefficient of friction between the cart and the track.
    /// </summary>
    readonly double mu_c = 0.1;
    /// <summary>
    /// Combined mass of the cart and the two poles.
    /// </summary>
    readonly double M;

    /// <summary>
    /// Construct with the default cart-pole model parameters.
    /// </summary>
    public CartDoublePoleEquations()   
    {
        M = m + m2 + m_c;
        l_hat = l / 2.0;
        l2_hat = l2 / 2.0;
    }

    /// <summary>
    /// Construct with the provided cart-pole model parameters.
    /// </summary>
    public CartDoublePoleEquations(
        double g,
        double m,
        double m2,
        double m_c,
        double l,
        double l2,
        double mu_p,
        double mu_c)
    {
        this.g = g;
        this.m = m;
        this.m2 = m2;
        this.m_c = m_c;
        this.l = l;
        this.l2 = l2;
        this.mu_p = mu_p;
        this.mu_c = mu_c;
        this.M = this.m + this.m2 + this.m_c;
        l_hat = l / 2f;
        l2_hat = l2 / 2f;
    }

    /// <summary>
    /// Calculate cart acceleration, and pole angular acceleration for a given model state and external horizontal force applied to the cart.
    /// </summary>
    /// <param name="state">The cart-pole model state. The model state variables are:
    ///  [0] x-axis coordinate of the cart (metres).
    ///  [1] x-axis velocity of the cart (m/s).
    ///  [2] Pole 1 angle (radians). Clockwise deviation from the vertical.
    ///  [3] Pole 1 angular velocity (radians/s). Positive is clockwise.
    ///  [4] Pole 2 angle (radians). Clockwise deviation from the vertical.
    ///  [5] Pole 2 angular velocity (radians/s). Positive is clockwise.
    /// <param name="f">The external horizontal force applied to the cart.</param>
    /// <param name="xa">Returns the cart's horizontal acceleration.</param>
    /// <param name="thetaa1">Returns pole 1's angular acceleration.</param>
    /// <param name="thetaa2">Returns pole 2's angular acceleration.</param>
    public void CalcAccelerations(
        double[] state,
        double f,
        out double xa,
        out double thetaa1,
        out double thetaa2)
    {
        // Note. This code is primarily written for clarity rather than execution speed, hence it is probably amenable to being optimised somewhat.

        // Extract state into named variables.
        double xv = state[1];
        double theta = state[2];
        double thetav = state[3];
        double theta2 = state[4];
        double thetav2 = state[5];

        // Precompute some reused values (pole 1).
        double sin_theta = Math.Sin(theta);
        double cos_theta = Math.Cos(theta);
        double cos_theta_sqr = cos_theta * cos_theta;
        double thetav_sqr = thetav * thetav;

        // Precompute some reused values (pole 2).
        double sin_theta2 = Math.Sin(theta2);
        double cos_theta2 = Math.Cos(theta2);
        double cos_theta2_sqr = cos_theta2 * cos_theta2;
        double thetav2_sqr = thetav2 * thetav2;

        // Calc cart horizontal acceleration.
        xa = (g * ((m*sin_theta*cos_theta) + (m2*sin_theta2*cos_theta2))
            - (7.0/3.0) * (f +(m*l_hat*thetav_sqr*sin_theta) + (m2*l2_hat*thetav2_sqr*sin_theta2) - mu_c*xv)
            - (((mu_p*thetav*cos_theta)/l_hat) + ((mu_p*thetav2*cos_theta2)/l2_hat)))
            / ((m*cos_theta_sqr) + (m2*cos_theta2_sqr) - (7.0/3.0)*M);

        // Calc pole 1 angular acceleration.
        thetaa1 = (3.0/(7.0*l_hat)) * (g*sin_theta - xa*cos_theta - ((mu_p * thetav)/(m*l_hat)));

        // Calc pole 2 angular acceleration.
        thetaa2 = (3.0/(7.0*l2_hat)) * (g*sin_theta2 - xa*cos_theta2 - ((mu_p * thetav2)/(m2*l2_hat)));
    }
}
