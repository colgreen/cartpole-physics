using System;

namespace CartPolePhysics.DoublePole.SinglePrecision
{
    /// <summary>
    /// Encapsulates the equations of motion for the cart-pole model with two poles.
    /// See https://sharpneat.sourceforge.io/research/cart-pole/cart-pole-equations.html for a derivation and exploration of these equations.
    /// </summary>
    public class CartDoublePoleEquations
    {
        #region Instance Fields

        /// <summary>
        /// Gravitational acceleration (in m/s^2). Here g is taken to be the directionless magnitude of the acceleration
        /// caused by gravity (i.e. approximately 9.8 m/s^2 for gravity on Earth). The direction of gravitational acceleration
        /// is taken into account in the formulation of the equations, therefore the sign of g is positive.
        /// </summary>
        readonly float g = 9.8f;
        /// <summary>
        /// Mass of pole 1 (in kilograms).
        /// </summary>
        readonly float m = 0.1f;
        /// <summary>
        /// Mass of pole 2 (in kilograms).
        /// </summary>
        readonly float m2 = 0.01f;
        /// <summary>
        /// Mass of the cart (in kilograms).
        /// </summary>
        readonly float m_c = 1f;
        /// <summary>
        /// Length of pole 1 (in metres). This is the full length of the pole, and not the half length as used widely 
        /// elsewhere in the literature.
        /// </summary>
        readonly float l = 1f;
        /// <summary>
        /// Half length of pole 1.
        /// </summary>
        readonly float l_hat;
        /// <summary>
        /// Length of pole 2 (in metres). This is the full length of the pole, and not the half length as used widely 
        /// elsewhere in the literature.
        /// </summary>
        readonly float l2 = 0.1f;
        /// <summary>
        /// Half length of pole 2.
        /// </summary>
        readonly float l2_hat;
        /// <summary>
        /// Coefficient of friction between the pole and the cart, i.e. friction at the pole's pivot joint.
        /// </summary>
        readonly float mu_p = 0.001f;
        /// <summary>
        /// Coefficient of friction between the cart and the track.
        /// </summary>
        readonly float mu_c = 0.1f;
        /// <summary>
        /// Combined mass of the cart and the two poles.
        /// </summary>
        readonly float M;

        #endregion

        #region Constructors

        /// <summary>
        /// Construct with the default cart-pole model parameters.
        /// </summary>
        public CartDoublePoleEquations()   
        {
            M = m + m2 + m_c;
            l_hat = l / 2f;
            l2_hat = l2 / 2f;
        }

        /// <summary>
        /// Construct with the provided cart-pole model parameters.
        /// </summary>
        public CartDoublePoleEquations(
            float g,
            float m,
            float m2,
            float m_c,
            float l,
            float l2,
            float mu_p,
            float mu_c)
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

        #endregion

        #region Public Methods

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
            float[] state,
            float f,
            out float xa,
            out float thetaa1,
            out float thetaa2)
        {
            // Note. This code is primarily written for clarity rather than execution speed, hence it is probably amenable to being optimised somewhat.

            // Extract state into named variables.
            float xv = state[1];
            float theta = state[2];
            float thetav = state[3];
            float theta2 = state[4];
            float thetav2 = state[5];

            // Precompute some reused values (pole 1).
            float sin_theta = MathF.Sin(theta);
            float cos_theta = MathF.Cos(theta);
            float cos_theta_sqr = cos_theta * cos_theta;
            float thetav_sqr = thetav * thetav;

            // Precompute some reused values (pole 2).
            float sin_theta2 = MathF.Sin(theta2);
            float cos_theta2 = MathF.Cos(theta2);
            float cos_theta2_sqr = cos_theta2 * cos_theta2;
            float thetav2_sqr = thetav2 * thetav2;

            // Calc cart horizontal acceleration.
            xa = (g * ((m*sin_theta*cos_theta) + (m2*sin_theta2*cos_theta2))
                - (7f/3f) * (f +(m*l_hat*thetav_sqr*sin_theta) + (m2*l2_hat*thetav2_sqr*sin_theta2) - mu_c*xv)
                - (((mu_p*thetav*cos_theta)/l_hat) + ((mu_p*thetav2*cos_theta2)/l2_hat)))
                / ((m*cos_theta_sqr) + (m2*cos_theta2_sqr) - (7/3)*M);

            // Calc pole 1 angular acceleration.
            thetaa1 = (3f/(7f*l_hat)) * (g*sin_theta - xa*cos_theta - ((mu_p * thetav)/(m*l_hat)));

            // Calc pole 2 angular acceleration.
            thetaa2 = (3f/(7f*l2_hat)) * (g*sin_theta2 - xa*cos_theta2 - ((mu_p * thetav2)/(m2*l2_hat)));
        }

        #endregion
    }
}
