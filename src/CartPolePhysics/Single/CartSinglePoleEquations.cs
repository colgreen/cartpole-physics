using System;

namespace CartPolePhysics.Single
{
    /// <summary>
    /// Encapsulates the equations of motion for the cart-pole model with a single pole.
    /// See https://sharpneat.sourceforge.io/research/cart-pole/cart-pole-equations.html for a derivation and exploration of these equations.
    /// </summary>
    public class CartSinglePoleEquations
    {
        #region Instance Fields

        /// <summary>
        /// Gravitational acceleration (in m/s^2). Here g is taken to be the directionless magnitude of the acceleration
        /// caused by gravity (i.e. approximately 9.8 m/s^2 for gravity on Earth). The direction of gravitational acceleration
        /// is taken into account in the formulation of the equations, therefore the sign of g is positive.
        /// </summary>
        readonly float g = 9.8f;
        /// <summary>
        /// Mass of the pole (in kilograms).
        /// </summary>
        readonly float m = 0.1f;
        /// <summary>
        /// Mass of the cart (in kilograms).
        /// </summary>
        readonly float m_c = 10f;
        /// <summary>
        /// Length of the pole (in metres). This is the full length of the pole, and not the half length as used widely 
        /// elsewhere in the literature.
        /// </summary>
        readonly float l = 1;
        /// <summary>
        /// Half of the pole's length.
        /// </summary>
        readonly float l_hat;
        /// <summary>
        /// Coefficient of friction between the pole and the cart, i.e. friction at the pole's pivot joint.
        /// </summary>
        readonly float mu_p = 0.001f;
        /// <summary>
        /// Coefficient of friction between the cart and the track.
        /// </summary>
        readonly float mu_c = 0.1f;
        /// <summary>
        /// Combined mass of the cart and the pole.
        /// </summary>
        readonly float M;

        #endregion

        #region Constructors

        /// <summary>
        /// Construct with the default cart-pole model parameters.
        /// </summary>
        public CartSinglePoleEquations()   
        {
            M = m + m_c;
            l_hat = l / 2f;
        }

        /// <summary>
        /// Construct with the provided cart-pole model parameters.
        /// </summary>
        public CartSinglePoleEquations(
            float g,
            float m,
            float m_c,
            float l,
            float mu_p,
            float mu_c)
        {
            this.g = g;
            this.m = m;
            this.m_c = m_c;
            this.l = l;
            this.mu_p = mu_p;
            this.mu_c = mu_c;
            this.M = this.m + this.m_c;
            l_hat = l / 2f;
        }

        #endregion

        #region Public Methods

        /// <summary>
        /// Calculate cart acceleration, and pole angular acceleration for a given model state and external horizontal force applied to the cart.
        /// </summary>
        /// <param name="state">The cart-pole model state. The model state variables are:
        ///  [0] x-axis coordinate of the cart (metres).
        ///  [1] x-axis velocity of the cart (m/s).
        ///  [2] Pole angle (radians). Clockwise deviation from the vertical.
        ///  [3] Pole angular velocity (radians/s). Positive is clockwise.</param>
        /// <param name="f">The external horizontal force applied to the cart.</param>
        /// <param name="xa">Returns the cart's horizontal acceleration.</param>
        /// <param name="thetaa">Returns the pole's angular acceleration.</param>
        public void CalcAccelerations(
            float[] state,
            float f,
            out float xa,
            out float thetaa)
        {
            // Note. This code is primarily written for clarity rather than execution speed, hence it is probably amenable to being optimised somewhat.

            // Extract state into named variables.
            float xv = state[1];
            float theta = state[2];
            float thetav = state[3];

            // Precompute some reused values.
            float sin_theta = MathF.Sin(theta);
            float cos_theta = MathF.Cos(theta);
            float cos_theta_sqr = cos_theta * cos_theta;
            float thetav_sqr = thetav * thetav;

            // Calc cart horizontal acceleration.
            xa = (m*g*sin_theta*cos_theta - (7f/3f)*(f + m*l_hat * thetav_sqr * sin_theta - mu_c*xv) - (mu_p*thetav*cos_theta)/l_hat) / (m*cos_theta_sqr - (7f/3f)*M);

            // Calc pole angular acceleration.
            thetaa = (3f/(7f*l_hat)) * (g*sin_theta - xa*cos_theta - (mu_p * thetav)/(m*l_hat));
        }

        #endregion
    }
}
