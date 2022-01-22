using System;

namespace CartPolePhysics.SinglePole.DoublePrecision
{
    /// <summary>
    /// Encapsulates the equations of motion for the cart-pole model with a single pole.
    /// See https://sharpneat.sourceforge.io/research/cart-pole/cart-pole-equations.html for a derivation and exploration of these equations.
    /// </summary>
    public class CartSinglePoleEquations
    {
        #region Instance Fields

        /// <summary>
        /// Gravitational acceleration (in m/s^2). Here g is taken to be the direction-less magnitude of the acceleration
        /// caused by gravity (i.e. approximately 9.8 m/s^2 for gravity on Earth). The direction of gravitational acceleration
        /// is taken into account in the formulation of the equations, therefore the sign of g is positive.
        /// </summary>
        readonly double g = 9.8;
        /// <summary>
        /// Mass of the pole (in kilograms).
        /// </summary>
        readonly double m = 0.1;
        /// <summary>
        /// Mass of the cart (in kilograms).
        /// </summary>
        readonly double m_c = 1.0;
        /// <summary>
        /// Length of the pole (in metres). This is the full length of the pole, and not the half length as used widely 
        /// elsewhere in the literature.
        /// </summary>
        readonly double l = 1;
        /// <summary>
        /// Half of the pole's length.
        /// </summary>
        readonly double l_hat;
        /// <summary>
        /// Coefficient of friction between the pole and the cart, i.e. friction at the pole's pivot joint.
        /// </summary>
        readonly double mu_p = 0.001;
        /// <summary>
        /// Coefficient of friction between the cart and the track.
        /// </summary>
        readonly double mu_c = 0.1;
        /// <summary>
        /// Combined mass of the cart and the pole.
        /// </summary>
        readonly double M;

        #endregion

        #region Constructors

        /// <summary>
        /// Construct with the default cart-pole model parameters.
        /// </summary>
        public CartSinglePoleEquations()   
        {
            M = m + m_c;
            l_hat = l / 2.0;
        }

        /// <summary>
        /// Construct with the provided cart-pole model parameters.
        /// </summary>
        public CartSinglePoleEquations(
            double g,
            double m,
            double m_c,
            double l,
            double mu_p,
            double mu_c)
        {
            this.g = g;
            this.m = m;
            this.m_c = m_c;
            this.l = l;
            this.mu_p = mu_p;
            this.mu_c = mu_c;
            this.M = this.m + this.m_c;
            l_hat = l / 2.0;
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
            double[] state,
            double f,
            out double xa,
            out double thetaa)
        {
            // Note. This code is primarily written for clarity rather than execution speed, hence it is probably amenable to being optimised somewhat.

            // Extract state into named variables.
            double xv = state[1];
            double theta = state[2];
            double thetav = state[3];

            // Precompute some reused values.
            double sin_theta = Math.Sin(theta);
            double cos_theta = Math.Cos(theta);
            double cos_theta_sqr = cos_theta * cos_theta;
            double thetav_sqr = thetav * thetav;

            // Calc cart horizontal acceleration.
            xa = (m*g*sin_theta*cos_theta - (7.0/3.0)*(f + m*l_hat * thetav_sqr * sin_theta - mu_c*xv) - ((mu_p*thetav*cos_theta)/l_hat)) / (m*cos_theta_sqr - (7.0/3.0)*M);

            // Calc pole angular acceleration.
            thetaa = (3.0/(7.0*l_hat)) * (g*sin_theta - xa*cos_theta - (mu_p * thetav)/(m*l_hat));
        }

        #endregion
    }
}
