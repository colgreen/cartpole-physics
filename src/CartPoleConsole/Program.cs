using System;
using System.IO;

namespace CartPoleConsole
{
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("Cart and Pole Model Simulator");

            // Initialise the simulation.
            var sim = InitSim_DoublePole();

            // Run the simulation.
            sim.Run();

            // Save the recorded model state at each timestep.
            Save(
                "theta-RK4-tau16th.csv",
                sim.TimeSeries,
                sim.XSeries,
                sim.Theta1Series,
                sim.Theta2Series);
        }

        #region Private Methods [Single-Pole Double-Precision Floating-point Maths]

        private static SinglePole.DoublePrecision.CartSinglePoleSimulator InitSim_SinglePole()
        {
            // Initialise the simulation.
            double[] state = new double[4];
            state[2] = Math.PI / 2.0; // theta = 90 degrees.

            var sim = new SinglePole.DoublePrecision.CartSinglePoleSimulator(
                15.0,
                new CartPolePhysics.SinglePole.DoublePrecision.CartSinglePolePhysicsRK4(0.01, state));

            return sim;
        }

        private static void Save(
            string filename,
            double[] t_series,
            double[] theta_series)
        {
            using FileStream fs = File.Create(filename);
            using var sw = new StreamWriter(fs);

            sw.WriteLine("time,theta");

            for(int i=0; i < t_series.Length; i++)
            {
                sw.WriteLine($"{t_series[i]:N3},{theta_series[i]}");
            }
        }

        #endregion

        #region Private Methods [Single-Pole Single-Precision Floating-point Maths]

        private static SinglePole.SinglePrecision.CartSinglePoleSimulator InitSim_SinglePole_SinglePrecision()
        {
            // Initialise the simulation.
            float[] state = new float[4];
            state[2] = MathF.PI / 2f; // theta = 90 degrees.

            var sim = new SinglePole.SinglePrecision.CartSinglePoleSimulator(
                15f,
                new CartPolePhysics.SinglePole.SinglePrecision.CartSinglePolePhysicsRK4(1.0f / 6.0f, state));

            return sim;
        }

        private static void Save(
            string filename,
            float[] t_series,
            float[] theta_series)
        {
            using FileStream fs = File.Create(filename);
            using var sw = new StreamWriter(fs);

            sw.WriteLine("time,theta");

            for(int i = 0; i < t_series.Length; i++)
            {
                sw.WriteLine($"{t_series[i]:N3},{theta_series[i]}");
            }
        }

        #endregion

        #region Private Methods [Double-Pole Double-Precision Floating-point Maths]

        private static DoublePole.DoublePrecision.CartDoublePoleSimulator InitSim_DoublePole()
        {
            // Initialise the simulation.
            double[] state = new double[6];
            state[2] = Math.PI;
            state[4] = Math.PI;

            var sim = new DoublePole.DoublePrecision.CartDoublePoleSimulator(
                15.0,
                new CartPolePhysics.DoublePole.DoublePrecision.CartDoublePolePhysicsRK4(1f/16f, state));

            return sim;
        }

        private static void Save(
            string filename,
            double[] t_series,
            double[] x_series,
            double[] theta1_series,
            double[] theta2_series)
        {
            using FileStream fs = File.Create(filename);
            using var sw = new StreamWriter(fs);

            sw.WriteLine("time,x,theta1,theta2");

            for(int i = 0; i < t_series.Length; i++)
            {
                sw.WriteLine($"{t_series[i]:N3},{x_series[i]},{theta1_series[i]},{theta2_series[i]}");
            }
        }

        #endregion
    }
}
