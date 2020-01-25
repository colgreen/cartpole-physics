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
            var sim = InitSim();

            // Run the simulation.
            sim.Run();

            // Save the recorded model state at each timestep.
            Save(
                "theta-RK4-tau0_01.csv",
                sim.TimeSeries,
                sim.ThetaSeries);
        }

        #region Private Methods [Double Precision Floating-point Maths]

        private static SinglePole.DoublePrecision.CartSinglePoleSimulator InitSim()
        {
            // Initialise the simulation.
            double[] state = new double[4];
            state[1] = Math.PI / 2.0; // theta = 90 degrees.

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
            using(FileStream fs = File.Create(filename))
            using(var sw = new StreamWriter(fs))
            {
                sw.WriteLine("time,theta");

                for(int i=0; i < t_series.Length; i++)
                {
                    sw.WriteLine($"{t_series[i]:N3},{theta_series[i]}");
                }
            }
        }

        #endregion

        #region Private Methods [Single Precision Floating-point Maths]

        private static SinglePole.SinglePrecision.CartSinglePoleSimulator InitSim_SinglePrecision()
        {
            // Initialise the simulation.
            float[] state = new float[4];
            state[1] = MathF.PI / 2f; // theta = 90 degrees.

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
            using(FileStream fs = File.Create(filename))
            using(var sw = new StreamWriter(fs))
            {
                sw.WriteLine("time,theta");

                for(int i = 0; i < t_series.Length; i++)
                {
                    sw.WriteLine($"{t_series[i]:N3},{theta_series[i]}");
                }
            }
        }

        #endregion
    }
}
