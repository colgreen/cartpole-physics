﻿using System;
using System.Drawing;
using System.Windows.Forms;
using CartPolePhysics.SinglePole.DoublePrecision;
using ZedGraph;

namespace CartPoleWinForms
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
            Init();
        }

        private void Init()
        {
            double[] state = new double[4];
            state[2] = Math.PI / 2.0; // theta = 90 degrees.
            var physics = new CartSinglePolePhysicsRK4(0.01, state);

            const int durationSecs = 15;
            int steps = (int)(durationSecs / physics.Tau);

            double[] t_series = new double[steps];
            double[] x_series = new double[steps];
            double[] xv_series = new double[steps];
            double[] theta_series = new double[steps];

            RunSimulation(
                physics,
                t_series,
                x_series,
                xv_series,
                theta_series);

            PointPairList x_ppl = new PointPairList(t_series, x_series);
            PointPairList xv_ppl = new PointPairList(t_series, xv_series);
            PointPairList theta_ppl = new PointPairList(t_series, theta_series);

            GraphPane pane = zed.GraphPane;
            pane.AddCurve("theta", theta_ppl, Color.Black, SymbolType.None);
            LineItem curveX = pane.AddCurve("x", x_ppl, Color.Red, SymbolType.None);
            curveX.IsY2Axis = true;
            pane.Y2Axis.IsVisible = true;
            pane.YAxis.Scale.Min = 1;
            pane.YAxis.Scale.Max = 5;
            pane.Y2Axis.Scale.Min = 0;
            pane.Y2Axis.Scale.Max = 0.025;
            pane.Title.IsVisible = false;
            pane.XAxis.Title.Text = "Time (seconds)";
            pane.YAxis.Title.Text = "Pole angle (radians)";
            pane.Y2Axis.Title.Text = "Cart position (metres)";

            LineItem curveXV = pane.AddCurve("xv", xv_ppl, Color.Blue, SymbolType.None);

            zed.AxisChange();
        }

        private static void RunSimulation(
            CartSinglePolePhysics physics,
            double[] t_series,
            double[] x_series,
            double[] xv_series,
            double[] theta_series)
        {
            double t = 0.0;

            // Record initial state.
            t_series[0] = t;
            x_series[0] = physics.State[0];
            xv_series[0] = physics.State[1];
            theta_series[0] = physics.State[2];

            for(int timestep=0; timestep < t_series.Length; timestep++, t += physics.Tau)
            {
                // Update model state.
                physics.Update(0.0);

                // Record state.
                t_series[timestep] = t;
                x_series[timestep] = physics.State[0];
                xv_series[timestep] = physics.State[1];
                theta_series[timestep] = physics.State[2];
            }
        }
    }
}
