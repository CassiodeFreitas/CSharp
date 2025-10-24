using ScottPlot;
using System;
using System.Collections.Generic;
using System.Windows.Forms;

namespace BicycleModelPlot
{
    public partial class Form1 : Form
    {
        private Vehicle Car = new Vehicle();
        private List<double> timeList = new List<double>();
        private List<double> ayList = new List<double>();
        private double t = 0;
        private double dt = 0.05;
        private System.Windows.Forms.Timer simTimer = new System.Windows.Forms.Timer();

        public Form1()
        {
            InitializeComponent();

            // Configure timer
            simTimer.Interval = 50; // ms
            simTimer.Tick += SimTimer_Tick;
            simTimer.Start();
        }

        private void SimTimer_Tick(object sender, EventArgs e)
        {
            // Simple simulation step
            Car.UpdateCarStates(t, dt);
            t += dt;

            // Record results
            timeList.Add(t);
            ayList.Add(Car.ay);

            // Update live plot
            formsPlot1.Plot.Clear();
            formsPlot1.Plot.Add.Scatter(timeList.ToArray(), ayList.ToArray());
            formsPlot1.Plot.Title("Lateral Acceleration vs Time");
            formsPlot1.Plot.XLabel("Time (s)");
            formsPlot1.Plot.YLabel("ay (m/s²)");
            formsPlot1.Refresh();
        }
    }

    // Example simplified vehicle class
    public class Vehicle
    {
        public double ay = 0;
        private double omega = 0.5;

        public void UpdateCarStates(double t, double dt)
        {
            // Simulate some varying lateral acceleration
            ay = 2 * Math.Sin(omega * t);
        }
    }
}
