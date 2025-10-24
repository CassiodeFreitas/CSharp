using ScottPlot;
using System;
using System.Collections.Generic;
using System.Windows.Forms;

namespace VehicleSimPlot
{
    public partial class Form1 : Form
    {
        // --- Simulation objects and data ---
        private Vehicle Car = new Vehicle();
        private List<double> timeList = new List<double>();
        private List<double> gLatList = new List<double>();

        // Simulation parameters
        private double t = 0;
        private double dt = 0.01;
        private const double Vx = 120 / 3.6;
        private const double SA = 11 / 10.0; // steering angle
        private const double simEnd = 20;

        // Timer for live updates
        private Timer simTimer = new Timer();

        public Form1()
        {
            InitializeComponent();

            // Timer setup
            simTimer.Interval = 20; // ms update rate (~50 Hz)
            simTimer.Tick += SimTimer_Tick;
            simTimer.Start();

            // Initial plot setup
            formsPlot1.Plot.Title("Vehicle Lateral Acceleration (ay)");
            formsPlot1.Plot.XLabel("Time (s)");
            formsPlot1.Plot.YLabel("Lateral Accel (m/s²)");
        }

        private void SimTimer_Tick(object sender, EventArgs e)
        {
            // Run a few steps per timer tick
            for (int i = 0; i < 10; i++)
            {
                if (t > simEnd)
                {
                    simTimer.Stop();
                    MessageBox.Show("Simulation complete!");
                    return;
                }

                Car.UpdateAero(Vx, dt);
                Car.UpdateSlipAngles(Vx, SA, dt);
                Car.UpdateTyreForces(SA, dt);
                Car.UpdateCarStates(Vx, SA, dt);

                // Save data
                timeList.Add(t);
                gLatList.Add(Car.ay);

                t += dt;
            }

            // --- Live plot update ---
            formsPlot1.Plot.Clear();
            formsPlot1.Plot.Add.Scatter(timeList.ToArray(), gLatList.ToArray());
            formsPlot1.Refresh();
        }
    }

    // --- Your existing Vehicle class below ---
    public class Vehicle
    {
        public Constants Const { get; set; } = new Constants();
        public Chassis ChassisModel { get; set; } = new Chassis();
        public Aero AeroModel { get; set; } = new Aero();
        public FrontTyre Front { get; set; } = new FrontTyre();
        public RearTyre Rear { get; set; } = new RearTyre();

        public double FzF, FzR, FzFAero, FzRAero;
        public double aSlipF, aSlipR;
        public double FyF, FyR;
        public double ay, Vy, Y, dnYaw, nYaw, aYaw;

        public void UpdateAero(double Vx, double dt)
        {
            FzFAero = 0.5 * (ChassisModel.b / ChassisModel.L) * AeroModel.A * AeroModel.cz * AeroModel.rho * Vx * Vx;
            FzRAero = 0.5 * (ChassisModel.a / ChassisModel.L) * AeroModel.A * AeroModel.cz * AeroModel.rho * Vx * Vx;
        }

        public void UpdateTyreForces(double SA, double dt)
        {
            FzF = FzFAero + ChassisModel.m * Const.g * (ChassisModel.b / ChassisModel.L);
            FzR = FzRAero + ChassisModel.m * Const.g * (ChassisModel.a / ChassisModel.L);
            FyF = aSlipF * Math.Sin(Front.CF * Math.Atan(Front.BF * SA - Front.EF * (Front.BF * SA - Math.Atan(Front.BF * SA)))) * (FzF * (Front.a1F + Front.a2F)) * -1;
            FyR = aSlipR * Math.Sin(Rear.CR * Math.Atan(Rear.BR * SA - Rear.ER * (Rear.BR * SA - Math.Atan(Rear.BR * SA)))) * (FzR * (Rear.a1R + Rear.a2R)) * -1;
        }

        public void UpdateSlipAngles(double Vx, double SA, double dt)
        {
            aSlipF = 57.3 * (Vy + nYaw * ChassisModel.a) / Vx - SA;
            aSlipR = 57.3 * (Vy - nYaw * ChassisModel.b) / Vx;
        }

        public void UpdateCarStates(double Vx, double SA, double dt)
        {
            dnYaw = (ChassisModel.a * Math.Cos(SA / 57.3) * FyF - ChassisModel.b * FyR) / ChassisModel.Izz;
            nYaw += dnYaw * dt;
            aYaw += nYaw * dt;
            ay = (Math.Cos(SA / 57.3) * FyF + FyR) / ChassisModel.m;
            Vy += (ay - Vx * nYaw) * dt;
            Y += Vy * dt;
        }

        public class Constants { public double g = 9.81; }
        public class Chassis { public double m = 850; public double Izz = 1500; public double L = 3.5; public double a = 1.8; public double b = 1.7; }
        public class Aero { public double A = 1.25; public double cz = 1; public double rho = 1.225; }
        public class FrontTyre { public double CF = 1.8391; public double BF = 0.2719; public double EF = -2.5276; public double a1F = -1e-5; public double a2F = 1.25; }
        public class RearTyre { public double CR = 1.7631; public double BR = 0.3609; public double ER = -1.989; public double a1R = -1e-5; public double a2R = 1.25; }
    }
}
