// classes: simulation program, vehicle, front tyre and rear tyre?, post sim results?
// inputs: vehicle speed and steer angle
// variables:
// methods: front and rear vertical force, front and rear slip angle, front and rear lateral force, vehicle state (lateral and yaw acceleration);

using System;
using System.IO;
using System.Collections.Generic;

class Simulation
{
    static void Main()
    {
        // Create a new vehicle instance (object)
        Vehicle Car = new Vehicle();

        // Define Simulation Constants
        const double Vx = 120 / 3.6; // [m/s]
        const double SA = 11; // [deg]
        const double slope = 2; // [-]
        const double simulation_time = 20; // [s]
        const double dt = 1e-3;

        // Create lists to store results
        var timeList = new List<double>();
        var gLatList = new List<double>();
        var nYawList = new List<double>();
        
        Console.WriteLine("Simulation Started\n");

        // Create Simulation Loop
        for (double t = 0; t < simulation_time; t += dt)
        {
            Car.UpdateAero(Vx, dt);
            Car.UpdateSlipAngles(Vx, SA, dt);
            Car.UpdateTyreForces(SA, dt);
            Car.UpdateCarStates(Vx, SA, dt);

            // Save data
            timeList.Add(t);
            gLatList.Add(Car.ay);
            nYawList.Add(Car.nYaw);
        }

        // Write simulation data to .csv
        string file = "sim_data.csv";
        using (StreamWriter writer = new StreamWriter(file))
        {
            writer.WriteLine("Time, Lateral Acceleration, Yaw Rate");
            for (int i = 1; i < timeList.Count; i++)
            {
                writer.WriteLine($"{timeList[i]:F2}, {gLatList[i]:F4}, {nYawList[i]:F4}");
            }
        }
        Console.WriteLine("Simulation Complete\n");
    }
}

class Vehicle
{
    // Subsystem objects
    public Constants Const { get; set; } = new Constants();
    public Chassis ChassisModel { get; set; } = new Chassis();
    public Aero AeroModel { get; set; } = new Aero();
    public FrontTyre Front { get; set; } = new FrontTyre();
    public RearTyre Rear { get; set; } = new RearTyre();
    // Vertical load results
    public double FzF { get; set; } // [N]
    public double FzR { get; set; } // [N]
    // Vertical aero loads
    public double FzFAero { get; set; } // [N]
    public double FzRAero { get; set; } // [N]
    // Slip angles
    public double aSlipF { get; set; } // [deg]
    public double aSlipR { get; set; } // [deg]
    // Tyre lateral forces
    public double FyF { get; set; } // [N]
    public double FyR { get; set; } // [N]
    // Car states
    public double ay {  get; set; } // [m/s^2]
    public double Vy { get; set; } // [m/s]
    public double Y { get; set; } // [m]
    public double dnYaw { get; set; } // [rad/s^2]
    public double nYaw { get; set; } // [rad/s]
    public double aYaw { get; set; } // [rad]


    // Update Aero
    public void UpdateAero(double Vx, double dt)
    {
        FzFAero = 0.5 * (ChassisModel.b / ChassisModel.L) * AeroModel.A * AeroModel.cz * AeroModel.rho * Vx * Vx;
        FzRAero = 0.5 * (ChassisModel.a / ChassisModel.L) * AeroModel.A * AeroModel.cz * AeroModel.rho * Vx * Vx;
    }

    // Update tyre forces
    public void UpdateTyreForces(double SA, double dt)
    {
        FzF = FzFAero + ChassisModel.m * Const.g * (ChassisModel.b / ChassisModel.L);
        FzR = FzRAero + ChassisModel.m * Const.g * (ChassisModel.a / ChassisModel.L);
        FyF = aSlipF * Math.Sin(Front.CF * Math.Atan(Front.BF * SA - Front.EF * (Front.BF * SA - Math.Atan(Front.BF * SA)))) * (FzF * Front.a1F + Front.a2F) * -1;
        FyR = aSlipF * Math.Sin(Rear.CR * Math.Atan(Rear.BR * SA - Rear.ER * (Rear.BR * SA - Math.Atan(Rear.BR * SA)))) * (FzR * Rear.a1R + Rear.a2R) * -1;
    }

    // Update slip angles
    public void UpdateSlipAngles(double Vx, double SA, double dt)
    {
        aSlipF = 57.3 * (ay - nYaw * ChassisModel.a) / Vx - SA;
        aSlipR = 57.3 * (ay - nYaw * ChassisModel.b) / Vx;
    }

    // Update car states
    public void UpdateCarStates(double Vx, double SA, double dt)
    {
        dnYaw = (ChassisModel.a * Math.Cos(SA / 57.3) * FyF - ChassisModel.b * FyR) / ChassisModel.Izz ;
        nYaw += dnYaw  * dt;
        aYaw += nYaw * dt;
        ay = (Math.Cos(SA / 57.3) * FyF + FyR) / ChassisModel.m;
        Vy += (ay - Vx * nYaw) * dt;
        Y += Vy * dt ;
    }

    // Define Vehicle Parameters
    public class Constants
    {
        public double g = 9.81; // [m/s^2]
    }

    // Create new classes for each subsystem
    public class Chassis
    {
        public double m = 850; // [kg]
        public double Izz = 1500; // [kg*m^2]
        public double L = 3.5; // [m]
        public double a = 1.8; // [m]
        public double b = 1.7; // [m]
        public double SR = 10; // [-]
    }

    public class Aero
    {
        public double A = 1.25; // [A]
        public double cz = 1; // [-];
        public double rho = 1.225; // [kg/m^3]
    }

    public class FrontTyre
    {
        public double CF = 1.8391; // [-]
        public double BF = 0.2719; // [-]
        public double EF = -2.5276; // [-]
        public double a1F = -1e-5; // [-]
        public double a2F = 1.25; // [-]
    }

    public class RearTyre
    {
        public double CR = 1.7631; // [-]
        public double BR = 0.3609; // [-]
        public double ER = -1.989; // [-]
        public double a1R = -1e-5; // [-]
        public double a2R = 1.25; // [-]
    }
}

