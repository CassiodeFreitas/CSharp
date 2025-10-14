using System;
using System.IO;
using System.Collections.Generic;
using ScottPlot;

class Program
{
    static void Main()
    {
        Console.WriteLine("Time(s)\tPos(m)\tVel(m/s)\tAcc(m/s^2)");

        // Create a vehicle instance
        Vehicle car = new Vehicle();

        double dt = 0.10; // timestep (s)
        double tEnd = 5.00; // simulation time (s)

        // Create lists to store results
        var timeList = new List<double>();
        var posList = new List<double>();
        var velList = new List<double>();
        var accList = new List<double>();

        for (double t = 0; t <= tEnd; t += dt)
        {
            // Update vehicle state
            car.UpdateKinematics(t, dt);

            // Save data
            timeList.Add(t);
            posList.Add(car.x);
            velList.Add(car.v);
            accList.Add(car.a);

            // Print every 0.5s
            if (Math.Abs(t % 0.5) < 1e-9)
                Console.WriteLine("{0,6:F1}\t{1,10:F3}\t{2,10:F3}\t{3,10:F3}", t, car.x, car.v, car.a);
        }

        // Plot velocity
        var plt = new ScottPlot.Plot();
        plt.Add.Scatter(timeList.ToArray(), velList.ToArray());
        plt.Title("Velocity vs Time");
        plt.XLabel("Time (s)");
        plt.YLabel("Velocity (m/s)");
        
        plt.SavePng("velocity_plot.png", 600, 400); // <-- Save the plot as an image
        Console.WriteLine("Plot saved as velocity_plot.png!");

        // Write results to CSV
        string file = "sim_data.csv";
        using (StreamWriter writer = new StreamWriter(file))
        {
            writer.WriteLine("Time, Position, Velocity, Acceleration");
            for (int i = 0; i < timeList.Count; i++)
            {
                writer.WriteLine($"{timeList[i]:F2}, {posList[i]:F4}, {velList[i]:F4}, {accList[i]:F4}");
            }
        }

        Console.WriteLine("\nSimulation Complete!");
    }
}

class Vehicle
{
    public double x { get; private set; } = 0.00; // position (m)
    public double v { get; private set; } = 0.00; // velocity (m/s)
    public double a { get; private set; } = 0.00; // acceleration (m/s^2)

    public void UpdateKinematics(double t, double dt)
    {
        a = AccelerationProfile(t);
        v += (a - 0.1 * v) * dt;
        x += v * dt;
    }

    private double AccelerationProfile(double t)
    {
        if (t < 2)
            return 2;
        else
            return -1;
    }
}
