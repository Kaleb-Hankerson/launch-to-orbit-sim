#This is the main program for the 2D launch-to-orbit sim. Simulates a fixed, staged rocket configuration,
#logs telemetry to CSV, and plots trajectory/velocity/speed results.

from rocket import Rocket
import matplotlib.pyplot as plt
import numpy as np
import csv

with open("telemetry.csv", "w", newline='') as csv_file:
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["Time", "X position", "Y position", "Velocity(x)", "Velocity(y)", "Speed", "Mass","Accel(x)", "Accel(y)", "Q"])

    my_rocket = Rocket()
    max_steps = 500000
    step_count = 0

    print("Welcome to the Launch-to-Orbit Simulator!\n")
    print("This version simulates a fixed, staged rocket configuration:")
    for i, stage in enumerate(my_rocket.stages, start=1):
        print(f"  Stage {i}: {stage['exhaust_velocity']} m/s exhaust velocity, "
              f"{stage['mass_flow_rate']} kg/s flow rate, {stage['propellant_mass']} kg propellant")

    dt = 0.1
    time = 0
    time_list = []
    x_list = []
    y_list = []
    vx_list = []
    vy_list = []
    speed_list = []



    # Main loop that loops over each delta time interval of 0.1s, breaks out of loop when rocket comes back to the ground.
    # The time and heights at each step are stored in respective lists
    while True:
        my_rocket.rk4_stepper()
        vx_list.append(my_rocket.velocity[0])
        vy_list.append(my_rocket.velocity[1])
        speed_list.append(np.linalg.norm(my_rocket.velocity))
        time += dt
        time_list.append(time)
        x_list.append(my_rocket.position[0])
        y_list.append(my_rocket.position[1])

        csv_writer.writerow([
            time,
            my_rocket.position[0],
            my_rocket.position[1],
            my_rocket.velocity[0],
            my_rocket.velocity[1],
            np.linalg.norm(my_rocket.velocity),
            my_rocket.mass,
            my_rocket.accel[0],
            my_rocket.accel[1],
            my_rocket.q
        ])

        step_count += 1

        if step_count % 10000 == 0:
            print(
                f"Step {step_count}: t={time:.1f}s, altitude={my_rocket.position[1]:.0f}m, mass={my_rocket.mass:.2f}, accel=({my_rocket.accel[0]:.4f}, {my_rocket.accel[1]:.4f}), phase={my_rocket.flight_phase}")

        if step_count > max_steps or my_rocket.position[1] <= 0:
            break


print(f"Final flight phase: {my_rocket.flight_phase}")
print(f"Eccentricity: {my_rocket.eccentricity}")
print(f"Period: {my_rocket.period}")


#X over Y plot
plt.plot(x_list, y_list)
plt.xlabel("Horizontal movement (m)")
plt.ylabel("Vertical movement (m)")
plt.title("X over Y plot")
plt.show()

#Velocity components over time plot
plt.plot(time_list, vx_list, label="Horizontal velocity (vx)")
plt.plot(time_list, vy_list, label="Vertical velocity (vy)")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.title("Velocity Components Over Time")
plt.legend()
plt.show()

#Speed over time plot
plt.plot(time_list, speed_list)
plt.xlabel("Time (s)")
plt.ylabel("Speed (m/s)")
plt.title("Total Speed Over Time")
plt.show()
