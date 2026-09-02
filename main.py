#Main program for the 2D launch-to-orbit sim. Runs a fixed, staged rocket configuration
#through RK4 integration, logs telemetry to CSV, and produces trajectory/velocity/speed
#plots plus an animated, phase-colored trajectory replay.

from rocket import Rocket, FlightPhase
import matplotlib.pyplot as plt
import numpy as np
import csv
from matplotlib.animation import FuncAnimation

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
    phase_list = []



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
        phase_list.append(my_rocket.flight_phase)

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

fig, ax = plt.subplots()
ax.set_xlim(0, max(x_list))
ax.set_ylim(0, max(y_list))
point, = ax.plot([], [], 'ko', markersize=8)
ax.set_xlabel("Horizontal movement (m)")
ax.set_ylabel("Vertical movement (m)")
ax.set_title("Animated Trajectory")


#Maps each flight phase to a display color for the animated trajectory plot
phase_colors = {
    FlightPhase.PRE_LAUNCH: 'gray',
    FlightPhase.POWERED_FLIGHT: 'red',
    FlightPhase.COAST: 'blue',
    FlightPhase.ORBIT_INSERTION: 'green'
}

#matplotlib can't color a single line multiple colors, so the trajectory gets split into one segment per phase.
segments = []
start_idx = 0
for i in range(1, len(phase_list)):
    if phase_list[i] != phase_list[start_idx]:
        segments.append((start_idx, i, phase_list[start_idx]))
        start_idx = i
segments.append((start_idx, len(phase_list), phase_list[start_idx]))

#One empty line object per segment, pre-colored by phase that is filled in progressively as the animation plays
trail_lines = [ax.plot([], [], '-', linewidth=1.5, color=phase_colors[phase])[0] for _, _, phase in segments]

#dt is 0.1s so only animating every 20th point
frame_skip = 20


def update_frame(frame):
    i = frame * frame_skip
    point.set_data([x_list[i]], [y_list[i]])

    for (start, end, phase), line in zip(segments, trail_lines):
        if i > start:
            actual_end = min(i, end)
            line.set_data(x_list[start:actual_end], y_list[start:actual_end])

    return point, *trail_lines


num_frames = len(x_list) // frame_skip
ani = FuncAnimation(fig, update_frame, frames=num_frames, interval=20)
plt.show()
