#This is the main program for the 1D launch-to-orbit sim. It will prompt the user for starting numbers then start itself

from rocket import Rocket
import matplotlib.pyplot as plt
import numpy as np

print("Welcome to the Launch-to-Orbit Simulator!\n")
mass = float(input("What is the total mass of your rocket?\n"))
dry_mass = float(input("What is the dry mass of your rocket? (the total mass of the rocket minus the mass of the propellant)\n"))
exhaust_velocity = float(input("What is the exhaust velocity of your rocket?\n"))
mass_flow_rate = float(input("What is the mass flow rate of your rocket?\n"))

dt = 0.1
time = 0
time_list = []
x_list = []
y_list = []
vx_list = []
vy_list = []
speed_list = []


my_rocket = Rocket(mass,dry_mass,exhaust_velocity,mass_flow_rate)

#Main loop that loops over each delta time interval of 0.1s, breaks out of loop when rocket comes back to the ground.
#The time and heights at each step are stored in respective lists
while True:
    my_rocket.rk4_stepper()
    vx_list.append(my_rocket.velocity[0])
    vy_list.append(my_rocket.velocity[1])
    speed_list.append(np.linalg.norm(my_rocket.velocity))
    time += dt
    time_list.append(time)
    x_list.append(my_rocket.position[0])
    y_list.append(my_rocket.position[1])
    if my_rocket.position [1] <= 0:
        break

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
