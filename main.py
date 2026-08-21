#This is the main program for the 1D launch-to-orbit sim. It will prompt the user for starting numbers then start itself

from rocket import Rocket
import matplotlib.pyplot as plt

print("Welcome to the Launch-to-Orbit Simulator!\n")
mass = float(input("What is the total mass of your rocket?\n"))
dry_mass = float(input("What is the dry mass of your rocket? (the total mass of the rocket minus the mass of the propellant)\n"))
exhaust_velocity = float(input("What is the exhaust velocity of your rocket?\n"))
mass_flow_rate = float(input("What is the mass flow rate of your rocket?\n"))

dt = 0.1
time = 0
time_list = []
height_list = []
velocity_list = []

my_rocket = Rocket(mass,dry_mass,exhaust_velocity,mass_flow_rate)

#Main loop that loops over each delta time interval of 0.1s, breaks out of loop when rocket comes back to the ground.
#The time and heights at each step are stored in respective lists
while True:
    my_rocket.update()
    time += dt
    time_list.append(time)
    height_list.append(my_rocket.height)
    velocity_list.append(my_rocket.velocity)
    if my_rocket.height <=0:
        break

#Height over time plot
plt.plot(time_list, height_list)
plt.xlabel("Time (s)")
plt.ylabel("Height (m)")
plt.title("Height over time plot")
plt.show()

#Velocity over time plot
plt.plot(time_list, velocity_list)
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.title("Velocity over time plot")
plt.show()
