import matplotlib.pyplot as plt
#Initial values for the height at which the ball drops from, acceleration due to gravity, initial velocity, time passed
#and the change in time between iterations/recorded measurements
height = 100
gravity = -9.8
velocity = 0
time = 0
dt = 0.1

#Lists to track the time and heights as the ball falls
time_list = []
height_list = []

#Loop to iterate each interval of the ball dropping
#Calculates the velocity, height, and time passed at each interval then writes the time passed and current height to a
#list
while height > 0:
    velocity = velocity + gravity * dt
    height = height + velocity * dt
    # This stops the height from being negative
    if height < 0:
        height = 0
    time = time + dt
    time_list.append(time)
    height_list.append(height)

plt.plot(time_list, height_list)
plt.xlabel("Time (s)")
plt.ylabel("Height(m)")
plt.title("Ball Drop Under Gravity")
plt.show()
