import csv
import matplotlib.pyplot as plt

time_list = []
x_list = []
y_list = []
vx_list = []
vy_list = []
speed_list = []

with open("telemetry.csv", "r") as csvfile:
    csv_reader = csv.reader(csvfile)
    next(csv_reader)
    for row in csv_reader:
        time_list.append(float(row[0]))
        x_list.append(float(row[1]))
        y_list.append(float(row[2]))
        vx_list.append(float(row[3]))
        vy_list.append(float(row[4]))
        speed_list.append(float(row[5]))

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
