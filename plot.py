import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

fig = plt.figure()

data = np.loadtxt("data.txt", delimiter=" ").astype(float)
n = len(data)
data[:, 2] += 1000.0
data[:, :3] = data[:, :3] / 1000.0

roll = 0.0
rolls = []

pitch = 0.0
pitches = []

alpha = 0.95

for i in range(n):
    accel_roll = np.arctan2(data[i, 1], data[i, 2]) * 180 / np.pi
    gyro_roll = roll + data[i, 3]
    roll = alpha * gyro_roll + (1 - alpha) * accel_roll
    rolls.append(roll)

    accel_pitch = np.arctan2(data[i, 0], data[i, 2]) * 180 / np.pi
    gyro_pitch = pitch + data[i, 4]
    pitch = alpha * gyro_pitch + (1 - alpha) * accel_pitch
    pitches.append(pitch)


accel_rolls = np.arctan2(data[:, 1], data[:, 2]) * 180 / np.pi
accel_pitches = np.arctan2(data[:, 0], data[:, 2]) * 180 / np.pi


gyro_rolls = np.cumsum(data[:, 3])
gyro_pitches = np.cumsum(data[:, 4])


# def animate(i):
#     plt.cla()
#     plt.plot(accel_rolls[:i], accel_pitches[:i], label="accelerometer", c="b")
#     plt.plot(gyro_rolls[:i], gyro_pitches[:i], label="gyroscope", c="r")
#     plt.plot(rolls[:i], pitches[:i], label="filter", c="g")


# anim = FuncAnimation(plt.gcf(), animate, interval=1)

ax0 = fig.add_subplot(121)
ax1 = fig.add_subplot(122)

ax0.plot(accel_rolls, label="accelerometer")
ax0.plot(gyro_rolls, label="gyroscope")
ax0.plot(rolls, label="filter")
ax0.legend()


ax1.plot(accel_pitches, label="accelerometer")
ax1.plot(gyro_pitches, label="gyroscope")
ax1.plot(pitches, label="filter")
ax1.legend()

plt.show()
