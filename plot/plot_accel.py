import numpy as np
import matplotlib.pyplot as plt

fig, ax = plt.subplots(1, 2)
x = np.loadtxt("accel.txt", delimiter=",")
ax[0].plot(x[:, 0])
ax[0].plot(x[:, 1])
ax[0].plot(x[:, 2])
ax[1].plot(x[:, 3])
ax[1].plot(x[:, 4])
ax[1].plot(x[:, 5])

plt.show()
