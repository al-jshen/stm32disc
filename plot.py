import numpy as np
import matplotlib.pyplot as plt

fig, ax = plt.subplots(1, 3)
x = np.loadtxt("data.txt", delimiter=",")
ax[0].plot(x[:, 0])
ax[0].plot(x[:, 1])
ax[0].plot(x[:, 2])
ax[1].plot(x[:, 3])
ax[1].plot(x[:, 4])
ax[1].plot(x[:, 5])
ax[2].plot(x[:, 6])
ax[2].plot(x[:, 7])
ax[2].plot(x[:, 8])

plt.show()
