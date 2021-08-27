import numpy as np
import matplotlib.pyplot as plt


x = np.loadtxt("data.txt", delimiter=",")
plt.scatter(x[:, 3], x[:, 4])
plt.scatter(x[:, 4], x[:, 5])
plt.scatter(x[:, 3], x[:, 5])

plt.show()
