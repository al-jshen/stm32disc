import numpy as np
import matplotlib.pyplot as plt

fig = plt.figure()
x = np.loadtxt("data.txt", delimiter=" ")
plt.plot(x[:, 0])
plt.plot(x[:, 1])

print(x[:1500].mean(axis=0))
print(x[6000:].mean(axis=0))
print(x[:1500].std(axis=0))
print(x[6000:].std(axis=0))
print(x[1:, 2].mean(), x[1:, 2].std())
plt.show()
