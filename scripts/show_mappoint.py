import numpy as np
import matplotlib.pyplot as plt

log = np.loadtxt('output/mappoint.txt')
log2 = np.loadtxt('output/mappoint_woTRN.txt')

fig = plt.figure('Map')
ax = fig.add_subplot(111, projection='3d')
ax.scatter(log[:, 0], log[:, 1], log[:, 2], c='b', marker='.')
ax.scatter(log2[:, 0], log2[:, 1], log2[:, 2], c='r', marker='.')

fig = plt.figure('xyz')
ax = fig.add_subplot(111)
ax.plot(log[:, 0], log[:, 1], c='b', marker='.')
ax.plot(log2[:, 0], log2[:, 1], c='r', marker='.')

plt.grid()
plt.title('Map')
plt.tight_layout()

plt.show()