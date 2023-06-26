import numpy as np
import matplotlib.pyplot as plt

log = np.loadtxt('output/mappoint.txt')

fig = plt.figure('Map')
ax = fig.add_subplot(111, projection='3d')
ax.scatter(log[:, 0], log[:, 1], log[:, 2], c='b', marker='.')
plt.grid()
plt.title('Map')
plt.tight_layout()

plt.show()