import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Load CSV from unittest output
data = np.loadtxt('../../build/line_full.csv', delimiter=',', skiprows=1)
start = data[0]
end   = data[-1]

fig = plt.figure(figsize=(9,7))
ax = fig.add_subplot(111, projection='3d')

# Plot trajectory line
ax.plot(data[:,0], data[:,1], data[:,2], color='blue', linewidth=2, label='Trajectory')

# Plot start and goal points as spheres
ax.scatter(*start, color='red', s=200, label='Start')
ax.scatter(*end, color='green', s=200, label='Goal')

# Labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Line Trajectory with Vel & Acc')
ax.legend()

# Equal axis scaling
ax.set_box_aspect([np.ptp(data[:,0]), np.ptp(data[:,1]), np.ptp(data[:,2])])

plt.show()
