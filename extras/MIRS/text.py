# # Import libraries
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


# # Create axis
axes = [1, 5, 5]
data = np.ones(axes, dtype=bool)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.voxels(data, facecolors='green')

plt.show()
