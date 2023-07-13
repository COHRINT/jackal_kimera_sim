"""
    This file takes the truth data and estimated data from Kimera. 
    It aligns the data frames and then graphs the 2.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import ralign
import tf.transformations as tft
from scipy.spatial.transform import Rotation 

TEST_NAME = "data/BASE_TEST"

# Opens saved data and loads them
with open(TEST_NAME + '_truth.npy', 'rb') as f:
    truth = np.load(f)

with open(TEST_NAME + '_kimera.npy', 'rb') as f:
    kimera = np.load(f)

# Calculates the frame difference between truth and estimates
R, c, t = ralign.ralign(kimera.T, truth.T)
r =  Rotation.from_dcm(R)
angles = r.as_euler("xyz")

transform = tft.compose_matrix(translate=t, angles=angles)

 

# Transforms kimera data to same frame as truth data
transformed_kimera =    [tft.translation_from_matrix(
                        tft.concatenate_matrices(
                        transform, tft.compose_matrix(translate=point)))
                        for point in kimera]

xk = [point[0] for point in transformed_kimera]
yk = [point[1] for point in transformed_kimera]
zk = [point[2] for point in transformed_kimera]

xt = [point[0] for point in truth]
yt = [point[1] for point in truth]
zt = [point[2] for point in truth]


# Graphs them with matplotlib
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the data
ax.plot(xt, yt, zt, color='k', label='truth')
ax.scatter(xt[0],yt[0],zt[0], color='green')
ax.scatter(xt[-1],yt[-1],zt[-1], color='red')

ax.plot(xk, yk, zk, color='blue', label='estimate')
ax.scatter(xk[0],yk[0],zk[0], color='green')
ax.scatter(xk[-1],yk[-1],zk[-1], color='red')

# Set labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
ax.set_title('Truth vs Estimate')

# Show the plot

plt.show()
