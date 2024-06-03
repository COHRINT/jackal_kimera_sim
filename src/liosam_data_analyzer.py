"""
    This file takes the truth data and estimated data from sim. 
    It aligns the data frames and then graphs the 2.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import ralign
import tf.transformations as tft
from scipy.spatial.transform import Rotation 
import math

# TEST_NAME = "data/LIO_SAM_SIM"
TEST_NAME = "data/LIO_SAM_SIM_multi_long"

jackal_name ='tars'   # 'kipp'

# Opens saved data and loads them
with open(TEST_NAME + '_truth.npy', 'rb') as f:
    truthDict = np.load(f,  allow_pickle=True).item()

with open(TEST_NAME + '_LIO.npy', 'rb') as f:
    simDict = np.load(f,  allow_pickle=True).item()

truth = np.array(truthDict[jackal_name])
sim = np.array(simDict[jackal_name])
# Calculates the frame difference between truth and estimates
R, c, t = ralign.ralign(sim.T, truth.T)
r =  Rotation.from_matrix(R)
angles = r.as_euler("xyz")

transform = tft.compose_matrix(translate=t, angles=angles)

 

# Transforms simulation data to same frame as truth data
transformed_sim =    [tft.translation_from_matrix(
                        tft.concatenate_matrices(
                        transform, tft.compose_matrix(translate=point)))
                        for point in sim]

xk = [point[0] for point in transformed_sim]
yk = [point[1] for point in transformed_sim]
zk = [point[2] for point in transformed_sim]

# xk = [point[0] for point in sim]
# yk = [point[1] for point in sim]
# zk = [point[2] for point in sim]

xt = [point[0] for point in truth]
yt = [point[1] for point in truth]
zt = [point[2] for point in truth]

print('Truth:', truth[0,:] )
# print('sim transformed:', transformed_sim[0] )
print('sim ', sim[0,:] )
# print('Initial delta', np.abs(truth[0,:]-transformed_sim[0] ) )

delta = truth-transformed_sim

# delta = truth-sim
MSE = np.square(delta).mean() 
 
RMSE = math.sqrt(MSE)
print("Root Mean Square Error:\n")
print(RMSE)

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

fig = plt.figure()
ax1 = fig.add_subplot(111)

# Plot the data
ax1.plot(xt, yt, color='k', label='truth')
ax1.scatter(xt[0],yt[0], color='green')
ax1.scatter(xt[-1],yt[-1], color='red')

ax1.plot(xk, yk,  color='blue', label='estimate')
ax1.scatter(xk[0],yk[0], color='green')
ax1.scatter(xk[-1],yk[-1], color='red')
# ax1.view_init(elev=90, azim=0)

plt.savefig(jackal_name+"_trajectory.svg")


# Show the plot

plt.show()
