from mpl_toolkits.mplot3d import Axes3D     # Implicitely needed for 3D projections
import matplotlib.pyplot as plt
from vrep.const import *
from youbot import YouBot
from vrep import VRep
import numpy as np

# Illustrates the V-REP bindings, more specifically the way to take a 3D point cloud.

# (C) Copyright Renaud Detry 2013, Thibaut Cuvelier 2017.
# Distributed under the GNU General Public License.
# (See http://www.gnu.org/copyleft/gpl.html)

## Initiate the connection to the simulator. 
print('Program started')
VRep.simxFinish(-1)
vrep = VRep('127.0.0.1', 19998, True, True, 2000, 5)
print('Connection %d to the remote API server open.\n' % vrep.clientID)

vrep.simxStartSimulation(simx_opmode_oneshot_wait)

# Retrieve all handles, mostly the Hokuyo for this example.
youbot = YouBot(vrep)
youbot.hokuyo_init(vrep)

## Read data from the depth camera (Hokuyo)
# Reading a 3D image costs a lot to VREP (it has to simulate the image). It also requires a lot of
# bandwidth, and processing a 3D point cloud (for instance, to find one of the boxes or cylinders 
# that the robot has to grasp) will take a long time in MATLAB. In general, you will only want to
# capture a 3D image at specific times, for instance when you believe you're facing one of the 
# tables.

# Reduce the view angle to pi/8 in order to better see the objects. Do it only once.
# ^^^^^^     ^^^^^^^^^^    ^^^^                                     ^^^^^^^^^^^^^^^
# simxSetFloatSignal                                                simx_opmode_oneshot_wait
#            |
#            rgbd_sensor_scan_angle
# The depth camera has a limited number of rays that gather information. If this number is 
# concentrated on a smaller angle, the resolution is better. pi/8 has been determined by 
# experimentation.
vrep.simxSetFloatSignal('rgbd_sensor_scan_angle', np.pi / 8, simx_opmode_oneshot_wait)

# Ask the sensor to turn itself on, take A SINGLE POINT CLOUD, and turn itself off again.
# ^^^     ^^^^^^                ^^       ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
# simxSetIntegerSignal          1        simx_opmode_oneshot_wait
#         |
#         handle_xyz_sensor
vrep.simxSetIntegerSignal('handle_xyz_sensor', 1, simx_opmode_oneshot_wait)

# Then retrieve the last point cloud the depth sensor took.
# If you were to try to capture multiple images in a row, try other values than
# vrep.simx_opmode_oneshot_wait.
print('Capturing point cloud...')
pts = youbot.xyz_read(vrep, simx_opmode_oneshot_wait)
# Each column of pts has [xyzdistancetosensor]. However, plot3 does not have the same frame of 
# reference as the output data. To get a correct plot, you should invert the y and z dimensions.

# Plot all the points.
ax = plt.subplot(projection='3d') # type: plt.Axes
ax.scatter(pts[0, :], pts[2, :], pts[1, :], '*')
plt.show()

# Plot the points of the wall (further away than 1.87 m, which is determined either in the simulator
# by measuring distances or by trial and error) in a different colour. This value is only valid 
# for this robot position, of course. This simple test ignores the variation of distance along 
# the wall (distance between a point and several points on a line).
ax = plt.subplot(projection='3d') # type: plt.Axes
pts_wall = pts[:3, pts[3, :] >= 1.87]
ax.scatter(pts_wall[0, :], pts_wall[2, :], pts_wall[1, :], '.r')
plt.show()

# Save the point cloud to pc.xyz. (This file can be displayed with http://www.meshlab.net/.)
with open('pc.xyz', 'w') as f:
    for pt in pts.T:
        f.write("%f %f %f\n" % tuple(pt[:3]))
print('Read %d 3D points, saved to pc.xyz.' % pts.shape[1])

