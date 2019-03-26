import matplotlib.pyplot as plt
from vrep.const import *
from youbot import YouBot
from vrep import VRep
import numpy as np
from robopy.base import transl, trotx, troty, trotz
from youbot.transforms import homtrans

# (C) Copyright Renaud Detry 2013, Thibaut Cuvelier 2017.
# Distributed under the GNU General Public License.
# (See http://www.gnu.org/copyleft/gpl.html)

## Initiate the connection to the simulator. 
print('Program started')
VRep.simxFinish(-1)
vrep = VRep('127.0.0.1', 19998, True, True, 2000, 5)
print('Connection %d to the remote API server open.\n' % vrep.clientID)

vrep.simxStopSimulation(simx_opmode_blocking)
vrep.simxStartSimulation(simx_opmode_oneshot_wait)

# Retrieve all handles, mostly the Hokuyo for this example.
youbot = YouBot(vrep)
youbot.streaming_init(vrep)
youbot.hokuyo_init(vrep)

## Read data from the depth camera (Hokuyo)
# Get the position and orientation of the youBot in the world reference frame (as if with a GPS).
youbot_pos = vrep.simxGetObjectPosition(youbot.ref, -1, simx_opmode_buffer)
youbot_euler = vrep.simxGetObjectOrientation(youbot.ref, -1, simx_opmode_buffer)

# Determine the position of the Hokuyo with global coordinates (world reference frame).
trf = np.asarray(transl(youbot_pos) * trotx(youbot_euler[0]) * troty(youbot_euler[1]) *\
                 trotz(youbot_euler[2]))
world_hokuyo1 = homtrans(trf, np.asarray(youbot.hokuyo1_pos)[:, None])
world_hokuyo2 = homtrans(trf, np.asarray(youbot.hokuyo2_pos)[:, None])

# Use the sensor to detect the visible points, within the world frame.
pts, contacts = youbot.hokuyo_read(vrep, simx_opmode_buffer, trf)

# Plot this data: delimit the visible area and highlight contact points.
ax = plt.subplot()  # type: plt.Axes
ax.plot(pts[0, contacts], pts[1, contacts], '*r')
ax.plot([world_hokuyo1[0], *pts[0, :], world_hokuyo2[0]],
        [world_hokuyo1[1], *pts[1, :], world_hokuyo2[1]], 'r')
ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)
ax.set_aspect('equal')
plt.show()
