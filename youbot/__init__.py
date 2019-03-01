from vrep import VRep
from vrep.const import *
from robopy.base.transforms import transl, trotz, troty, trotx
import numpy as np

# Initialize youBot

# (C) Copyright Renaud Detry 2013.
# Distributed under the GNU General Public License.
# (See http://www.gnu.org/copyleft/gpl.html)

class YouBot:
    # Contains all handles, stream arm and wheel joints, the robot's pose, the Hokuyo, 
    # and the arm tip pose.
    
    def __init__(self, vrep: VRep):
        # Wheel handles (front left, rear left, rear right, front right).
        self.wheel_joints = [vrep.simxGetObjectHandle('rollingJoint_' + s) for s in
                             ['fl', 'rl', 'rr', 'fr']]
        for wheel_joint in self.wheel_joints:
            vrep.simxSetJointTargetVelocity(wheel_joint, 0, simx_opmode_oneshot)
        
        # Rigidbody variables (have a look at youbot_drive for these values)
        self.previous_forw_back_vel = 0
        self.previous_left_right_vel = 0
        self.previous_rot_vel = 0
        
        # Sensor handles. The Hokuyo sensor is implemented with two planar sensors that each 
        # cover 120 degrees, hence the two handles
        self.hokuyo1 = vrep.simxGetObjectHandle('fastHokuyo_sensor1')
        self.hokuyo2 = vrep.simxGetObjectHandle('fastHokuyo_sensor2')
        
        self.xyz_sensor = vrep.simxGetObjectHandle('xyzSensor')
        self.rgb_sensor = vrep.simxGetObjectHandle('rgbSensor')
        self.rgbd_casing = vrep.simxGetObjectHandle('rgbdSensor')
        
        # Robot handles
        self.ref = vrep.simxGetObjectHandle('youBot_center')
        self.arm_ref = vrep.simxGetObjectHandle('youBot_ref')
        
        # Arm handles. 
        # The project page ( http://renaud-detry.net/teaching/info0948/private/project.php )
        # contains information on the different control modes of the arm. Search for
        # km_mode on the project webpage to find the arm documentation. Read that documentation
        # before working with the code below.
        
        # The *position* of this object always corresponds to the position of the tip of
        # the arm (the tip is somewhere between the two fingers)
        self.ptip = vrep.simxGetObjectHandle('youBot_gripperPositionTip')
        
        # In IK mode (km_mode set to 1 or 2), the robot will try to move the *position*
        # of ptip to the *position* of ptarget.
        self.ptarget = vrep.simxGetObjectHandle('youBot_gripperPositionTarget')
        
        # The *orientation* of this object always corresponds to the orientation of the tip of
        # the arm (the tip is somewhere between the two fingers)
        self.otip = vrep.simxGetObjectHandle('youBot_gripperOrientationTip')
        
        # In IK mode 2 (km_mode set to 2), the robot will try to move the *orientation*
        # of otip to the *orientation* of otarget.
        self.otarget = vrep.simxGetObjectHandle('youBot_gripperOrientationTarget')
        
        # Tip orientations are easier to manipulate in the reference frame of Rectangle22,
        # because then the degree of freedom onto which the orientation controller acts
        # corresponds to one of the three Euler angles of the tip orientation.
        self.r22 = vrep.simxGetObjectHandle('Rectangle22')
        
        self.arm_joints = [vrep.simxGetObjectHandle('youBotArmJoint%d' % i) for i in range(1, 5)]
        self.map_looker = vrep.simxGetObjectHandle('map')
        self.landmarks = vrep.simxGetObjectHandle('Landmarks')
    
    def hokuyo_init(self, vrep: VRep):
        # Initialize Hokuyo sensor in VREP
        # This def starts the Hokuyo sensor, and it computes the transformations
        # between the Hokuyo frame and the youBot reference frame h.ref.
        # These transformations are stored in h.hokuyo1Trans and h.hokuyo2Trans
        
        # (C) Copyright Renaud Detry 2013.
        # Distributed under the GNU General Public License.
        # (See http://www.gnu.org/copyleft/gpl.html)
        
        # Turn the Hokuyo on (constantly)
        vrep.simxSetIntegerSignal('handle_xy_sensor', 2, simx_opmode_oneshot)
        # Display the red laser beams of active sensors
        vrep.simxSetIntegerSignal('displaylasers', 1, simx_opmode_oneshot)
        
        # In order to build a map, you will need to transfer the data coming from the
        # Hokuyo to the world reference frame. The transformation between the frame of
        # hokuyoXHandle and the world is not accessible. However, you can access
        # the transformation between h.hokuyoX and h.ref (right below), and you
        # can access the transformation between h.ref and the world (see below, search for
        # "Stream wheel angles and robot pose"). By combining these two transformations,
        # you can transform the Hokuyo data to the world frame and build your map.
        # Do not forget that you can use the functions from the robot toolbox to compute
        # transformations, see page 23 of the book, e.g. the functions se2(), inv(),
        # h2e(), e2h(), homtrans(), ...
        self.hokuyo1_pos = vrep.simxGetObjectPosition(self.hokuyo1, self.ref,
                                                      simx_opmode_oneshot_wait)
        self.hokuyo1_euler = vrep.simxGetObjectOrientation(self.hokuyo1, self.ref,
                                                           simx_opmode_oneshot_wait)
        self.hokuyo2_pos = vrep.simxGetObjectPosition(self.hokuyo2, self.ref,
                                                      simx_opmode_oneshot_wait)
        self.hokuyo2_euler = vrep.simxGetObjectOrientation(self.hokuyo2, self.ref,
                                                           simx_opmode_oneshot_wait)
        
        # fixme
        # for j = [0:.1:(2*pi) 2*pi]
        #  vrep.simxSetObjectOrientation(h.id, h.rgbdCasing, h.ref, [0 0 -pi/2+j], 
        #  vrep.simx_opmode_oneshot)
        #  pause(1/20)
        # end
        
        # Compute the transformations between the two Hokuyo subsensors and the youBot
        # ref frame h.ref
        self.hokuyo1_trans = transl(self.hokuyo1_pos) * trotx(self.hokuyo1_euler[0]) * troty(
            self.hokuyo1_euler[1]) * trotz(self.hokuyo1_euler[2])
        self.hokuyo2_trans = transl(self.hokuyo2_pos) * trotx(self.hokuyo2_euler[0]) * troty(
            self.hokuyo2_euler[1]) * trotz(self.hokuyo2_euler[2])

    def hokuyo_read(self, vrep, opmode, trans=None):
        # Reads from Hokuyo sensor.
       
        t1 = self.hokuyo1_trans
        t2 = self.hokuyo2_trans
        if trans is not None:
            t1 = trans * self.hokuyo1_trans
            t2 = trans * self.hokuyo2_trans
    
        # The Hokuyo data comes in a funny format. Use the code below to move it
        # to a Matlab matrix
        det, aux_data = vrep.simxReadVisionSensor(self.hokuyo1, opmode)
        pts1 = np.reshape(aux_data[1][2:], (-1, 4))
        
        # Each column of pts1 has [xyzdistancetosensor]
        # The Hokuyo sensor has a range of 5m. If there are no obstacles, a point
        # is returned at the 5m limit. As we do not want these points, we throw
        # away all points that are 5m far from the sensor.
        obst1 = pts1[:, 4] < 4.9999
        pts1 = pts1[:, :3]

        # Process the other 120 degrees      
        det, aux_data = vrep.simxReadVisionSensor(self.hokuyo2, opmode)
        pts2 = np.reshape(aux_data[1][2:], (-1, 4))
        obst2 = pts2[:, 4] < 4.9999
        pts2 = pts2[:, :3]
        
        raise NotImplemented()  # TODO
        # 
        # scanned_points = [homtrans(t1, pts1) homtrans(t2, pts2)]
        # contacts = [obst1 obst2]
        # 
        # return scanned_points, contacts
    
    def examples(self, vrep):
        ## Examples: getting information from the simulator (and testing the connection). Stream 
        # wheel angles, Hokuyo data, and robot pose (see usage below). Wheel angles are not used 
        # in this example, but they may be necessary in your project.
        
        for wheel_joint in self.wheel_joints:
            vrep.simxGetJointPosition(wheel_joint)
        vrep.simxGetObjectPosition(self.ref, -1)
        vrep.simxGetObjectOrientation(self.ref, -1, simx_opmode_streaming)
        vrep.simxReadVisionSensor(self.hokuyo1, simx_opmode_streaming)
        vrep.simxReadVisionSensor(self.hokuyo2, simx_opmode_streaming)
        
        # Stream the arm joint angles and the tip position/orientation
        vrep.simxGetObjectPosition(self.ptip, self.arm_ref, simx_opmode_streaming)
        vrep.simxGetObjectOrientation(self.otip, self.r22, simx_opmode_streaming)
        for arm_joint in self.arm_joints:
            vrep.simxGetJointPosition(arm_joint, simx_opmode_streaming)
        
        # Make sure that all streaming data has reached the client at least once
        vrep.simxGetPingTime()


