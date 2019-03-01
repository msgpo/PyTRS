from vrep import VRep
from vrep.const import *
from vrep.vrchk import vrchk
from youbot import YouBot
from time import sleep
import numpy as np
import cv2
import math
import atexit
import matplotlib.pyplot as plt

# Illustrates the V-REP bindings.

# (C) Copyright Renaud Detry 2013, Thibaut Cuvelier 2017, Mathieu Baijot 2017.
# Distributed under the GNU General Public License.
# (See http://www.gnu.org/copyleft/gpl.html)


# Open house.ttt in vrep
# Run simRemoteApi.start(19998) in the vrep LUA console

if __name__ == '__main__':
    ## Initiate the connection to the simulator. 
    print('Program started')
    VRep.simxFinish(-1)
    vrep = VRep('127.0.0.1', 19998, True, True, 2000, 5)
    print('Connection %d to the remote API server open.\n' % vrep.clientID)

    # This will only work in "continuous remote API server service". 
    # See http://www.v-rep.eu/helpFiles/en/remoteApiServerSide.htm
    vrep.simxStartSimulation(simx_opmode_oneshot_wait)

    def stop_simulation():
        vrep.simxStopSimulation(simx_opmode_oneshot_wait)
        vrep.simxFinish(vrep.clientID)

    # Stop the simulation when exciting with for example ctrl-C
    atexit.register(stop_simulation)

    # Retrieve all handles, and stream arm and wheel joints, the robot's pose, the Hokuyo, and the 
    # arm tip pose. The tip corresponds to the point between the two tongs of the gripper (for 
    # more details, see later or in the file focused/youbot_arm.m). 
    youbot = YouBot(vrep)
    youbot.examples(vrep)
    youbot.hokuyo_init(vrep)
    
    # Let a few cycles pass to make sure there's a value waiting for us next time we try to get a 
    # joint angle or the robot pose with the simx_opmode_buffer option.
    sleep(.2)

    ## Youbot constants
    # The time step the simulator is using (your code should run close to it). 
    timestep = .05

    # Minimum and maximum angles for all joints. Only useful to implement custom IK. 
    arm_joint_ranges = [-2.9496064186096, 2.9496064186096
                        -1.5707963705063, 1.308996796608
                        -2.2863812446594, 2.2863812446594
                        -1.7802357673645, 1.7802357673645
                        -1.5707963705063, 1.5707963705063]

    # Definition of the starting pose of the arm (the angle to impose at each joint to be in the 
    # rest position).
    starting_joints = np.deg2rad(np.array([0, 30.91, 52.42, 72.68, 0]))

    ## Preset values for the demo. 
    print('Starting robot')
    
    # Define the preset pickup pose for this demo. 
    pickup_joints = np.deg2rad(np.array([90, 19.6, 113, -41, 0]))

    # Parameters for controlling the youBot's wheels: at each iteration, those values will be set 
    # for the wheels.
    forw_back_vel = 0       # Move straight ahead. 
    right_vel = 0           # Go sideways. 
    rotate_right_vel = 0    # Rotate. 
    prev_orientation = 0    # Previous angle to goal (easy way to have a condition on the robot's 
                            # angular speed). 
    prev_position = 0       # Previous distance to goal (easy way to have a condition on the 
                            # robot's speed). 

    # Set the arm to its starting configuration. 
    vrep.simxPauseCommunication(True) 
    for arm_joint, starting_joint in zip(youbot.arm_joints, starting_joints):
        vrep.simxSetJointTargetPosition(arm_joint, starting_joint)
    vrep.simxPauseCommunication(False) 

    # Initialise the plot. 
    plot_data = True
    if plot_data:
        # Prepare the plot area to receive three plots: what the Hokuyo sees at the top (2D map), 
        # the point cloud and the image of what is in front of the robot at the bottom. 
        plt.subplot(211)

        # Create a 2D mesh of points, stored in the vectors X and Y. This will be used to display 
        # the area the robot can see, by selecting the points within this mesh that are within 
        # the visibility range.
        x, y = np.meshgrid(np.arange(-5, 5, .25), np.arange(-5, 5, .25))
        x, y = x.reshape(-1), y.reshape(-1)

    # Make sure everything is settled before we start. 
    sleep(2)

    # Retrieve the position of the gripper. 
    home_gripper_position = vrep.simxGetObjectPosition(youbot.ptip, youbot.arm_ref, 
                                                       simx_opmode_buffer)

    # Initialise the state machine. 
    #fsm = 'rotate'
    fsm = 'snapshot'

    ## Start the demo. 
    while True:
        if vrep.simxGetConnectionId() == -1:
            raise Exception('Lost connection to remote API.')

        # Get the position and the orientation of the robot. 
        youbot_pos = vrep.simxGetObjectPosition(youbot.ref, -1, simx_opmode_buffer)
        youbot_euler = vrep.simxGetObjectOrientation(youbot.ref, -1, simx_opmode_buffer)

        # ## Plot something if required. 
        # if plot_data:
        #     # Read data from the depth sensor, more often called the Hokuyo (if you want to be more
        #     # precise about the way you control the sensor, see later for the details about this 
        #     # line or the file focused/youbot_3dpointcloud.m).
        #     # This def returns the set of points the Hokuyo saw in pts. contacts indicates, 
        #     # for each point, if it corresponds to an obstacle (the ray the Hokuyo sent was 
        #     # interrupted by an obstacle, and was not allowed to go to infinity without being 
        #     # stopped). 
        #     pts, contacts = youbot.hokuyo_read(vrep, simx_opmode_buffer)
        # 
        #     # Select the points in the mesh [X, Y] that are visible, as returned by the Hokuyo (it returns the area that
        #     # is visible, but the visualisation draws a series of points that are within this visible area). 
        #     in = inpolygon(X, Y, ...
        #                    [handles.hokuyo1Pos(1), pts(1,:), handles.hokuyo2Pos(1)], ...
        #                    [handles.hokuyo1Pos(2), pts(2,:), handles.hokuyo2Pos(2)])
        # 
        #     # Plot those points. Green dots: the visible area for the Hokuyo. Red starts: the obstacles. Red lines: the
        #     # visibility range from the Hokuyo sensor. 
        #     # The youBot is indicated with two dots: the blue one corresponds to the rear, the red one to the Hokuyo
        #     # sensor position. 
        #     subplot(211)
        #     plot(X(in), Y(in), '.g',...
        #          pts(1, contacts), pts(2, contacts), '*r', ...
        #          [handles.hokuyo1Pos(1), pts(1,:), handles.hokuyo2Pos(1)], [handles.hokuyo1Pos(2), pts(2, :), handles.hokuyo2Pos(2)], 'r', ...
        #          0, 0, 'ob',...
        #     handles.hokuyo1Pos(1), handles.hokuyo1Pos(2), 'or', ...
        #     handles.hokuyo2Pos(1), handles.hokuyo2Pos(2), 'or')
        #     axis([-5.5, 5.5, -5.5, 2.5])
        #     axis equal
        #     drawnow
        # angl = -pi/2

    #     ## Apply the state machine. 
        if fsm == 'rotate':
            pass
    #         ## First, rotate the robot to go to one table.             
    #         # The rotation velocity depends on the difference between the current angle and the target. 
    #         rotateRightVel = angdiff(angl, youbotEuler(3))
    # 
    #         # When the rotation is done (with a sufficiently high precision), move on to the next state. 
    #         if (abs(angdiff(angl, youbotEuler(3))) < .1 / 180 * pi) && ...:
    #                 (abs(angdiff(prevOrientation, youbotEuler(3))) < .01 / 180 * pi)
    #             rotateRightVel = 0
    #             fsm = 'drive'
    # 
    #         prevOrientation = youbotEuler(3)
    #     elif strcmp(fsm, 'drive'):
    #         ## Then, make it move straight ahead until it reaches the table (x = 3.167 m). 
    #         # The further the robot, the faster it drives. (Only check for the first dimension.)
    #         # For the project, you should not use a predefined value, but rather compute it from your map. 
    #         forwBackVel = - (youbotPos(1) + 3.167)
    # 
    #         # If the robot is sufficiently close and its speed is sufficiently low, stop it and move its arm to 
    #         # a specific location before moving on to the next state.
    #         if (youbotPos(1) + 3.167 < .001) && (abs(youbotPos(1) - prevPosition) < .001):
    #             forwBackVel = 0
    # 
    #             # Change the orientation of the camera to focus on the table (preparation for next state). 
    #             vrep.simxSetObjectOrientation(link_id, handles.rgbdCasing, handles.ref, [0, 0, pi / 4], vrep.simx_opmode_oneshot)
    # 
    #             # Move the arm to the preset pose pickupJoints (only useful for this demo you should compute it based
    #             # on the object to grasp). 
    #             for i in range(5):
    #                 res = vrep.simxSetJointTargetPosition(link_id, handles.armJoints(i), pickupJoints(i), ...
    #                                                       vrep.simx_opmode_oneshot)
    #                 vrchk(vrep, res, True)
    # 
    #             fsm = 'snapshot'
    #         prevPosition = youbotPos(1)
        elif fsm == 'snapshot':
    #         ## Read data from the depth camera (Hokuyo)
    #         # Reading a 3D image costs a lot to VREP (it has to simulate the image). It also requires a lot of 
    #         # bandwidth, and processing a 3D point cloud (for instance, to find one of the boxes or cylinders that 
    #         # the robot has to grasp) will take a long time in MATLAB. In general, you will only want to capture a 3D 
    #         # image at specific times, for instance when you believe you're facing one of the tables.
    # 
    #         # Reduce the view angle to pi/8 in order to better see the objects. Do it only once. 
    #         # ^^^^^^     ^^^^^^^^^^    ^^^^                                     ^^^^^^^^^^^^^^^ 
    #         # simxSetFloatSignal                                                simx_opmode_oneshot_wait
    #         #            |
    #         #            rgbd_sensor_scan_angle
    #         # The depth camera has a limited number of rays that gather information. If this number is concentrated 
    #         # on a smaller angle, the resolution is better. pi/8 has been determined by experimentation. 
    #         res = vrep.simxSetFloatSignal(id, 'rgbd_sensor_scan_angle', pi / 8, vrep.simx_opmode_oneshot_wait)
    #         vrchk(vrep, res)
    # 
    #         # Ask the sensor to turn itself on, take A SINGLE POINT CLOUD, and turn itself off again. 
    #         # ^^^     ^^^^^^                ^^       ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    #         # simxSetIntegerSignal          1        simx_opmode_oneshot_wait
    #         #         |
    #         #         handle_xyz_sensor
    #         res = vrep.simxSetIntegerSignal(id, 'handle_xyz_sensor', 1, vrep.simx_opmode_oneshot_wait)
    #         vrchk(vrep, res)
    # 
    #         # Then retrieve the last point cloud the depth sensor took.
    #         # If you were to try to capture multiple images in a row, try other values than 
    #         # vrep.simx_opmode_oneshot_wait. 
    #         fprintf('Capturing a point cloud...\n')
    #         pts = youbot_xyz_sensor(vrep, handles, vrep.simx_opmode_oneshot_wait)
    #         # Each column of pts has [xyzdistancetosensor]. However, plot3 does not have the same frame of reference as 
    #         # the output data. To get a correct plot, you should invert the y and z dimensions. 
    # 
    #         # Here, we only keep points within 1 meter, to focus on the table. 
    #         pts = pts(1:3, pts(4, :) < 1)
    # 
    #         if plotData:
    #             subplot(223)
    #             plot3(pts(1, :), pts(3, :), pts(2, :), '*')
    #             axis equal
    #             view([-169 -46])
    # 
    #         # Save the point cloud to pc.xyz. (This file can be displayed with http://www.meshlab.net/.)
    #         fileID = fopen('pc.xyz','w')
    #         fprintf(fileID,'#f #f #f\n', pts)
    #         fclose(fileID)
    #         fprintf('Read #i 3D points, saved to pc.xyz.\n', max(size(pts)))
    # 
    #         ## Read data from the RGB camera
    #         # This starts the robot's camera to take a 2D picture of what the robot can see. 
    #         # Reading an image costs a lot to VREP (it has to simulate the image). It also requires a lot of bandwidth, 
    #         # and processing an image will take a long time in MATLAB. In general, you will only want to capture 
    #         # an image at specific times, for instance when you believe you're facing one of the tables or a basket.
    # 
            # Ask the sensor to turn itself on, take A SINGLE IMAGE, and turn itself off again. 
            # ^^^     ^^^^^^                ^^       ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
            # simxSetIntegerSignal          1        simx_opmode_oneshot_wait
            #         |
            #         handle_rgb_sensor
            res = vrep.simxSetIntegerSignal('handle_rgb_sensor', 1, simx_opmode_oneshot_wait)

            #Then retrieve the last picture the camera took. The image must be in RGB 
            # (not grayscale). 
            #     ^^^^^^^^^^^^^^^^^^^^^^^^^     ^^^^^^                            ^^^
            #     simxGetVisionSensorImage2     h.rgbSensor                       0
            # If you were to try to capture multiple images in a row, try other values than 
            # simx_opmode_oneshot_wait. 
            print('Capturing image...\n')
            resolution, image = vrep.simxGetVisionSensorImage(youbot.rgb_sensor, 0, 
                                                              simx_opmode_oneshot_wait)
    
            # fprintf('Captured #i pixels (#i x #i).\n', resolution(1) * resolution(2), resolution(1), resolution(2))

            # Finally, show the image. 
            img = np.array(image, dtype=np.uint8)
            img.resize([resolution[0], resolution[1], 3])
            img = img[::-1,:,:]

            cv2.imshow("RGB Image", img)
            cv2.waitKey(1)

    #         if plotData:
    #             subplot(224)
    #             imshow(image)
    #             drawnow
    # 
    #         # Next state. 
    #         fsm = 'extend'
    #     elif strcmp(fsm, 'extend'):
    #         ## Move the arm to face the object.
    #         # Get the arm position. 
    #         [res, tpos] = vrep.simxGetObjectPosition(id, handles.ptip, handles.armRef, vrep.simx_opmode_buffer)
    #         vrchk(vrep, res, True)
    # 
    #         # If the arm has reached the wanted position, move on to the next state. 
    #         # Once again, your code should compute this based on the object to grasp. 
    #         if norm(tpos - [0.3259, -0.0010, 0.2951]) < .002:
    #             # Set the inverse kinematics (IK) mode to position AND orientation. 
    #             res = vrep.simxSetIntegerSignal(id, 'km_mode', 2, vrep.simx_opmode_oneshot_wait)
    #             vrchk(vrep, res, True)
    #             fsm = 'reachout'
    #     elif strcmp(fsm, 'reachout'):
    #         ## Move the gripper tip along a line so that it faces the object with the right angle.
    #         # Get the arm tip position. The arm is driven only by the position of the tip, not by the angles of 
    #         # the joints, except if IK is disabled.
    #         # Following the line ensures the arm attacks the object with the right angle. 
    #         [res, tpos] = vrep.simxGetObjectPosition(id, handles.ptip, handles.armRef, vrep.simx_opmode_buffer)
    #         vrchk(vrep, res, True)
    # 
    #         # If the tip is at the right position, go on to the next state. Again, this value should be computed based
    #         # on the object to grasp and on the robot's position. 
    #         if tpos(1) > .39:
    #             fsm = 'grasp'
    # 
    #         # Move the tip to the next position along the line. 
    #         tpos(1) = tpos(1) + .01
    #         res = vrep.simxSetObjectPosition(id, handles.ptarget, handles.armRef, tpos, vrep.simx_opmode_oneshot)
    #         vrchk(vrep, res, True)
    #     elif strcmp(fsm, 'grasp'):
    #         ## Grasp the object by closing the gripper on it.
    #         # Close the gripper. Please pay attention that it is not possible to adjust the force to apply:  
    #         # the object will sometimes slip from the gripper!
    #         res = vrep.simxSetIntegerSignal(id, 'gripper_open', 0, vrep.simx_opmode_oneshot_wait)
    #         vrchk(vrep, res)
    # 
    #         # Make MATLAB wait for the gripper to be closed. This value was determined by experiments. 
    #         pause(2)
    # 
    #         # Disable IK this is used at the next state to move the joints manually. 
    #         res = vrep.simxSetIntegerSignal(id, 'km_mode', 0, vrep.simx_opmode_oneshot_wait)
    #         vrchk(vrep, res)
    #         fsm = 'backoff'
    #     elif strcmp(fsm, 'backoff'):
    #         ## Go back to rest position.
    #         # Set each joint to their original angle, as given by startingJoints. Please note that this operation is not
    #         # instantaneous, it takes a few iterations of the code for the arm to reach the requested pose. 
    #         for i in range(5):
    #             res = vrep.simxSetJointTargetPosition(id, handles.armJoints(i), startingJoints(i), vrep.simx_opmode_oneshot)
    #             vrchk(vrep, res, True)
    # 
    #         # Get the gripper position and check whether it is at destination (the original position).
    #         [res, tpos] = vrep.simxGetObjectPosition(id, handles.ptip, handles.armRef, vrep.simx_opmode_buffer)
    #         vrchk(vrep, res, True)
    #         if norm(tpos - homeGripperPosition) < .02:
    #             # Open the gripper when the arm is above its base. 
    #             res = vrep.simxSetIntegerSignal(id, 'gripper_open', 1, vrep.simx_opmode_oneshot_wait)
    #             vrchk(vrep, res)
    # 
    #         if norm(tpos - homeGripperPosition) < .002:
    #             fsm = 'finished'
    #     elif strcmp(fsm, 'finished'):
    #         ## Demo done: exit the function. 
    #         pause(3)
    #         break
    #     else:
    #         error('Unknown state #s.', fsm)
    # 
    #     # Update wheel velocities using the global values (whatever the state is). 
    #     handles = youbot_drive(vrep, handles, forwBackVel, rightVel, rotateRightVel)
    # 
    #     # Make sure that we do not go faster than the physics simulation (each iteration must take roughly 50 ms). 
    #     elapsed = toc
    #     timeleft = timestep - elapsed
    #     if timeleft > 0:
    #         pause(min(timeleft, .01))
    # 
    # 
