#!/usr/bin/env python

'''
Implements the Demo 1 of ControlIt! and Dreamer disassembling a part.
'''

import sys, getopt     # for getting and parsing command line arguments
import time
import math
import threading
import rospy

from std_msgs.msg import Float64, Float64MultiArray, MultiArrayDimension, Bool, Int32

import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import TrajectoryGeneratorCubicSpline

NUM_CARTESIAN_DOFS = 3 # Cartesian goal is x, y, z
NUM_ORIENTATION_DOFS = 3 # Orientation is defined using a x, y, z vector
NUM_ROBOT_DOFS = 16

# These are the right hand Cartesian goal positions.
# Left hand goal positions simply negate the Y axis goal.
# R_GOALS = [[0.45, -0.25, 1.1],        # move right hand to the right
#            [0.438, -0.356, 1.106],
#            [0.375, -0.451, 1.102],
#            [0.291, -0.508, 1.092],
#            [0.144, -0.528, 1.049],

#            [0.13829324246419664, -0.5779865808605724, 1.1269252743334133], # move right hand up
#            [0.14043433470588146, -0.601732376320113, 1.1971891610582215],
#            [0.1434783706196868, -0.6189997526862935, 1.2998303686016914],
#            [0.14583955739422813, -0.6231028442528961, 1.3856296718569716],
#            [0.1481883320341411, -0.6013934906799016, 1.4712865537773556],
#            [0.1500215472226119, -0.568639980698996, 1.5753674402311533]]

# # These values were taken from R_GOALS above
# WAYPOINT_X_SWING = [0.45, 0.438, 0.375, 0.291, 0.144]
# WAYPOINT_Y_SWING = [-0.25, -0.356, -0.451, -0.508, -0.528]
# WAYPOINT_Z_SWING = [1.1, 1.106, 1.102, 1.092, 1.049]

# WAYPOINT_X_WAVE = [0.144, 0.138, 0.140, 0.143, 0.146, 0.148, 0.150]
# WAYPOINT_Y_WAVE = [-0.528, -0.578, -0.601, -0.619, -0.623, -0.601, -0.569]
# WAYPOINT_Z_WAVE = [1.049, 1.127, 1.197, 1.300, 1.386, 1.471, 1.575]

# Define FSM States and constants
# STATE_GO_TO_START = 0
# STATE_WAVE_UP = 1
# STATE_WAVE_DOWN = 2
# STATE_GO_TO_FINISH = 3

# INIT_TRAJ_TIME = 5  # The number of seconds to spend traversing the initial / final trajectory
# WAVE_TRAJ_TIME = 1  # The number of seconds to spend traversing the wave up / down trajectory

# NUM_WAVES = 3
# Y_AXIS_INDEX = 1

class Demo1_ProductDisassembly:
    def __init__(self):

        # Define the dimensions of the message
        dim = MultiArrayDimension()
        dim.size = NUM_CARTESIAN_DOFS
        dim.label = "rightHandCartesianGoal"
        dim.stride = 1

        # Define the goal messages
        self.rightHandCartesianGoalMsg = Float64MultiArray()
        for ii in range(0, NUM_CARTESIAN_DOFS):
            self.rightHandCartesianGoalMsg.data.append(0)
        self.rightHandCartesianGoalMsg.layout.dim.append(dim)
        self.rightHandCartesianGoalMsg.layout.data_offset = 0

        dim.label = "leftHandCartesianGoal"
        self.leftHandCartesianGoalMsg = Float64MultiArray()
        for ii in range(0, NUM_CARTESIAN_DOFS):
            self.leftHandCartesianGoalMsg.data.append(0)
        self.leftHandCartesianGoalMsg.layout.dim.append(dim)
        self.leftHandCartesianGoalMsg.layout.data_offset = 0

        dim.size = NUM_ORIENTATION_DOFS
        dim.label = "rightHandOrientationGoal"
        self.rightHandOrientationGoalMsg = Float64MultiArray()
        for ii in range(0, NUM_ORIENTATION_DOFS):
            self.rightHandOrientationGoalMsg.data.append(0)
        self.rightHandOrientationGoalMsg.layout.dim.append(dim)
        self.rightHandOrientationGoalMsg.layout.data_offset = 0

        dim.label = "leftHandOrientationGoal"
        self.leftHandOrientationGoalMsg = Float64MultiArray()
        for ii in range(0, NUM_ORIENTATION_DOFS):
            self.leftHandOrientationGoalMsg.data.append(0)
        self.leftHandOrientationGoalMsg.layout.dim.append(dim)
        self.leftHandOrientationGoalMsg.layout.data_offset = 0

        dim.size = NUM_ROBOT_DOFS
        dim.label = "postureGoal"
        self.postureGoalMsg = Float64MultiArray()
        for ii in range(0, NUM_ROBOT_DOFS):
            self.postureGoalMsg.data.append(0)
        self.postureGoalMsg.layout.dim.append(dim)
        self.postureGoalMsg.layout.data_offset = 0

        self.rightHandCmdMsg = Bool()
        self.rightHandCmdMsg.data = False  # relax hand

        self.leftGripperCmdMsg = Bool()
        self.leftGripperCmdMsg.data = False  # relax gripper

        self.tareMsg = Int32()
        self.tareMsg.data = 1

        # Initialize member variables
        self.currentPosture = None
        self.postureError = None

        self.currentRightCartesianPos = None
        self.rightCartesianPosError = None

        self.currentLeftCartesianPos = None
        self.leftCartesianPosError = None

        self.currentRightOrientation = None
        self.rightOrientationError = None

        self.currentLeftOrientation = None
        self.leftOrientationError = None

        # Create the ROS topic subscriptions
        self.postureTaskActualSubscriber = rospy.Subscriber("/dreamer_controller/JPosTask/actualPosition", Float64MultiArray, self.postureTaskActualCallback)
        self.postureTaskErrorSubscriber  = rospy.Subscriber("/dreamer_controller/JPosTask/error",          Float64MultiArray, self.postureTaskErrorCallback)

        self.rightCartesianTaskActualSubscriber = rospy.Subscriber("/dreamer_controller/RightHandPosition/actualWorldPosition", Float64MultiArray, self.rightCartesianTaskActualCallback)
        self.rightCartesianTaskErrorSubscriber  = rospy.Subscriber("/dreamer_controller/RightHandPosition/error",               Float64MultiArray, self.rightCartesianTaskErrorCallback)

        self.leftCartesianTaskActualSubscriber = rospy.Subscriber("/dreamer_controller/LeftHandPosition/actualWorldPosition",  Float64MultiArray, self.leftCartesianTaskActualCallback)
        self.leftCartesianTaskErrorSubscriber  = rospy.Subscriber("/dreamer_controller/LeftHandPosition/error",                Float64MultiArray, self.leftCartesianTaskErrorCallback)

        self.rightOrientationTaskActualSubscriber = rospy.Subscriber("/dreamer_controller/RightHandOrientation/actualHeading", Float64MultiArray, self.rightOrientationTaskActualCallback)
        self.rightOrientationTaskErrorSubscriber  = rospy.Subscriber("/dreamer_controller/RightHandOrientation/errorAngle",    Float64,           self.rightOrientationTaskErrorCallback)

        self.leftOrientationTaskActualSubscriber = rospy.Subscriber("/dreamer_controller/LeftHandOrientation/actualHeading", Float64MultiArray, self.leftOrientationTaskActualCallback)
        self.leftOrientationTaskErrorSubscriber  = rospy.Subscriber("/dreamer_controller/LeftHandOrientation/errorAngle",    Float64,           self.leftOrientationTaskErrorCallback)

        # Create the ROS topic publishers
        self.postureTaskGoalPublisher = rospy.Publisher("/dreamer_controller/JPosTask/goalPosition", Float64MultiArray, queue_size=1)
        self.postureTaskTarePublisher = rospy.Publisher("/dreamer_controller/JPosTask/tare", Int32, queue_size=1)

        self.rightCartesianTaskGoalPublisher = rospy.Publisher("/dreamer_controller/RightHandPosition/goalPosition", Float64MultiArray, queue_size=1)
        self.rightCartesianTaskEnablePublisher = rospy.Publisher("/dreamer_controller/RightHandPosition/enabled", Int32, queue_size=1)
        self.rightCartesianTaskTarePublisher = rospy.Publisher("/dreamer_controller/RightHandPosition/tare", Int32, queue_size=1)

        self.leftCartesianTaskGoalPublisher = rospy.Publisher("/dreamer_controller/LeftHandPosition/goalPosition", Float64MultiArray, queue_size=1)
        self.leftCartesianTaskEnablePublisher = rospy.Publisher("/dreamer_controller/LeftHandPosition/enabled", Int32, queue_size=1)
        self.leftCartesianTaskTarePublisher = rospy.Publisher("/dreamer_controller/LeftHandPosition/tare", Int32, queue_size=1)

        self.rightOrientationTaskGoalPublisher = rospy.Publisher("/dreamer_controller/RightHandOrientation/goalVector", Float64MultiArray, queue_size=1)
        self.rightOrientationTaskEnablePublisher = rospy.Publisher("/dreamer_controller/RightHandOrientation/enabled", Int32, queue_size=1)
        self.rightOrientationTaskTarePublisher = rospy.Publisher("/dreamer_controller/RightHandOrientation/tare", Int32, queue_size=1)

        self.leftOrientationTaskGoalPublisher = rospy.Publisher("/dreamer_controller/LeftHandOrientation/goalVector", Float64MultiArray, queue_size=1)
        self.leftOrientationTaskEnablePublisher = rospy.Publisher("/dreamer_controller/LeftHandOrientation/enabled", Int32, queue_size=1)
        self.leftOrientationTaskTarePublisher = rospy.Publisher("/dreamer_controller/LeftHandOrientation/tare", Int32, queue_size=1)

        self.rightHandCmdPublisher = rospy.Publisher("/dreamer_controller/controlit/rightHand/powerGrasp", Bool, queue_size=1)
        self.leftGripperCmdPublisher = rospy.Publisher("/dreamer_controller/controlit/leftGripper/powerGrasp", Bool, queue_size=1)
        

    def postureTaskActualCallback(self, msg):
        self.currentPosture = msg.data

    def postureTaskErrorCallback(self, msg):
        self.postureError = msg.data

    def rightCartesianTaskActualCallback(self, msg):
        self.currentRightCartesianPos = msg.data

    def rightCartesianTaskErrorCallback(self, msg):
        self.rightCartesianPosError = msg.data

    def leftCartesianTaskActualCallback(self, msg):
        self.currentLeftCartesianPos = msg.data

    def leftCartesianTaskErrorCallback(self, msg):
        self.leftCartesianPosError = msg.data

    def rightOrientationTaskActualCallback(self, msg):
        self.currentRightOrientation = msg.data

    def rightOrientationTaskErrorCallback(self, msg):
        self.rightOrientationError = msg.data

    def leftOrientationTaskActualCallback(self, msg):
        self.currentLeftOrientation = msg.data

    def leftOrientationTaskErrorCallback(self, msg):
        self.leftOrientationError = msg.data

    # def linearInterpolate(self, trajectory, deltaTime):
    #     """
    #     Computes the position along the trajectory at the specified deltaTime.
    #     The trajectory is assumed to have millisecond resolution.
    #     deltaTime is assumed to be in seconds.
    #     """
    #     lowerIndex = math.floor(deltaTime * 1000)
    #     if lowerIndex >= len(trajectory):
    #         print "linearInterpolate: WARNING: deltaTime is beyond end of trajectory!\n"\
    #               "  - length of trajectory: {0}\n"\
    #               "  - delta time: {1}\n"\
    #               "Returning last position in trajectory.".format(len(trajectory), deltaTime)
    #         return trajectory[len(trajectory) - 1]
    #     elif lowerIndex == len(trajectory) - 1:
    #         return trajectory[len(trajectory) - 1]
    #     else:
    #         beforePoint = trajectory[lowerIndex]
    #         afterPoint = trajectory[lowerIndex + 1]

    #         # compute fraction of milliseconds that have elapsed
    #         fractionElapsed = (deltaTime * 1000) - math.floor(deltaTime * 1000)

    #         # do linear interpolation
    #         return (afterPoint - beforePoint) * fractionElapsed + beforePoint

    def getTimeSeconds(self):
        """
        Returns the current time in seconds.
        """
        return rospy.get_time()

    def issueTareCommands(self):
        print "Issuing tare commands..."
        self.postureTaskTarePublisher.publish(self.tareMsg)
        self.rightCartesianTaskTarePublisher.publish(self.tareMsg)
        self.leftCartesianTaskTarePublisher.publish(self.tareMsg)
        self.rightOrientationTaskTarePublisher.publish(self.tareMsg)
        self.leftOrientationTaskTarePublisher.publish(self.tareMsg)

        time.sleep(1)

        pauseCount = 0
        printWarning = False
        while not rospy.is_shutdown() and (self.postureError == None or \
              self.rightCartesianPosError == None or self.leftCartesianPosError == None or \
              self.rightOrientationError == None or self.leftOrientationError == None):
            time.sleep(0.5)
            pauseCount = pauseCount + 1
            if pauseCount > 5 and not printWarning:
                print "Waiting for error information..."
                printWarning = True

        return not rospy.is_shutdown()

    def doTare(self):

        # Wait for connection to ControlIt!
        pauseCount = 0
        printWarning = False
        while not rospy.is_shutdown() and (self.postureTaskTarePublisher.get_num_connections() == 0 or \
              self.rightCartesianTaskTarePublisher.get_num_connections() == 0 or \
              self.leftCartesianTaskTarePublisher.get_num_connections() == 0):
            time.sleep(0.5)
            pauseCount = pauseCount + 1
            if pauseCount > 5 and not printWarning:
                print "Waiting for connection to ControlIt!..."
                printWarning = True

        if rospy.is_shutdown():
            return False

        if not self.issueTareCommands():
            return False

        # while not rospy.is_shutdown() and self.actualPosture == None:
        #     time.sleep(0.1) # 10Hz 

        # print "Got actual posture data, updating the goal of the posture task..."

        # self.postureGoalMsg.data = self.actualPosture
        # self.postureTaskGoalPublisher.publish(self.postureGoalMsg)
        
        done = False
        while not done:

            resultList = [["Index", "Joint Name", "Error (rad)", "Error (deg)"]]
            resultList.append(["-----", "----------", "-----------", "-----------"])
            
            JOINT_NAMES = ["torso_lower_pitch", "torso_upper_pitch",
                           "left_shoulder_extensor", "left_shoulder_abductor", 
                           "left_shoulder_rotator", "left_elbow",
                           "left_wrist_rotator", "left_wrist_pitch", "left_wrist_yaw", 
                           "right_shoulder_extensor", "right_shoulder_abductor", 
                           "right_shoulder_rotator", "right_elbow", "right_wrist_rotator",
                           "right_wrist_pitch", "right_wrist_yaw"]

            index = 0
            for jointError in self.postureError:
                errorRad = jointError
                errorDeg = errorRad / 3.14 * 180
                entry = [str(index), JOINT_NAMES[index], str(errorRad), str(errorDeg)]
                resultList.append(entry)
                index = index + 1
        
            col_width = max(len(word) for row in resultList for word in row) + 2  # padding
            resultTable =""
            for row in resultList:
                resultTable = resultTable + "\n    " + "".join(word.ljust(col_width) for word in row) 
    
            index = raw_input("Current errors are:\n"\
                              " - Posture task:    {0}\n"\
                              " - Right hand Cartesian position:\n    {1}\n"\
                              " - Left hand Cartesian position:\n    {2}\n"\
                              " - Right hand orientation:\n    {3}\n"\
                              " - Left hand orientation:\n    {4}\n"\
                              "If error is low undo e-stop then type enter, otherwise type 'r' to retry or 'q' to quit.\n".format(
                resultTable, self.rightCartesianPosError, self.leftCartesianPosError, 
                self.rightOrientationError, self.leftOrientationError))

            if "q" == index:
                print "Exiting..."
                return False
            elif "r" == index:
                self.issueTareCommands()
            else:
                done = True

        return True

    def goToStartPosition(self):

        # Define the waypoints
        # Note waypoints are formatted as: [[x, y, z], ...]
        rightHandCartesianWP = []
        rightHandCartesianWP.append(self.currentRightCartesianPos)
        rightHandCartesianWP.append([0.034156182237965314, -0.2536961667775097, 0.7942923566248334])
        rightHandCartesianWP.append([-0.03852115301282585, -0.36702885542756375, 1.0044042662878492])
        rightHandCartesianWP.append([-0.0275400056213187, -0.4346278435022028, 1.109258357008881])
        rightHandCartesianWP.append([0.16786527968278075, -0.48763818929105546, 1.2849643133074693])
        rightHandCartesianWP.append([0.2561600552895205, -0.36355117909588747, 1.2737345840311838])

        rightHandOrientationWP = []
        rightHandOrientationWP.append(self.currentRightOrientation)
        rightHandOrientationWP.append([0.9239165069202464, -0.16720064850712463, 0.34412531348200354])
        rightHandOrientationWP.append([0.7104721853318615, 0.5004153180136336, 0.4947866038678526])
        rightHandOrientationWP.append([0.5901777988221774, 0.748119163684364, 0.3033280117391358])
        rightHandOrientationWP.append([-0.043115032422085975, 0.9965622982370774, 0.07074376093816886])
        rightHandOrientationWP.append([0.34390539739454545, -0.566640981750115, 0.7487636980010218])

        # Create the trajectory generators
        rightHandCartesianTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(rightHandCartesianWP)
        rightHandOrientationTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(rightHandOrientationWP)

        TOTAL_TRAVEL_TIME = 10.0 # seconds
        rightHandCartesianTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        rightHandOrientationTG.generateTrajectory(TOTAL_TRAVEL_TIME)

        index = raw_input("Done generating trajectories. Ready to to go start position? Y/n\n")

        if index == "N" or index == "n":
            return False  # quit

        # Follow the trajectories
        startTime = self.getTimeSeconds()
        done = False

        while not done and not rospy.is_shutdown():
            deltaTime = self.getTimeSeconds() - startTime

            goalRightHandCartPos = None
            goalRightHandOrientation = None

            if deltaTime > TOTAL_TRAVEL_TIME:
                goalRightHandCartPos = rightHandCartesianTG.getLastPoint()
                goalRightHandOrientation = rightHandOrientationTG.getLastPoint()
                done = True
            else:
                goalRightHandCartPos = rightHandCartesianTG.getPoint(deltaTime)
                goalRightHandOrientation = rightHandOrientationTG.getPoint(deltaTime)

            # Save the new goals in ROS messages
            self.rightHandCartesianGoalMsg.data = goalRightHandCartPos
            self.rightHandOrientationGoalMsg.data = goalRightHandOrientation

            # Publish the ROS messages
            self.rightCartesianTaskGoalPublisher.publish(self.rightHandCartesianGoalMsg)
            self.rightOrientationTaskGoalPublisher.publish(self.rightHandOrientationGoalMsg)

            if not done:
                rospy.sleep(0.01) # 100Hz

        print "Done going to start position!"
        return True


    def run(self):
        """
        Runs the demo 1 behavior.
        """

        if not self.doTare():
            return

        if not self.goToStartPosition():
            return

        # rightGoalPub = rospy.Publisher("/dreamer_controller/RightHandPosition/goalPosition", Float64MultiArray, queue_size=1)
        # leftGoalPub = rospy.Publisher("/dreamer_controller/LeftHandPosition/goalPosition", Float64MultiArray, queue_size=1)

        # # Perform cubic spline interpolation for initial swing
        # tt = np.linspace(0, INIT_TRAJ_TIME, 5)             # start, end, number of values
        # xx = np.array(WAYPOINT_X_SWING)
        # yy = np.array(WAYPOINT_Y_SWING)
        # zz = np.array(WAYPOINT_Z_SWING)

        # fInitX = interp1d(tt, xx, kind='cubic')   # interpolation functions
        # fInitY = interp1d(tt, yy, kind='cubic')
        # fInitZ = interp1d(tt, zz, kind='cubic')

        # # Generate the interpolated points along initial / final trajectory with 1 millisecond resolution
        # # To get millisecond resolution of the trajectory, the number of 
        # # interpolation points is (INIT_TRAJ_TIME * 1000)
        # tInitT = np.linspace(0, INIT_TRAJ_TIME, INIT_TRAJ_TIME * 1000)   # start, end, number of values to interpolate along trajectory
        # tInitX = fInitX(tInitT)
        # tInitY = fInitY(tInitT)
        # tInitZ = fInitZ(tInitT)

        # fig = plt.figure()
        # a3d = Axes3D(fig)
        # a3d.scatter(xs = xx_interp, ys = yy_interp, zs = zz_interp, zdir=u'z', label='xs=x, ys=y, zdir=z')
        # plt.show()
        # return

        # Save the interpolated coordinates into a new datastructure
        # INITIAL_SWING_TRAJECTORY = []
        # for ii in range(0, tt_interp.size):
        #     waypoint = [xx_interp[ii], yy_interp[ii], zz_interp[ii]]
        #     INITIAL_SWING_TRAJECTORY.append(waypoint)

        # plt.plot(tt_interp, xx_interp, 'o', tt_interp, yy_interp, '-' , tt_interp, zz_interp, '--')
        # plt.legend(['x', 'y', 'z'], loc='best')
        # plt.show()
        # return

        # # Perform cubic spline interpolation for wave
        # tt = np.linspace(0, WAVE_TRAJ_TIME, 7)             # start, end, number of values
        # xx = np.array(WAYPOINT_X_WAVE)
        # yy = np.array(WAYPOINT_Y_WAVE)
        # zz = np.array(WAYPOINT_Z_WAVE)

        # fWaveX = interp1d(tt, xx, kind='cubic')    # interpolation functions
        # fWaveY = interp1d(tt, yy, kind='cubic')
        # fWaveZ = interp1d(tt, zz, kind='cubic')

        # # Generate the interpolated points along wave up/down trajectory with 1 millisecond resolution
        # tWaveT = np.linspace(0, WAVE_TRAJ_TIME, WAVE_TRAJ_TIME * 1000)
        # tWaveX = fWaveX(tWaveT)
        # tWaveY = fWaveY(tWaveT)
        # tWaveZ = fWaveZ(tWaveT)

        # Save the interpolated coordinates into a new datastructure
        # WAVE_TRAJECTORY = []
        # for ii in range(0, tt_interp.size):
        #     waypoint = [xx_interp[ii], yy_interp[ii], zz_interp[ii]]
        #     WAVE_TRAJECTORY.append(waypoint)

        # printInit = printWave = printFinal = True

        # # state = STATE_GO_TO_START
        # goalIndex = 0
        # numWaves = 0     # the number of times the robot has waved already

        # startTime = self.getTimeSeconds()

        # TOTAL_CYCLE_TIME = WAVE_TRAJ_TIME * 2 * NUM_WAVES + INIT_TRAJ_TIME * 2
        # END_WAVE_TIME = INIT_TRAJ_TIME + WAVE_TRAJ_TIME * 2 * NUM_WAVES

        # test code
        # self.linearInterpolate(tInitX, 2500)
        # return

        # while not rospy.is_shutdown():

        #     deltaTime = self.getTimeSeconds() - startTime

        #     # check for completion of entire cycle
        #     if deltaTime > TOTAL_CYCLE_TIME:
        #         startTime = startTime + TOTAL_CYCLE_TIME
        #         deltaTime = rospy.get_time() - startTime
        #         printInit = printWave = printFinal = True

        #     if deltaTime < INIT_TRAJ_TIME:
        #         # follow initial trajectory

        #         if printInit:
        #             print "Executing initial trajectory..."
        #             printInit = False
                
        #         self.leftHandCartesianGoalMsg.data[0] = self.rightHandCartesianGoalMsg.data[0] = self.linearInterpolate(tInitX, deltaTime)
        #         self.leftHandCartesianGoalMsg.data[1] = self.rightHandCartesianGoalMsg.data[1] = self.linearInterpolate(tInitY, deltaTime)
        #         self.leftHandCartesianGoalMsg.data[2] = self.rightHandCartesianGoalMsg.data[2] = self.linearInterpolate(tInitZ, deltaTime)

        #     elif deltaTime < END_WAVE_TIME:
        #         # follow wave trajectory
        #         if printWave:
        #             print "Executing wave trajectories..."
        #             printWave = False

        #         count = math.floor((deltaTime - INIT_TRAJ_TIME) / WAVE_TRAJ_TIME)

        #         # this is the delta time if we're waving up
        #         trajTime = deltaTime - INIT_TRAJ_TIME - (count * WAVE_TRAJ_TIME)

        #         if count % 2 == 1:
        #             # we are waving down, compute delta time for down trajectory
        #             trajTime = WAVE_TRAJ_TIME - trajTime

        #         # print "count = {0}, deltaTime = {1}, trajTime = {2}".format(count, deltaTime, trajTime)

        #         self.leftHandCartesianGoalMsg.data[0] = self.rightHandCartesianGoalMsg.data[0] = self.linearInterpolate(tWaveX, trajTime)
        #         self.leftHandCartesianGoalMsg.data[1] = self.rightHandCartesianGoalMsg.data[1] = self.linearInterpolate(tWaveY, trajTime)
        #         self.leftHandCartesianGoalMsg.data[2] = self.rightHandCartesianGoalMsg.data[2] = self.linearInterpolate(tWaveZ, trajTime)

        #     else:
        #         # follow final trajectory
        #         if printFinal:
        #             print "Executing final trajectory..."
        #             printFinal = False

        #         deltaTime = INIT_TRAJ_TIME - (deltaTime - END_WAVE_TIME)
        #         self.leftHandCartesianGoalMsg.data[0] = self.rightHandCartesianGoalMsg.data[0] = self.linearInterpolate(tInitX, deltaTime)
        #         self.leftHandCartesianGoalMsg.data[1] = self.rightHandCartesianGoalMsg.data[1] = self.linearInterpolate(tInitY, deltaTime)
        #         self.leftHandCartesianGoalMsg.data[2] = self.rightHandCartesianGoalMsg.data[2] = self.linearInterpolate(tInitZ, deltaTime)
            
        #     # flip the sign of the left hand's Y-axis goal
        #     self.leftHandCartesianGoalMsg.data[Y_AXIS_INDEX] = -1 * self.leftHandCartesianGoalMsg.data[Y_AXIS_INDEX]  

        #     # print "publishing goal:\n  - left: {0}\n  - right: {1}".format(self.leftHandCartesianGoalMsg.data, self.rightHandCartesianGoalMsg.data)

        #     rightGoalPub.publish(self.rightHandCartesianGoalMsg)
        #     leftGoalPub.publish(self.leftHandCartesianGoalMsg)

        #     time.sleep(0.01) # 100Hz 
        #     # time.sleep(0.1) # 10Hz 

        print "Demo 1 done!"

# Main method
if __name__ == "__main__":

    rospy.init_node('Demo1_ProductDisassembly', anonymous=True)

    demo1 = Demo1_ProductDisassembly()
    # t = threading.Thread(target=Demo1_ProductDisassembly.start)
    # t.start()
    demo1.run()


    # rospy.spin()