#!/usr/bin/env python

'''
Enables users to telemanipulate Dreamer's end effectors via CARL.
Uses WBOSC configured with end effector Cartesian position and orientation.

-----------------
Dependency notes:

If you're using Python 2.7, you need to install Python's
enum package. Download it from here: https://pypi.python.org/pypi/enum34#downloads
Then run:
  $ sudo python setup.py install

Visualzing the FSM requires smach_viewer:
  $ sudo apt-get install ros-indigo-smach-viewer
You will need to modify /opt/ros/indigo/lib/python2.7/dist-packages/xdot/xdot.py
lines 487, 488, 593, and 594 to contain self.read_float() instead of self.read_number().

-----------------
Usage Notes:

To issue a command using the command line:

  exit:
    $ rostopic pub --once /demo9/cmd std_msgs/Int32 'data: 0'

  go to ready:
    $ rostopic pub --once /demo9/cmd std_msgs/Int32 'data: 1'

  go to idle:
    $ rostopic pub --once /demo9/cmd std_msgs/Int32 'data: 2'

To visualize FSM:
  $ rosrun smach_viewer smach_viewer.py
'''

import sys, getopt     # for getting and parsing command line arguments
import time
import math
import threading
import rospy
import smach
import smach_ros

from enum import IntEnum

from std_msgs.msg import Float64, Float64MultiArray, MultiArrayDimension, Bool, Int32

import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import DreamerInterface
import Trajectory
import TrajectoryGeneratorCubicSpline

import roslib; roslib.load_manifest('controlit_dreamer_integration')
import TrapezoidVelocityTrajGen

# The previous demos that we would like to execute
import Demo4_HandWave
import Demo5_HandShake
import Demo7_HookemHorns

ENABLE_USER_PROMPTS = False

# Shoulder abductors about 10 degrees away from body and elbows bent 90 degrees
# DEFAULT_POSTURE = [0.0, 0.0,                                    # torso
#                    0.0, 0.174532925, 0.0, 1.57, 0.0, 0.0, 0.0,  # left arm
#                    0.0, 0.174532925, 0.0, 1.57, 0.0, 0.0, 0.0]  # right arm

# Shoulder abductors and elbows at about 10 degrees
DEFAULT_POSTURE = [0.0, 0.0,                                    # torso
                   0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
                   0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0]  # right arm

# Define the commands that can be received from the CARL user interface.
class Command(IntEnum):
    CMD_NONE = 0
    CMD_GOTO_READY = 1
    CMD_GOTO_IDLE = 2
    CMD_MOVE_RIGHT_HAND_FORWARD = 3
    CMD_MOVE_RIGHT_HAND_BACKWARD = 4
    CMD_MOVE_RIGHT_HAND_UP = 5
    CMD_MOVE_RIGHT_HAND_DOWN = 6
    CMD_MOVE_RIGHT_HAND_LEFT = 7
    CMD_MOVE_RIGHT_HAND_RIGHT = 8
    CMD_MOVE_LEFT_HAND_FORWARD = 9
    CMD_MOVE_LEFT_HAND_BACKWARD = 10
    CMD_MOVE_LEFT_HAND_UP = 11
    CMD_MOVE_LEFT_HAND_DOWN = 12
    CMD_MOVE_LEFT_HAND_LEFT = 13
    CMD_MOVE_LEFT_HAND_RIGHT = 14
    CMD_TOGGLE_RIGHT_HAND = 15
    CMD_TOGGLE_LEFT_GRIPPER = 16
    CMD_EXECUTE_DEMO = 17

# Define the Cartesian directions. This is used by the MoveCartesianState.
class CartesianDirection(IntEnum):
    UP = 0
    DOWN = 1
    LEFT = 2
    RIGHT = 3
    FORWARD = 4
    BACKWARD = 5

#==================================================================================
# The following parameters are used by the trapezoid velocity trajectory generator

# The time each trajectory should take
TIME_GO_TO_READY = 5.0
TIME_GO_TO_IDLE = 7.0

GO_BACK_TO_READY_SPEED = 0.01 # 1 cm/s

# The speed at which the Cartesian position should change
TRAVEL_SPEED = 0.02  # 2 cm per second
ACCELERATION = 0.06  # 1 cm/s^2
DECELERATION = 0.06  # 1 cm/s^2

# The update frequency when adjusting the Cartesian position
TRAJECTORY_UPDATE_FREQUENCY = 100

X_AXIS = 0
Y_AXIS = 1
Z_AXIS = 2

# Define the Cartesian movement increment
CARTESIAN_MOVE_DELTA = 0.01  # 1 cm movement increments


class TrajectoryState(smach.State):
    """
    A SMACH state that makes the robot follow a trajectory.
    """

    def __init__(self, dreamerInterface, traj):
        """
        The constructor.

        Keyword Parameters:
          - dreamerInterface: The object to which to provide the trajectory.
          - traj: The trajectory to follow.
        """

        smach.State.__init__(self, outcomes=["done", "exit"])
        self.dreamerInterface = dreamerInterface
        self.traj = traj

    def execute(self, userdata):
        rospy.loginfo('Executing TrajectoryState')

        if self.dreamerInterface.followTrajectory(self.traj):
            return "done"
        else:
            return "exit"

class GoBackToReadyState(smach.State):
    """
    A SMACH state that makes the end effectors go back to the ready state,
    which is also the beginning of the GoToIdle trajectory.
    """

    def __init__(self, dreamerInterface, goToIdleTraj):
        """
        The constructor.

        Keyword Parameters:
          - dreamerInterface: The object to which to provide the trajectory.
          - goToIdleTraj: The goToIdle trajectory. This is used to determine the final
                          waypoint of the GoBackToReady trajectory.
        """

        smach.State.__init__(self, outcomes=["done", "exit"])
        self.dreamerInterface = dreamerInterface
        self.goToIdleTraj = goToIdleTraj

    def dist(self, point1, point2):
        return math.sqrt(math.pow(point1[0] - point2[0], 2) + \
                         math.pow(point1[1] - point2[1], 2) + \
                         math.pow(point1[2] - point2[2], 2))

    def execute(self, userdata):
        rospy.loginfo("GoBackToReadyState: Executing GoBackToReadyState")

        # Let's make the trajectory's duration a function of the Cartesian distance to traverse
        rhCurrCartPos = self.dreamerInterface.rightHandCartesianGoalMsg.data
        lhCurrCartPos = self.dreamerInterface.leftHandCartesianGoalMsg.data

        rhFinalCartPos = self.goToIdleTraj.rhCartWP[0]
        lhFinalCartPos = self.goToIdleTraj.lhCartWP[0]

        travelDist = max(self.dist(rhCurrCartPos, rhFinalCartPos), self.dist(lhCurrCartPos, lhFinalCartPos))

        if travelDist < 0.01:  # less than 1cm of movement, return done
            rospy.loginfo("GoBackToReadyState: zero travel distance, returning done")
            return "done"

        travelTime = travelDist / GO_BACK_TO_READY_SPEED
        rospy.loginfo("GoBackToReadyState: distance to travel is {0}, travel time is {1}".format(travelDist, travelTime))

        # Create a trajectory to go back to the start / idle position
        traj = Trajectory.Trajectory("GoBackToReady", travelTime)  # TODO: make the time be proportional to the distance that needs to be traveled

        # Initial goal is current goal
        traj.setInitRHCartWP(rhCurrCartPos)
        traj.setInitLHCartWP(lhCurrCartPos)
        traj.setInitRHOrientWP(self.dreamerInterface.rightHandOrientationGoalMsg.data)
        traj.setInitLHOrientWP(self.dreamerInterface.leftHandOrientationGoalMsg.data)
        traj.setInitPostureWP(self.dreamerInterface.postureGoalMsg.data)

        # repeat the same point twice (this is to ensure the trajectory has sufficient number of points to perform cubic spline)
        traj.addRHCartWP(self.dreamerInterface.rightHandCartesianGoalMsg.data)
        traj.addRHOrientWP(self.dreamerInterface.rightHandOrientationGoalMsg.data)
        traj.addLHCartWP(self.dreamerInterface.leftHandCartesianGoalMsg.data)
        traj.addLHOrientWP(self.dreamerInterface.leftHandOrientationGoalMsg.data)
        traj.addPostureWP(self.dreamerInterface.postureGoalMsg.data)

        traj.addRHCartWP(self.dreamerInterface.rightHandCartesianGoalMsg.data)
        traj.addRHOrientWP(self.dreamerInterface.rightHandOrientationGoalMsg.data)
        traj.addLHCartWP(self.dreamerInterface.leftHandCartesianGoalMsg.data)
        traj.addLHOrientWP(self.dreamerInterface.leftHandOrientationGoalMsg.data)
        traj.addPostureWP(self.dreamerInterface.postureGoalMsg.data)

        # final way point is initial waypoint of GoToIdle trajectory
        traj.addRHCartWP(rhFinalCartPos)
        traj.addRHOrientWP(self.goToIdleTraj.rhOrientWP[0])
        traj.addLHCartWP(lhFinalCartPos)
        traj.addLHOrientWP(self.goToIdleTraj.lhOrientWP[0])
        traj.addPostureWP(self.goToIdleTraj.jPosWP[0])

        # repeat the same final point (this is to ensure the trajectory has sufficient number of points to perform cubic spline)
        traj.addRHCartWP(self.goToIdleTraj.rhCartWP[0])
        traj.addRHOrientWP(self.goToIdleTraj.rhOrientWP[0])
        traj.addLHCartWP(self.goToIdleTraj.lhCartWP[0])
        traj.addLHOrientWP(self.goToIdleTraj.lhOrientWP[0])
        traj.addPostureWP(self.goToIdleTraj.jPosWP[0])

        if self.dreamerInterface.followTrajectory(traj):
            return "done"
        else:
            return "exit"


class AwaitCommandState(smach.State):
    """
    A SMACH state that waits for a command to arrive. It subscribes to a 
    ROS topic over which commands are published and triggers a transition
    based on the received command.
    """

    def __init__(self, moveCartesianState, goToIdleState):
        """
        The constructor.

        Keyword parameters:
          - moveCartesianState: The SMACH state that moves the cartesian position
          - goToIdleState: The SMACH state that moves the robot back to the idle state
        """

        # Initialize parent class
        smach.State.__init__(self, outcomes=[
            "go_to_ready",
            "go_back_to_ready",
            "move_cartesian_position",
            "toggle_end_effector",
            "execute_demo",
            "done",
            "exit"],
            output_keys=['endEffectorSide', 'demoName'])

        self.moveCartesianState = moveCartesianState
        self.goToIdleState = goToIdleState

        # Initialize local variables
        self.rcvdCmd = False
        self.sleepPeriod = 0.5  # in seconds
        self.cmd = Command.CMD_NONE
        self.isIdle = True  # Initially we are in idle state

        # Register a ROS topic listener
        self.demoCmdSubscriber  = rospy.Subscriber("/demo9/cmd", Int32, self.demo9CmdCallback)
        self.demoDonePublisher = rospy.Publisher("/demo9/done",  Int32, queue_size=1)

        self.demoCmdSubscriber  = rospy.Subscriber("/demo8/cmd", Int32, self.demo8CmdCallback)
        self.demoDonePublisher = rospy.Publisher("/demo8/done",  Int32, queue_size=1)

    def demo9CmdCallback(self, msg):
        """
        The callback method of the command ROS topic subscriber.
        """

        self.cmd = msg.data
        self.rcvdCmd = True

    def demo8CmdCallback(self, msg):
        """
        The callback method of the command ROS topic subscriber.
        """

        self.cmd = Command.CMD_EXECUTE_DEMO

        # TODO: Get rid of hard-coded demo IDs. CARL Bridge should send this info to us directly.
        DEMO_WAVE = 0
        DEMO_SHAKE = 1
        DEMO_HOOKEM_HORNS = 2

        if msg.data == DEMO_WAVE:
            self.demoName = "HandWave"
        elif msg.data == DEMO_SHAKE:
            self.demoName = "HandShake"
        elif msg.data == DEMO_HOOKEM_HORNS:
            self.demoName = "HookemHorns"
        else:
            rospy.logwarn("Unknown demo {0}".format(msg.data))
            return

        self.rcvdCmd = True

    def execute(self, userdata):
        """
        Waits for a command to arrive. Then processes the command.
        """

        rospy.loginfo('AwaitCommandState: Executing...')

        self.rcvdCmd = False # reset this variable (ignore any commands sent prior to this state executing)

        while not self.rcvdCmd:
            rospy.sleep(self.sleepPeriod)

        if rospy.is_shutdown():
            return "exit"
        else:
            if self.cmd == Command.CMD_NONE:
                return "exit"
            elif self.cmd == Command.CMD_GOTO_READY:

                # Only go to ready position if we're in idle
                if self.isIdle:
                    self.isIdle = False
                    return "go_to_ready"
                else:
                    return "done"
            elif self.cmd == Command.CMD_GOTO_IDLE:

                # Only go to idle if we're not in idle
                if not self.isIdle:
                    self.isIdle = True
                    return "go_back_to_ready"
                else:
                    return "done"
            elif self.cmd == Command.CMD_MOVE_RIGHT_HAND_FORWARD:
                self.moveCartesianState.setParameters(endEffector="right", direction=CartesianDirection.FORWARD)
                return "move_cartesian_position"
            elif self.cmd == Command.CMD_MOVE_RIGHT_HAND_BACKWARD:
                self.moveCartesianState.setParameters(endEffector="right", direction=CartesianDirection.BACKWARD)
                return "move_cartesian_position"
            elif self.cmd == Command.CMD_MOVE_RIGHT_HAND_UP:
                self.moveCartesianState.setParameters(endEffector="right", direction=CartesianDirection.UP)
                return "move_cartesian_position"
            elif self.cmd == Command.CMD_MOVE_RIGHT_HAND_DOWN:
                self.moveCartesianState.setParameters(endEffector="right", direction=CartesianDirection.DOWN)
                return "move_cartesian_position"
            elif self.cmd == Command.CMD_MOVE_RIGHT_HAND_LEFT:
                self.moveCartesianState.setParameters(endEffector="right", direction=CartesianDirection.LEFT)
                return "move_cartesian_position"
            elif self.cmd == Command.CMD_MOVE_RIGHT_HAND_RIGHT:
                self.moveCartesianState.setParameters(endEffector="right", direction=CartesianDirection.RIGHT)
                return "move_cartesian_position"
            elif self.cmd == Command.CMD_MOVE_LEFT_HAND_FORWARD:
                self.moveCartesianState.setParameters(endEffector="left", direction=CartesianDirection.FORWARD)
                return "move_cartesian_position"
            elif self.cmd == Command.CMD_MOVE_LEFT_HAND_BACKWARD:
                self.moveCartesianState.setParameters(endEffector="left", direction=CartesianDirection.BACKWARD)
                return "move_cartesian_position"
            elif self.cmd == Command.CMD_MOVE_LEFT_HAND_UP:
                self.moveCartesianState.setParameters(endEffector="left", direction=CartesianDirection.UP)
                return "move_cartesian_position"
            elif self.cmd == Command.CMD_MOVE_LEFT_HAND_DOWN:
                self.moveCartesianState.setParameters(endEffector="left", direction=CartesianDirection.DOWN)
                return "move_cartesian_position"
            elif self.cmd == Command.CMD_MOVE_LEFT_HAND_LEFT:
                self.moveCartesianState.setParameters(endEffector="left", direction=CartesianDirection.LEFT)
                return "move_cartesian_position"
            elif self.cmd == Command.CMD_MOVE_LEFT_HAND_RIGHT:
                self.moveCartesianState.setParameters(endEffector="left", direction=CartesianDirection.RIGHT)
                return "move_cartesian_position"
            elif self.cmd == Command.CMD_TOGGLE_LEFT_GRIPPER:
                userdata.endEffectorSide = "left"
                return "toggle_end_effector"
            elif self.cmd == Command.CMD_TOGGLE_RIGHT_HAND:
                userdata.endEffectorSide = "right"
                return "toggle_end_effector"
            elif self.cmd == Command.CMD_EXECUTE_DEMO:
                if self.isIdle:
                    userdata.demoName = self.demoName
                    return "execute_demo"
                else:
                    rospy.logwarn("AwaitCommandState: ERROR: Attempted to run demo from a non-idle state!")
                    return "done"
            else:
                rospy.loginfo("AwaitCommandState: ERROR: Received a unknown command ({0})! Returning exit".format(self.cmd))
                return "exit"

class MoveCartesianState(smach.State):
    """
    A SMACH state that moves the Cartesian position of a point on the robot.
    """

    def __init__(self, dreamerInterface):
        """
        The constructor.

        Keyword Parameters:
          - dreamerInterface: The object providing access to Dreamer hardware.
        """

        smach.State.__init__(self, outcomes=["done", "exit"])
        self.dreamerInterface = dreamerInterface
        self.trajGen = TrapezoidVelocityTrajGen.TrapezoidVelocityTrajGen()

    def setParameters(self, endEffector, direction):
        """
        Sets the end effector and movement direction parameters.

        Keyword Parameters:
          - dreamerInterface: The object providing access to Dreamer hardware.
          - endEffector: Which end effector to adjust.
          - direction: The direction to adjust the end effector's Cartesian position.
        """

        self.endEffector = endEffector
        self.direction = direction

    def directionToString(self, direction):
        """
        Returns a string representation of the direction command.

        Keyword Parameters:
          - direction: the direction to convert into a string.
        """

        if direction == CartesianDirection.UP:
            return "UP"
        elif direction == CartesianDirection.DOWN:
            return "DOWN"
        elif direction == CartesianDirection.LEFT:
            return "LEFT"
        elif direction == CartesianDirection.RIGHT:
            return "RIGHT"
        elif direction == CartesianDirection.FORWARD:
            return "FORWARD"
        elif direction == CartesianDirection.BACKWARD:
            return "BACKWARD"
        else:
            return "UNKNOWN"

    def axisNameToString(self, axisID):
        if axisID == 0:
            return "X"
        if axisID == 1:
            return "Y"
        if axisID == 2:
            return "Z"

    def execute(self, userdata):
        rospy.loginfo('MoveCartesianState: Executing, end effector = {0}, direction = {1}'.format(
            self.endEffector, self.directionToString(self.direction)))

        # Determine the current Cartesian position
        if self.endEffector == "right":
            self.origCartesianPosition = self.dreamerInterface.rightHandCartesianGoalMsg.data
        else:
            self.origCartesianPosition = self.dreamerInterface.leftHandCartesianGoalMsg.data

        if self.origCartesianPosition == None:
            rospy.loginfo("MoveCartesianState: ERROR: Unable to get current Cartesian position. Aborting the move. Returning done.")
            return "done"

        # Compute new Cartesian position

        if self.direction == CartesianDirection.FORWARD:
            oldPos = self.origCartesianPosition[X_AXIS]
            newPos = self.origCartesianPosition[X_AXIS] + CARTESIAN_MOVE_DELTA
            self.axisOfMovement = X_AXIS

        if self.direction == CartesianDirection.BACKWARD:
            oldPos = self.origCartesianPosition[X_AXIS]
            newPos = self.origCartesianPosition[X_AXIS] - CARTESIAN_MOVE_DELTA
            self.axisOfMovement = X_AXIS

        if self.direction == CartesianDirection.LEFT:
            oldPos = self.origCartesianPosition[Y_AXIS]
            newPos = self.origCartesianPosition[Y_AXIS] + CARTESIAN_MOVE_DELTA
            self.axisOfMovement = Y_AXIS

        if self.direction == CartesianDirection.RIGHT:
            oldPos = self.origCartesianPosition[Y_AXIS]
            newPos = self.origCartesianPosition[Y_AXIS] - CARTESIAN_MOVE_DELTA
            self.axisOfMovement = Y_AXIS

        if self.direction == CartesianDirection.UP:
            oldPos = self.origCartesianPosition[Z_AXIS]
            newPos = self.origCartesianPosition[Z_AXIS] + CARTESIAN_MOVE_DELTA
            self.axisOfMovement = Z_AXIS

        if self.direction == CartesianDirection.DOWN:
            oldPos = self.origCartesianPosition[Z_AXIS]
            newPos = self.origCartesianPosition[Z_AXIS] - CARTESIAN_MOVE_DELTA
            self.axisOfMovement = Z_AXIS

        rospy.loginfo("MoveCartesianState: Modifying goal Cartesian position of {0} axis to be from {1} to {2}".format(
            self.axisNameToString(self.axisOfMovement), oldPos, newPos))

        # Initialize the trajectory generator
        self.trajGen.init(oldPos, newPos, TRAVEL_SPEED, ACCELERATION, DECELERATION,
            TRAJECTORY_UPDATE_FREQUENCY)

        # Start the trajectory! The callback method is updateTrajGoals(...), which is defined below.
        self.trajGen.start(self)

        # Wait 2 seconds to allow convergence before returning
        # rospy.sleep(2)

        if rospy.is_shutdown():
             return "exit"
        else:
            return "done"

    def updateTrajGoals(self, goalPos, goalVel, done):

        # Initialize the new Cartesian position to be the original Cartesian position
        newCartesianPosition = self.origCartesianPosition

        # Save the new goal Cartesian position
        if self.axisOfMovement == X_AXIS:
            newCartesianPosition[X_AXIS] = goalPos
        elif self.axisOfMovement == Y_AXIS:
            newCartesianPosition[Y_AXIS] = goalPos
        else:
            newCartesianPosition[Z_AXIS] = goalPos

        rospy.loginfo("MoveCartesianState: Updating goal of {0} and to be {1}.".format(
            self.endEffector, newCartesianPosition))

        if self.endEffector == "right":
            self.dreamerInterface.updateRightHandCartesianPosition(newCartesianPosition)
        else:
            self.dreamerInterface.updateLeftHandCartesianPosition(newCartesianPosition)            


class EndEffectorToggleState(smach.State):
    """
    A SMACH state that toggles the state of an end effector.
    """

    def __init__(self, dreamerInterface):
        """
        The constructor.

        Keyword Parameters:
          - dreamerInterface: The object to which to provide the trajectory.
        """

        smach.State.__init__(self, outcomes=["done", "exit"], input_keys=['endEffectorSide'])
        self.dreamerInterface = dreamerInterface
        self.isClosed = False  # Assume initial state is relaxed

    def execute(self, userdata):
        rospy.loginfo('Executing EndEffectorToggleState, side = {0}'.format(userdata.endEffectorSide))

        if userdata.endEffectorSide == "right":
            if self.isClosed:
                self.dreamerInterface.openRightHand()
            else:
                self.dreamerInterface.closeRightHand()
        else:
            if self.isClosed:
                self.dreamerInterface.openLeftGripper()
            else:
                self.dreamerInterface.closeLeftGripper()

        self.isClosed = not self.isClosed

        # wait 2 seconds to allow convergence before returning
        rospy.sleep(2)

        if rospy.is_shutdown():
             return "exit"
        else:
            return "done"

class SleepState(smach.State):
    """
    Makes the robot pause for a specified amount of time.
    """

    def __init__(self, goodResult, period):
        smach.State.__init__(self, outcomes=[goodResult, "exit"])
        self.goodResult = goodResult
        self.period = period

    def execute(self, userdata):
        rospy.loginfo("Executing SleepState")
        rospy.sleep(self.period)

        if rospy.is_shutdown():
            return "exit"
        else:
            return self.goodResult

class ExecuteDemoState(smach.State):
    """
    Executes a demo.
    """

    def __init__(self, dreamerInterface):
        smach.State.__init__(self, outcomes=["done", "exit"], input_keys=['demoName'])

        # Instantiate the previous demos
        self.handWaveDemo = Demo4_HandWave.Demo4_HandWave(dreamerInterface)
        self.handShakeDemo = Demo5_HandShake.Demo5_HandShake(dreamerInterface)
        self.hookemHornsDemo = Demo7_HookemHorns.Demo7_HookemHorns(dreamerInterface)

    def execute(self, userdata):
        rospy.loginfo('Executing demo {0}'.format(userdata.demoName))

        if userdata.demoName == "HandWave":
            print "Starting the Hand Wave Demo!"
            self.handWaveDemo.run(enablePrompts = False)
        elif userdata.demoName == "HandShake":
            print "Starting the Hand Shake Demo!"
            self.handShakeDemo.run(enablePrompts = False)
        elif userdata.demoName == "HookemHorns":
            print "Starting the Hook'em Horns Demo!"
            self.hookemHornsDemo.run(enablePrompts = False)
        else:
            rospy.logwarn("Unknown demo {0}".format(userdata.demoName))

        if rospy.is_shutdown():
             return "exit"
        else:
            return "done"

class Demo9_CARL_Telemanipulation:
    """
    The primary class that implement's the demo's FSM.
    """

    def __init__(self):
        self.dreamerInterface = DreamerInterface.DreamerInterface(ENABLE_USER_PROMPTS)

    def createTrajectories(self):

        # ==============================================================================================
        # Define the GoToReady trajectory
        self.trajGoToReady = Trajectory.Trajectory("GoToReady", TIME_GO_TO_READY)

        # These are the initial values as specified in the YAML ControlIt! configuration file
        self.trajGoToReady.setInitRHCartWP([0.033912978219317776, -0.29726881641499886, 0.82])
        self.trajGoToReady.setInitLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        self.trajGoToReady.setInitRHOrientWP([1.0, 0.0, 0.0])
        self.trajGoToReady.setInitLHOrientWP([1.0, 0.0, 0.0])
        self.trajGoToReady.setInitPostureWP(DEFAULT_POSTURE)
        
        self.trajGoToReady.addRHCartWP([0.019903910090688474, -0.28423307267223147, 0.9179288590591458])
        self.trajGoToReady.addRHCartWP([-0.055152798770261954, -0.2907526623508046, 1.009663652974324])
        self.trajGoToReady.addRHCartWP([-0.03366873622218044, -0.40992725074781894, 1.1144948070701866])
        self.trajGoToReady.addRHCartWP([0.11866831717348489, -0.4101100845056917, 1.209699047600146])
        self.trajGoToReady.addRHCartWP([0.21649227857092893, -0.3006839904787592, 1.1140502834793191])
        self.trajGoToReady.addRHCartWP([0.25822435038901964, -0.1895604971725577, 1.0461857180093073])

        self.trajGoToReady.addRHOrientWP([0.8950968852599132, 0.26432788250814326, 0.3590714922223199])
        self.trajGoToReady.addRHOrientWP([0.8944226954968388, 0.33098423072776184, 0.3007615015086225])
        self.trajGoToReady.addRHOrientWP([0.8994250702615956, 0.22626156457297464, 0.3739521993275524])
        self.trajGoToReady.addRHOrientWP([0.19818667912613866, -0.8161433027447201, 0.5428002851895832])
        self.trajGoToReady.addRHOrientWP([0.260956993686226, -0.8736061290033836, 0.4107478287392042])
        self.trajGoToReady.addRHOrientWP([0.5409881394605172, -0.8191390472602035, 0.19063854336595773])

        self.trajGoToReady.addLHCartWP([0.019903910090688474, 0.28423307267223147, 0.9179288590591458])
        self.trajGoToReady.addLHCartWP([-0.055152798770261954, 0.2907526623508046, 1.009663652974324])
        self.trajGoToReady.addLHCartWP([-0.03366873622218044, 0.40992725074781894, 1.1144948070701866])
        self.trajGoToReady.addLHCartWP([0.11866831717348489, 0.4101100845056917, 1.209699047600146])
        self.trajGoToReady.addLHCartWP([0.21649227857092893, 0.3006839904787592, 1.1140502834793191])
        self.trajGoToReady.addLHCartWP([0.25822435038901964, 0.25,               1.0461857180093073])

        self.trajGoToReady.addLHOrientWP([0.8950968852599132, -0.26432788250814326, 0.3590714922223199])
        self.trajGoToReady.addLHOrientWP([0.8944226954968388, -0.33098423072776184, 0.3007615015086225])
        self.trajGoToReady.addLHOrientWP([0.8994250702615956, -0.22626156457297464, 0.3739521993275524])
        self.trajGoToReady.addLHOrientWP([0.19818667912613866, 0.8161433027447201, 0.5428002851895832])
        self.trajGoToReady.addLHOrientWP([0.260956993686226, 0.8736061290033836, 0.4107478287392042])
        self.trajGoToReady.addLHOrientWP([0.5409881394605172, 0.8191390472602035, 0.19063854336595773])

        self.trajGoToReady.addPostureWP([0.06826499288341317, 0.06826499288341317, 
                       -0.6249282444166423,  0.3079607416653748,  -0.1220981510225299,  1.3675006234559883, 0.06394316468492173, -0.20422693251592328, 0.06223224746326836, 
                       -0.6249282444166423,  0.3079607416653748,  -0.1220981510225299,  1.3675006234559883, 0.06394316468492173, -0.20422693251592328, 0.06223224746326836])
        self.trajGoToReady.addPostureWP([0.0686363596318602,  0.0686363596318602,  
                       -1.0914342991625676,  0.39040871074764566, -0.03720209764435387, 1.7583823306095314, 0.05438773164693069, -0.20257591921666193, 0.06386553930484179, 
                       -1.0914342991625676,  0.39040871074764566, -0.03720209764435387, 1.7583823306095314, 0.05438773164693069, -0.20257591921666193, 0.06386553930484179])
        self.trajGoToReady.addPostureWP([0.06804075180539401, 0.06804075180539401, 
                       -1.3637873691001094,  0.3926057912988488,  0.575755053425441,    1.9732992187122156, 0.29999797251313004, -0.20309827518257023, 0.05586603055643467, 
                       -1.3637873691001094,  0.3926057912988488,  0.575755053425441,    1.9732992187122156, 0.29999797251313004, -0.20309827518257023, 0.05586603055643467])
        self.trajGoToReady.addPostureWP([0.06818415549992426, 0.06818415549992426, 
                       -0.8497599545494692,  0.47079074342878563, 0.8355038507753617,   2.2318590905389852, 1.8475059506175733,  -0.405570582208143,   -0.0277359315904628, 
                       -0.8497599545494692,  0.47079074342878563, 0.8355038507753617,   2.2318590905389852, 1.8475059506175733,  -0.405570582208143,   -0.0277359315904628])
        self.trajGoToReady.addPostureWP([0.06794500584573498, 0.06794500584573498, 
                       -0.24608246913199228, 0.13441397755549533, 0.2542869735593113,   2.0227000417984633, 1.3670468713459782,  -0.45978204939890815, 0.030219082955597457, 
                       -0.24608246913199228, 0.13441397755549533, 0.2542869735593113,   2.0227000417984633, 1.3670468713459782,  -0.45978204939890815, 0.030219082955597457])
        self.trajGoToReady.addPostureWP([0.06796522908004803, 0.06796522908004803,                                                                   # torso
                       -0.08569654146540764, 0.07021124925432169, 0,                    1.7194162945362514, 1.51, -0.07, -0.18,  # left arm
                       -0.08569654146540764, 0.07021124925432169, -0.15649686418494702, 1.7194162945362514, 1.51, -0.07, -0.18])   # right arm

        # ==============================================================================================        
        # Define the GoToIdle trajectory
        self.trajGoToIdle = Trajectory.Trajectory("GoToIdle", TIME_GO_TO_IDLE)
        self.trajGoToIdle.setPrevTraj(self.trajGoToReady)                        # This trajectory always starts where the GoToReady trajectory ends

        # 2015.01.06 Trajectory
        self.trajGoToIdle.addRHCartWP([0.25822435038901964, -0.1895604971725577, 1.0461857180093073])
        self.trajGoToIdle.addRHCartWP([0.21649227857092893, -0.3006839904787592, 1.1140502834793191])
        self.trajGoToIdle.addRHCartWP([0.11866831717348489, -0.4101100845056917, 1.209699047600146])
        self.trajGoToIdle.addRHCartWP([-0.03366873622218044, -0.40992725074781894, 1.1144948070701866])
        self.trajGoToIdle.addRHCartWP([-0.055152798770261954, -0.2907526623508046, 1.009663652974324])
        self.trajGoToIdle.addRHCartWP([0.019903910090688474, -0.28423307267223147, 0.9179288590591458])
        self.trajGoToIdle.addRHCartWP([0.033912978219317776, -0.29726881641499886, 0.82]) # Matches the start of trajectory GoToReady

        self.trajGoToIdle.addRHOrientWP([0.5409881394605172, -0.8191390472602035, 0.19063854336595773])
        self.trajGoToIdle.addRHOrientWP([0.260956993686226, -0.8736061290033836, 0.4107478287392042])
        self.trajGoToIdle.addRHOrientWP([0.19818667912613866, -0.8161433027447201, 0.5428002851895832])
        self.trajGoToIdle.addRHOrientWP([0.8994250702615956, 0.22626156457297464, 0.3739521993275524])
        self.trajGoToIdle.addRHOrientWP([0.8944226954968388, 0.33098423072776184, 0.3007615015086225])
        self.trajGoToIdle.addRHOrientWP([0.8950968852599132, 0.26432788250814326, 0.3590714922223199])
        self.trajGoToIdle.addRHOrientWP([1.0, 0.0, 0.0]) # Matches the start of trajectory GoToReady
        
        self.trajGoToIdle.addLHCartWP([0.25822435038901964, 0.1895604971725577, 1.0461857180093073])
        self.trajGoToIdle.addLHCartWP([0.21649227857092893, 0.3006839904787592, 1.1140502834793191])
        self.trajGoToIdle.addLHCartWP([0.11866831717348489, 0.4101100845056917, 1.209699047600146])
        self.trajGoToIdle.addLHCartWP([-0.03366873622218044, 0.40992725074781894, 1.1144948070701866])
        self.trajGoToIdle.addLHCartWP([-0.055152798770261954, 0.2907526623508046, 1.009663652974324])
        self.trajGoToIdle.addLHCartWP([0.019903910090688474, 0.28423307267223147, 0.9179288590591458])
        self.trajGoToIdle.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82]) # Matches the start of trajectory GoToReady
        
        self.trajGoToIdle.addLHOrientWP([0.5409881394605172, 0.8191390472602035, 0.19063854336595773])
        self.trajGoToIdle.addLHOrientWP([0.260956993686226, 0.8736061290033836, 0.4107478287392042])
        self.trajGoToIdle.addLHOrientWP([0.19818667912613866, 0.8161433027447201, 0.5428002851895832])
        self.trajGoToIdle.addLHOrientWP([0.8994250702615956, -0.22626156457297464, 0.3739521993275524])
        self.trajGoToIdle.addLHOrientWP([0.8944226954968388, -0.33098423072776184, 0.3007615015086225])
        self.trajGoToIdle.addLHOrientWP([0.8950968852599132, -0.26432788250814326, 0.3590714922223199])
        self.trajGoToIdle.addLHOrientWP([1.0, 0.0, 0.0]) # Matches the start of trajectory GoToReady

        self.trajGoToIdle.addPostureWP([0.06796522908004803, 0.06796522908004803,                                                  # torso
                       -0.08569654146540764, 0.07021124925432169,                    0, 1.7194162945362514, 1.51, -0.07, -0.18,    # left arm
                       -0.08569654146540764, 0.07021124925432169, -0.15649686418494702, 1.7194162945362514, 1.51, -0.07, -0.18])   # right arm
        self.trajGoToIdle.addPostureWP([0.06794500584573498, 0.06794500584573498, -0.24608246913199228, 0.13441397755549533, 0.2542869735593113,   2.0227000417984633, 1.3670468713459782,  -0.45978204939890815, 0.030219082955597457, -0.24608246913199228, 0.13441397755549533, 0.2542869735593113,   2.0227000417984633, 1.3670468713459782,  -0.45978204939890815, 0.030219082955597457])
        self.trajGoToIdle.addPostureWP([0.06818415549992426, 0.06818415549992426, -0.8497599545494692,  0.47079074342878563, 0.8355038507753617,   2.2318590905389852, 1.8475059506175733,  -0.405570582208143,   -0.0277359315904628, -0.8497599545494692,  0.47079074342878563, 0.8355038507753617,   2.2318590905389852, 1.8475059506175733,  -0.405570582208143,   -0.0277359315904628])
        self.trajGoToIdle.addPostureWP([0.06804075180539401, 0.06804075180539401, -1.3637873691001094,  0.3926057912988488,  0.575755053425441,    1.9732992187122156, 0.29999797251313004, -0.20309827518257023, 0.05586603055643467, -1.3637873691001094,  0.3926057912988488,  0.575755053425441,    1.9732992187122156, 0.29999797251313004, -0.20309827518257023, 0.05586603055643467])
        self.trajGoToIdle.addPostureWP([0.0686363596318602,  0.0686363596318602,  -1.0914342991625676,  0.39040871074764566, -0.03720209764435387, 1.7583823306095314, 0.05438773164693069, -0.20257591921666193, 0.06386553930484179, -1.0914342991625676,  0.39040871074764566, -0.03720209764435387, 1.7583823306095314, 0.05438773164693069, -0.20257591921666193, 0.06386553930484179])
        self.trajGoToIdle.addPostureWP([0.06826499288341317, 0.06826499288341317, -0.6249282444166423,  0.3079607416653748,  -0.1220981510225299,  1.3675006234559883, 0.06394316468492173, -0.20422693251592328, 0.06223224746326836, -0.6249282444166423,  0.3079607416653748,  -0.1220981510225299,  1.3675006234559883, 0.06394316468492173, -0.20422693251592328, 0.06223224746326836])
        self.trajGoToIdle.addPostureWP(DEFAULT_POSTURE) # Matches the start of trajectory GoToReady

    def createFSM(self):
        # define the states
        moveCartesianState = MoveCartesianState(dreamerInterface = self.dreamerInterface)

        goToReadyState = TrajectoryState(self.dreamerInterface, self.trajGoToReady)
        goToIdleState = TrajectoryState(self.dreamerInterface, self.trajGoToIdle)
        goBackToReadyState = GoBackToReadyState(self.dreamerInterface, self.trajGoToIdle)
        awaitCommandState = AwaitCommandState(moveCartesianState = moveCartesianState, goToIdleState = goToIdleState)
        executeDemoState = ExecuteDemoState(self.dreamerInterface)
        toggleEndEffectorState = EndEffectorToggleState(self.dreamerInterface)

        # wire the states into a FSM
        self.fsm = smach.StateMachine(outcomes=['exit'])
        self.fsm.userdata.endEffectorSide = "right"
        self.fsm.userdata.demoName = "none"

        with self.fsm:

            smach.StateMachine.add("AwaitCommandState", awaitCommandState,
                transitions={"go_to_ready":"GoToReadyState",
                             "go_back_to_ready":"GoBackToReadyState",
                             "move_cartesian_position":"MoveCartesianState",
                             "toggle_end_effector":"ToggleEndEffectorState",
                             "execute_demo":"ExecuteDemoState",
                             "done":"AwaitCommandState",
                             "exit":"exit"},
                remapping={'endEffectorSide':'endEffectorSide', 'demoName':'demoName'})

            smach.StateMachine.add("GoToReadyState", goToReadyState,
                transitions={'done':'AwaitCommandState',
                             'exit':'exit'})
            smach.StateMachine.add("GoToIdleState", goToIdleState,
                transitions={'done':'AwaitCommandState',
                             'exit':'exit'})

            smach.StateMachine.add("GoBackToReadyState", goBackToReadyState,
                transitions={'done':'GoToIdleState',
                             'exit':'exit'})

            smach.StateMachine.add("MoveCartesianState", moveCartesianState,
                transitions={'done':'AwaitCommandState',
                             'exit':'exit'})

            smach.StateMachine.add("ToggleEndEffectorState", toggleEndEffectorState,
                transitions={'done':'AwaitCommandState',
                             'exit':'exit'},
                remapping={'endEffectorSide':'endEffectorSide'})

            smach.StateMachine.add("ExecuteDemoState", executeDemoState,
                transitions={'done':'AwaitCommandState',
                             'exit':'exit'},
                remapping={'demoName':'demoName'})

    def run(self):
        """
        Runs the Cartesian and orientation demo 9 behavior.
        """

        if not self.dreamerInterface.connectToControlIt(DEFAULT_POSTURE):
            return

        self.createTrajectories()
        self.createFSM()

        # Create and start the introspection server
        sis = smach_ros.IntrospectionServer('server_name', self.fsm, '/SM_ROOT')
        sis.start()

        if ENABLE_USER_PROMPTS:
            index = raw_input("Start demo? Y/n\n")
            if index == "N" or index == "n":
                return

        outcome = self.fsm.execute()

        print "Demo 9 done, waiting until ctrl+c is hit..."
        rospy.spin()  # just to prevent this node from exiting
        sis.stop()


# Main method
if __name__ == "__main__":
    rospy.init_node('Demo9_CARL_Telemanipulation', anonymous=True)
    demo = Demo9_CARL_Telemanipulation()
    demo.run()

    print "Demo 9 done, waiting until ctrl+c is hit..."
    rospy.spin()  # just to prevent this node from exiting