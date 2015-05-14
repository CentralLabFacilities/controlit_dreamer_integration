#!/usr/bin/env python

'''
Tests 6-DOF end effector control of Dreamer.
'''

import sys, getopt     # for getting and parsing command line arguments
import time
import math
import threading
import rospy
import smach
import smach_ros

from std_msgs.msg import Float64, Float64MultiArray, MultiArrayDimension, Bool, Int32

import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import DreamerInterface
import Trajectory
import TrajectoryGeneratorCubicSpline

ENABLE_USER_PROMPTS = False

# Shoulder abductors about 10 degrees away from body and elbows bent 90 degrees
# DEFAULT_POSTURE = [0.0, 0.0,                                    # torso
#                    0.0, 0.174532925, 0.0, 1.57, 0.0, 0.0, 0.0,  # left arm
#                    0.0, 0.174532925, 0.0, 1.57, 0.0, 0.0, 0.0]  # right arm

# Shoulder abductors and elbows at about 10 degrees
DEFAULT_POSTURE = [0.0, 0.0,                                    # torso
                   0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
                   0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0]  # right arm

# The time each trajectory should take
TRAJECTORY_GO_TO_READY_EXECUTION_TIME = 10.0
TRAJECTORY_GO_TO_IDLE_EXECUTION_TIME = 10.0

class TrajectoryState(smach.State):
    def __init__(self, dreamerInterface, traj):
        smach.State.__init__(self, outcomes=["done", "exit"])
        self.dreamerInterface = dreamerInterface
        self.traj = traj

    def execute(self, userdata):
        rospy.loginfo('Executing TrajectoryState')

        if self.dreamerInterface.followTrajectory(self.traj):
            return "done"
        else:
            return "exit"

class UserInputState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["done", "exit"])

    def execute(self, userdata):
        rospy.loginfo('Executing UserInputState')

        index = raw_input("Type any key to continue or 'q' to quit\n")
        if index == "q" or index == "Q":
            return "exit"
        else:
            return "done"

class ExitState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["done", "exit"])

    def execute(self, userdata):
        rospy.loginfo('Executing ExitState')

        index = raw_input("Repeat? (Y/n)\n")
        if index == "N" or index == "n":
            return "exit"
        else:
            return "done"

# class EnablePowerGraspState(smach.State):
#     def __init__(self, dreamerInterface):
#         smach.State.__init__(self, outcomes=["done", "exit"])
#         self.dreamerInterface = dreamerInterface

#     def execute(self, userdata):
#         rospy.loginfo('Executing EnablePowerGraspState')
        
#         self.dreamerInterface.rightHandCmdMsg.data = True
#         self.dreamerInterface.rightHandCmdPublisher.publish(self.dreamerInterface.rightHandCmdMsg)
        
#         rospy.sleep(5) # allow fingers to move

#         if rospy.is_shutdown():
#             return "exit"
#         else:
#             return "done"

# class DisablePowerGraspState(smach.State):
#     def __init__(self, dreamerInterface):
#         smach.State.__init__(self, outcomes=["done", "exit"])
#         self.dreamerInterface = dreamerInterface

#     def execute(self, userdata):
#         rospy.loginfo('Executing DisablePowerGraspState')
        
#         self.dreamerInterface.rightHandCmdMsg.data = False
#         self.dreamerInterface.rightHandCmdPublisher.publish(self.dreamerInterface.rightHandCmdMsg)
        
#         rospy.sleep(5) # allow fingers to move

#         if rospy.is_shutdown():
#             return "exit"
#         else:
#             return "done"

class SleepState(smach.State):
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

class Demo12_Test6DOFTraj:
    def __init__(self):
        self.dreamerInterface = DreamerInterface.DreamerInterface(enableUserPrompts = ENABLE_USER_PROMPTS, useQuaternionControl = True)

    def createFSM(self):

        # ==============================================================================================
        # Define the GoToReady trajectory
        trajGoToReady = Trajectory.Trajectory("TrajGoToReady", TRAJECTORY_GO_TO_READY_EXECUTION_TIME)

        # These are the initial values as specified in the YAML ControlIt! configuration file
        trajGoToReady.setInitRHCartWP([0.033912978219317776, -0.29726881641499886, 0.82])
        trajGoToReady.setInitLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        trajGoToReady.setInitRHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajGoToReady.setInitLHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajGoToReady.setInitPostureWP(DEFAULT_POSTURE)
        
        trajGoToReady.addRHCartWP([0.03770122239874958, -0.2198480697094594, 0.8117255494770094])
        trajGoToReady.addRHCartWP([-0.040412081820136, -0.26397896734315346, 0.8614132694842028])
        trajGoToReady.addRHCartWP([-0.08745294968918928, -0.4212025201185584, 0.9700075682372389])
        trajGoToReady.addRHCartWP([-0.11703885123141751, -0.5441284402599831, 1.074238114641072])
        trajGoToReady.addRHCartWP([-0.028571575782775867, -0.5331226739243937, 1.1077348538402065])
        trajGoToReady.addRHCartWP([0.10443412373794271, -0.43466008783175114, 1.0813821843583091])
        trajGoToReady.addRHCartWP([0.18110137021381892, -0.2940613557332989, 1.0518617028889898])
        trajGoToReady.addRHCartWP([0.2113219018061716, -0.2643487426168306, 1.0374345284989988])

        trajGoToReady.addRHOrientWP([0.015314958442464818, 0.9742020083686487, -0.03310730940157143, 0.2227101367306515])
        trajGoToReady.addRHOrientWP([0.06315100311214952, 0.963529935854198, 0.0732094285307895, 0.24952433366894244])
        trajGoToReady.addRHOrientWP([0.2237161097085336, 0.9227471407807557, 0.22962971144883815, 0.2139135621173723])
        trajGoToReady.addRHOrientWP([0.39813383612342007, 0.8861239319509271, 0.141108980196644, 0.19068844082781058])
        trajGoToReady.addRHOrientWP([0.28435745155745307, 0.7917325092960383, -0.22346213905397447, 0.49230594743205314])
        trajGoToReady.addRHOrientWP([-0.15757388336647615, 0.5786146703417355, -0.45588708135956824, 0.6576796359897327])
        trajGoToReady.addRHOrientWP([-0.428763505205464, 0.5658049764829423, -0.4585747035651799, 0.5345426329500469])
        trajGoToReady.addRHOrientWP([0.5154210756453367, -0.5011947096853305, 0.5117860403981165, -0.47034033063055375])

        # left arm does not move
        trajGoToReady.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        trajGoToReady.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        trajGoToReady.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        trajGoToReady.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        trajGoToReady.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        trajGoToReady.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        trajGoToReady.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])

        trajGoToReady.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajGoToReady.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajGoToReady.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajGoToReady.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajGoToReady.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajGoToReady.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajGoToReady.addLHOrientWP([0.0, 1.0, 0.0, 0.0])


        trajGoToReady.addPostureWP([-0.03275314504282583, -0.03275314504282583, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.2703826362458247, -0.0065433902706428905, 0.041972401461917844, 0.6110144337952284, 0.019912862716942595, 0.044322813664107175, 0.02367545574623616])
        trajGoToReady.addPostureWP([-0.032668738760182066, -0.032668738760182066, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.6433949416194826, 0.09891907709380923, 0.02617480781444035, 1.042555339864615, -0.09837493173922425, 0.042368789771995014, 0.020423394398665785])
        trajGoToReady.addPostureWP([-0.03291797737590012, -0.03291797737590012, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -1.027979301155709, 0.42664946889046196, 0.26838727684045416, 1.3431449354282685, -0.18598081367704852, 0.045863959573267304, 0.011991375307989063])
        trajGoToReady.addPostureWP([-0.033166757392176754, -0.033166757392176754, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -1.2063541697039177, 0.5105390055758586, 0.7413537899427806, 1.3121828016124317, -0.04628156004509675, 0.07479736649384078, -0.023032732690594346])
        trajGoToReady.addPostureWP([-0.03330523512928211, -0.03330523512928211, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -1.0632108817150043, 0.5069731198534251, 0.8282099405168639, 1.5441810293971316, 0.7963049502680233, 0.03203551846031133, 0.014448711039086787])
        trajGoToReady.addPostureWP([-0.033705424911517234, -0.033705424911517234, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.48228016411548497, 0.21224044191487212, 0.669719319246093, 1.7273118869841055, 1.2982157441184288, -0.1522229498804123, -0.08341418135999394])
        trajGoToReady.addPostureWP([-0.033652912808301884, -0.033652912808301884, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.3473204748468209, 0.10776519554599014, 0.19223728101830065, 1.7387782499406619, 1.4955383045612167, -0.15524780645352, -0.06426266303552737])
        trajGoToReady.addPostureWP([-0.033391175597322645, -0.033391175597322645, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.23757389535525622, 0.04591719434787172, 0.1278410467643636, 1.6277323745869778, 1.6772115700161672, -0.16424177713962002, -0.07271150445497984])

        # ==============================================================================================
        # Define the GoToIdle trajectory
        trajGoToIdle = Trajectory.Trajectory("TrajGoToIdle", TRAJECTORY_GO_TO_IDLE_EXECUTION_TIME)

        # Set the previous trajectory to be TrajGoToReady
        trajGoToIdle.setPrevTraj(trajGoToReady)
        
        trajGoToIdle.addRHCartWP([0.2113219018061716, -0.2643487426168306, 1.0374345284989988])
        trajGoToIdle.addRHCartWP([0.18110137021381892, -0.2940613557332989, 1.0518617028889898])
        trajGoToIdle.addRHCartWP([0.10443412373794271, -0.43466008783175114, 1.0813821843583091])
        trajGoToIdle.addRHCartWP([-0.028571575782775867, -0.5331226739243937, 1.1077348538402065])
        trajGoToIdle.addRHCartWP([-0.11703885123141751, -0.5441284402599831, 1.074238114641072])
        trajGoToIdle.addRHCartWP([-0.08745294968918928, -0.4212025201185584, 0.9700075682372389])
        trajGoToIdle.addRHCartWP([-0.040412081820136, -0.26397896734315346, 0.8614132694842028])
        trajGoToIdle.addRHCartWP([0.03770122239874958, -0.2198480697094594, 0.8117255494770094])
        trajGoToIdle.addRHCartWP([0.033912978219317776, -0.29726881641499886, 0.82])

        trajGoToIdle.addRHOrientWP([0.5154210756453367, -0.5011947096853305, 0.5117860403981165, -0.47034033063055375])
        trajGoToIdle.addRHOrientWP([-0.428763505205464, 0.5658049764829423, -0.4585747035651799, 0.5345426329500469])
        trajGoToIdle.addRHOrientWP([-0.15757388336647615, 0.5786146703417355, -0.45588708135956824, 0.6576796359897327])
        trajGoToIdle.addRHOrientWP([0.28435745155745307, 0.7917325092960383, -0.22346213905397447, 0.49230594743205314])
        trajGoToIdle.addRHOrientWP([0.39813383612342007, 0.8861239319509271, 0.141108980196644, 0.19068844082781058])
        trajGoToIdle.addRHOrientWP([0.2237161097085336, 0.9227471407807557, 0.22962971144883815, 0.2139135621173723])
        trajGoToIdle.addRHOrientWP([0.06315100311214952, 0.963529935854198, 0.0732094285307895, 0.24952433366894244])
        trajGoToIdle.addRHOrientWP([0.015314958442464818, 0.9742020083686487, -0.03310730940157143, 0.2227101367306515])
        trajGoToIdle.addRHOrientWP([0.0, 1.0, 0.0, 0.0])

        # left arm does not move
        trajGoToIdle.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        trajGoToIdle.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        trajGoToIdle.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        trajGoToIdle.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        trajGoToIdle.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        trajGoToIdle.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        trajGoToIdle.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        trajGoToIdle.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])

        trajGoToIdle.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajGoToIdle.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajGoToIdle.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajGoToIdle.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajGoToIdle.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajGoToIdle.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajGoToIdle.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajGoToIdle.addLHOrientWP([0.0, 1.0, 0.0, 0.0])

        trajGoToIdle.addPostureWP([-0.033391175597322645, -0.033391175597322645, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.23757389535525622, 0.04591719434787172, 0.1278410467643636, 1.6277323745869778, 1.6772115700161672, -0.16424177713962002, -0.07271150445497984])
        trajGoToIdle.addPostureWP([-0.033652912808301884, -0.033652912808301884, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.3473204748468209, 0.10776519554599014, 0.19223728101830065, 1.7387782499406619, 1.4955383045612167, -0.15524780645352, -0.06426266303552737])
        trajGoToIdle.addPostureWP([-0.033705424911517234, -0.033705424911517234, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.48228016411548497, 0.21224044191487212, 0.669719319246093, 1.7273118869841055, 1.2982157441184288, -0.1522229498804123, -0.08341418135999394])
        trajGoToIdle.addPostureWP([-0.03330523512928211, -0.03330523512928211, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -1.0632108817150043, 0.5069731198534251, 0.8282099405168639, 1.5441810293971316, 0.7963049502680233, 0.03203551846031133, 0.014448711039086787])
        trajGoToIdle.addPostureWP([-0.033166757392176754, -0.033166757392176754, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -1.2063541697039177, 0.5105390055758586, 0.7413537899427806, 1.3121828016124317, -0.04628156004509675, 0.07479736649384078, -0.023032732690594346])
        trajGoToIdle.addPostureWP([-0.03291797737590012, -0.03291797737590012, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -1.027979301155709, 0.42664946889046196, 0.26838727684045416, 1.3431449354282685, -0.18598081367704852, 0.045863959573267304, 0.011991375307989063])
        trajGoToIdle.addPostureWP([-0.032668738760182066, -0.032668738760182066, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.6433949416194826, 0.09891907709380923, 0.02617480781444035, 1.042555339864615, -0.09837493173922425, 0.042368789771995014, 0.020423394398665785])
        trajGoToIdle.addPostureWP([-0.03275314504282583, -0.03275314504282583, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.2703826362458247, -0.0065433902706428905, 0.041972401461917844, 0.6110144337952284, 0.019912862716942595, 0.044322813664107175, 0.02367545574623616])

        
        trajGoToIdle.setInitPostureWP(DEFAULT_POSTURE)

        # ==============================================================================================
        # define the states
        stateGoToReady = TrajectoryState(self.dreamerInterface, trajGoToReady)
        stateGoToIdle = TrajectoryState(self.dreamerInterface, trajGoToIdle)
        stateUserInput = UserInputState()
        stateExit = ExitState()

        # ==============================================================================================
        # wire the states into a FSM
        self.fsm = smach.StateMachine(outcomes=['exit'])
        with self.fsm:
            smach.StateMachine.add("GoToReadyState", stateGoToReady,
                transitions={'done':'UserInputState',
                             'exit':'exit'})
            smach.StateMachine.add("GoToIdleState", stateGoToIdle,
                transitions={'done':'ExitState',
                             'exit':'exit'})
            smach.StateMachine.add("UserInputState", stateUserInput,
                transitions={'done':'GoToIdleState',
                             'exit':'exit'})
            smach.StateMachine.add("ExitState", stateExit,
                transitions={'done':'GoToReadyState',
                             'exit':'exit'})

    def run(self):
        """
        Runs demo 12 behavior.
        """

        if not self.dreamerInterface.connectToControlIt(DEFAULT_POSTURE):
            return

        self.createFSM()

        # Create and start the introspection server
        sis = smach_ros.IntrospectionServer('server_name', self.fsm, '/SM_ROOT')
        sis.start()

        index = raw_input("Start demo? Y/n\n")
        if index == "N" or index == "n":
            return

        outcome = self.fsm.execute()

        print "Demo 12 done, waiting until ctrl+c is hit..."
        rospy.spin()  # just to prevent this node from exiting
        sis.stop()


# Main method
if __name__ == "__main__":
    rospy.init_node('Demo12_Test_6DOF_Traj', anonymous=True)
    demo = Demo12_Test6DOFTraj()
    demo.run()

    
