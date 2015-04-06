#!/usr/bin/env python

'''
This uses both posture and orientation control.
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

import DreamerInterface
import Trajectory
import TrajectoryGeneratorCubicSpline

ENABLE_USER_PROMPTS = False
GRASP_TIME = 8

# Shoulder abductors about 10 degrees away from body and elbows bent 90 degrees
# DEFAULT_POSTURE = [0.0, 0.0,                                    # torso
#                    0.0, 0.174532925, 0.0, 1.57, 0.0, 0.0, 0.0,  # left arm
#                    0.0, 0.174532925, 0.0, 1.57, 0.0, 0.0, 0.0]  # right arm

# Shoulder abductors and elbows at about 10 degrees
DEFAULT_POSTURE = [0.0, 0.0,                                    # torso
                   0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
                   0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0]  # right arm

class Demo5_HandShake:
    def __init__(self):
        self.dreamerInterface = DreamerInterface.DreamerInterface(ENABLE_USER_PROMPTS)

    def createTrajectories(self):

        # ==============================================================================================
        # Define the GoToReady trajectory
        self.trajGoToReady = Trajectory.Trajectory("GoToReady", 5.0)

        # These are the initial values as specified in the YAML ControlIt! configuration file
        self.trajGoToReady.setInitRHCartWP([0.033912978219317776, -0.29726881641499886, 0.82])
        self.trajGoToReady.setInitLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        self.trajGoToReady.setInitRHOrientWP([1.0, 0.0, 0.0])
        self.trajGoToReady.setInitLHOrientWP([1.0, 0.0, 0.0])
        self.trajGoToReady.setInitPostureWP(DEFAULT_POSTURE)
        
        self.trajGoToReady.addRHCartWP([0.17932609171050676, -0.1947116107098258, 0.8387314141057434])
        self.trajGoToReady.addRHCartWP([0.2761888140937166, -0.2014111570190916, 0.934860166427957])
        # self.trajGoToReady.addRHCartWP([0.3568508489208336, -0.2522742841735632, 1.0477901706353308])
        self.trajGoToReady.addRHCartWP([0.30, -0.30, 1.10])

        self.trajGoToReady.addRHOrientWP([0.8389283544759069, -0.12533099736713216, 0.5296143475731253])
        self.trajGoToReady.addRHOrientWP([0.2235723628903799, -0.7215894666621694, 0.6552282351622359])
        self.trajGoToReady.addRHOrientWP([-0.030540625256204334, -0.9939692971780734, 0.1053200193519126])

        self.trajGoToReady.addLHCartWP([0.02965705887441486, 0.23088636680912317, 0.810922355509038])
        self.trajGoToReady.addLHCartWP([0.029689571433364353, 0.23083475171780563, 0.8109351922278043])
        self.trajGoToReady.addLHCartWP([0.02985679760506356, 0.23060205142957346, 0.8108808654985685])

        self.trajGoToReady.addLHOrientWP([0.9974231896120277, 0.03885099712962302, 0.06031236064193728])
        self.trajGoToReady.addLHOrientWP([0.9973855628117416, 0.039156806343972415, 0.06073535717887724])
        self.trajGoToReady.addLHOrientWP([0.9973534260815909, 0.03959962399135038, 0.06097551363503542])

        self.trajGoToReady.addPostureWP([0.022154761602009465, 0.022154761602009465, -0.24128221587505694, 0.03695440579936058, -0.025257242521253893, 0.6140504454635949, 0.0779998778955152, -0.2683878329384452, 0.0006097929281019635, 0.027855020993482953, -0.041381215034669044, -0.005952573692931204, 0.6552616241979254, 0.10812051456834937, -0.07093969869957849, -0.09962853201157851])
        self.trajGoToReady.addPostureWP([0.021791076348124116, 0.021791076348124116, -0.24126772903690724, 0.03678783103820191, -0.025068049370090353, 0.614121885152926, 0.07793206088242675, -0.2685578796206052, 0.0008613719366874122, 0.08441237407874143, -0.030654446867288323, -0.009171947872870517, 1.053685612334746, 0.7875534805163547, 0.11839301075003532, -0.32005664846914467])
        self.trajGoToReady.addPostureWP([0.02261344933362008, 0.02261344933362008, -0.24046242047415964, 0.03612594842600267, -0.024443836417768035, 0.6141064688418445, 0.07759789575168982, -0.26756988346271293, 0.0018279130452489236, 0.27597656277256155, 0.012763093450392634, 0.11650789924507622, 1.2552150356689133, 1.4437630086869198, -0.07478599278803119, -0.12877855214133435])

        # ==============================================================================================
        self.trajGrasp = Trajectory.Trajectory("Grasp", GRASP_TIME)
        self.trajGrasp.setPrevTraj(self.trajGoToReady)

        self.trajGrasp.addRHCartWP([0.30, -0.30, 1.10])
        self.trajGrasp.addRHCartWP([0.60, -0.20, 1.20])
        self.trajGrasp.addRHCartWP([0.60, -0.20, 1.00])

        # the following do not change
        self.trajGrasp.addRHOrientWP([-0.030540625256204334, -0.9939692971780734, 0.1053200193519126])
        self.trajGrasp.addRHOrientWP([-0.030540625256204334, -0.9939692971780734, 0.1053200193519126])
        self.trajGrasp.addRHOrientWP([-0.030540625256204334, -0.9939692971780734, 0.1053200193519126])
        
        self.trajGrasp.addLHCartWP([0.02985679760506356, 0.23060205142957346, 0.8108808654985685])
        self.trajGrasp.addLHCartWP([0.02985679760506356, 0.23060205142957346, 0.8108808654985685])
        self.trajGrasp.addLHCartWP([0.02985679760506356, 0.23060205142957346, 0.8108808654985685])

        self.trajGrasp.addLHOrientWP([0.9973534260815909, 0.03959962399135038, 0.06097551363503542])
        self.trajGrasp.addLHOrientWP([0.9973534260815909, 0.03959962399135038, 0.06097551363503542])
        self.trajGrasp.addLHOrientWP([0.9973534260815909, 0.03959962399135038, 0.06097551363503542])
        
        self.trajGrasp.addPostureWP([0.02261344933362008, 0.02261344933362008, -0.24046242047415964, 0.03612594842600267, -0.024443836417768035, 0.6141064688418445, 0.07759789575168982, -0.26756988346271293, 0.0018279130452489236, 0.27597656277256155, 0.012763093450392634, 0.11650789924507622, 1.2552150356689133, 1.4437630086869198, -0.07478599278803119, -0.12877855214133435])
        self.trajGrasp.addPostureWP([0.02261344933362008, 0.02261344933362008, -0.24046242047415964, 0.03612594842600267, -0.024443836417768035, 0.6141064688418445, 0.07759789575168982, -0.26756988346271293, 0.0018279130452489236, 0.27597656277256155, 0.012763093450392634, 0.11650789924507622, 1.2552150356689133, 1.4437630086869198, -0.07478599278803119, -0.12877855214133435])
        self.trajGrasp.addPostureWP([0.02261344933362008, 0.02261344933362008, -0.24046242047415964, 0.03612594842600267, -0.024443836417768035, 0.6141064688418445, 0.07759789575168982, -0.26756988346271293, 0.0018279130452489236, 0.27597656277256155, 0.012763093450392634, 0.11650789924507622, 1.2552150356689133, 1.4437630086869198, -0.07478599278803119, -0.12877855214133435])

        # ==============================================================================================
        self.trajReverseGrasp = Trajectory.Trajectory("ReverseGrasp", 5.0)
        self.trajReverseGrasp.setPrevTraj(self.trajGrasp)

        self.trajReverseGrasp.addRHCartWP([0.60, -0.20, 1.00])
        self.trajReverseGrasp.addRHCartWP([0.60, -0.20, 1.20])
        self.trajReverseGrasp.addRHCartWP([0.30, -0.30, 1.10])

        # the following do not change
        self.trajReverseGrasp.addRHOrientWP([-0.030540625256204334, -0.9939692971780734, 0.1053200193519126])
        self.trajReverseGrasp.addRHOrientWP([-0.030540625256204334, -0.9939692971780734, 0.1053200193519126])
        self.trajReverseGrasp.addRHOrientWP([-0.030540625256204334, -0.9939692971780734, 0.1053200193519126])
        
        self.trajReverseGrasp.addLHCartWP([0.02985679760506356, 0.23060205142957346, 0.8108808654985685])
        self.trajReverseGrasp.addLHCartWP([0.02985679760506356, 0.23060205142957346, 0.8108808654985685])
        self.trajReverseGrasp.addLHCartWP([0.02985679760506356, 0.23060205142957346, 0.8108808654985685])

        self.trajReverseGrasp.addLHOrientWP([0.9973534260815909, 0.03959962399135038, 0.06097551363503542])
        self.trajReverseGrasp.addLHOrientWP([0.9973534260815909, 0.03959962399135038, 0.06097551363503542])
        self.trajReverseGrasp.addLHOrientWP([0.9973534260815909, 0.03959962399135038, 0.06097551363503542])
        
        self.trajReverseGrasp.addPostureWP([0.02261344933362008, 0.02261344933362008, -0.24046242047415964, 0.03612594842600267, -0.024443836417768035, 0.6141064688418445, 0.07759789575168982, -0.26756988346271293, 0.0018279130452489236, 0.27597656277256155, 0.012763093450392634, 0.11650789924507622, 1.2552150356689133, 1.4437630086869198, -0.07478599278803119, -0.12877855214133435])
        self.trajReverseGrasp.addPostureWP([0.02261344933362008, 0.02261344933362008, -0.24046242047415964, 0.03612594842600267, -0.024443836417768035, 0.6141064688418445, 0.07759789575168982, -0.26756988346271293, 0.0018279130452489236, 0.27597656277256155, 0.012763093450392634, 0.11650789924507622, 1.2552150356689133, 1.4437630086869198, -0.07478599278803119, -0.12877855214133435])
        self.trajReverseGrasp.addPostureWP([0.02261344933362008, 0.02261344933362008, -0.24046242047415964, 0.03612594842600267, -0.024443836417768035, 0.6141064688418445, 0.07759789575168982, -0.26756988346271293, 0.0018279130452489236, 0.27597656277256155, 0.012763093450392634, 0.11650789924507622, 1.2552150356689133, 1.4437630086869198, -0.07478599278803119, -0.12877855214133435])

        # ==============================================================================================        
        self.trajGoToIdle = Trajectory.Trajectory("GoToIdle", 5.0)
        self.trajGoToIdle.setPrevTraj(self.trajReverseGrasp)

        self.trajGoToIdle.addRHCartWP([0.30, -0.30, 1.10])
        # self.trajGoToIdle.addRHCartWP([0.3568508489208336, -0.2522742841735632, 1.0477901706353308])
        self.trajGoToIdle.addRHCartWP([0.2761888140937166, -0.2014111570190916, 0.934860166427957])
        self.trajGoToIdle.addRHCartWP([0.17932609171050676, -0.1947116107098258, 0.8387314141057434])

        self.trajGoToIdle.addRHOrientWP([-0.030540625256204334, -0.9939692971780734, 0.1053200193519126])
        self.trajGoToIdle.addRHOrientWP([0.2235723628903799, -0.7215894666621694, 0.6552282351622359])
        self.trajGoToIdle.addRHOrientWP([0.8389283544759069, -0.12533099736713216, 0.5296143475731253])

        self.trajGoToIdle.addLHCartWP([0.02985679760506356, 0.23060205142957346, 0.8108808654985685])
        self.trajGoToIdle.addLHCartWP([0.029689571433364353, 0.23083475171780563, 0.8109351922278043])
        self.trajGoToIdle.addLHCartWP([0.02965705887441486, 0.23088636680912317, 0.810922355509038])

        self.trajGoToIdle.addLHOrientWP([0.9973534260815909, 0.03959962399135038, 0.06097551363503542])
        self.trajGoToIdle.addLHOrientWP([0.9973855628117416, 0.039156806343972415, 0.06073535717887724])
        self.trajGoToIdle.addLHOrientWP([0.9974231896120277, 0.03885099712962302, 0.06031236064193728])

        self.trajGoToIdle.addPostureWP([0.02261344933362008, 0.02261344933362008, -0.24046242047415964, 0.03612594842600267, -0.024443836417768035, 0.6141064688418445, 0.07759789575168982, -0.26756988346271293, 0.0018279130452489236, 0.27597656277256155, 0.012763093450392634, 0.11650789924507622, 1.2552150356689133, 1.4437630086869198, -0.07478599278803119, -0.12877855214133435])
        self.trajGoToIdle.addPostureWP([0.021791076348124116, 0.021791076348124116, -0.24126772903690724, 0.03678783103820191, -0.025068049370090353, 0.614121885152926, 0.07793206088242675, -0.2685578796206052, 0.0008613719366874122, 0.08441237407874143, -0.030654446867288323, -0.009171947872870517, 1.053685612334746, 0.7875534805163547, 0.11839301075003532, -0.32005664846914467])
        self.trajGoToIdle.addPostureWP([0.022154761602009465, 0.022154761602009465, -0.24128221587505694, 0.03695440579936058, -0.025257242521253893, 0.6140504454635949, 0.0779998778955152, -0.2683878329384452, 0.0006097929281019635, 0.027855020993482953, -0.041381215034669044, -0.005952573692931204, 0.6552616241979254, 0.10812051456834937, -0.07093969869957849, -0.09962853201157851])

    def run(self):
        """
        Runs the Cartesian and orientation demo 6 behavior.
        """

        if not self.dreamerInterface.connectToControlIt(DEFAULT_POSTURE):
            return

        self.createTrajectories()

        response = raw_input("Start demo? Y/n\n")
        if response == "N" or response == "n":
            return

        #=============================================================================
        if not self.dreamerInterface.followTrajectory(self.trajGoToReady):
            return

        #=============================================================================
        done = False
        while not done:

            response = raw_input("Type any key to grasp item...\n")

            if not self.dreamerInterface.followTrajectory(self.trajGrasp):
                return

            response = raw_input("Type any key to move right hand back to original location...\n")
            
            if not self.dreamerInterface.followTrajectory(self.trajReverseGrasp):
                return

            response = raw_input("Grasp again? Y/n\n")
            if response == "N" or response == "n":
                done = True
                self.trajGrasp.setPrevTraj(self.trajReverseGrasp)

        #=============================================================================
        if not self.dreamerInterface.followTrajectory(self.trajGoToIdle):
            return

# Main method
if __name__ == "__main__":

    rospy.init_node('Demo5_HandShake', anonymous=True)

    demo = Demo5_HandShake()
    # t = threading.Thread(target=demo.run)
    # t.start()
    demo.run()

    print "Demo 4 done, waiting until ctrl+c is hit..."
    rospy.spin()  # just to prevent this node from exiting
