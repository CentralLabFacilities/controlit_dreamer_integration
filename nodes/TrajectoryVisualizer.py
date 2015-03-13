#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import TrajectoryGeneratorCubicSpline

'''
Publishes MarkerArray messages for visualizing trajectories.
'''

class TrajectoryVisualizer:
    def __init__(self, name, trajectories):
        '''
        The constructor.

        Keyword arguments:
          - name: The name of the demo whose trajectories are being visualized.
          - trajectories: An array of Trajectory objects.
        '''

        self.name = name
        self.trajectories = trajectories
        
        # Create the trajectory generators
        self.trajGenerators = []
        for ii in range(0, len(self.trajectories)):
            rhCartTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(self.trajectories[ii].rhCartWP)
            lhCartTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(self.trajectories[ii].lhCartWP)

            rhCartTG.generateTrajectory(self.trajectories[ii].duration)
            lhCartTG.generateTrajectory(self.trajectories[ii].duration)

            self.trajGenerators.append([rhCartTG, lhCartTG])

    def start(self):

        # Create the ROS message publishers
        rhCartTrajPublisher = rospy.Publisher("trajectory/" + self.name + "/RHCartesian", MarkerArray, queue_size=1)
        lhCartTrajPublisher = rospy.Publisher("trajectory/" + self.name + "/LHCartesian", MarkerArray, queue_size=1)

        # Compute the total time of executing all trajectories
        totalTime = 0
        
        for traj in self.trajectories:
            totalTime = totalTime + traj.duration
        
        print "TrajectoryPublisher: total time is {0}".format(totalTime)

        if totalTime == 0:
            print "TrajectoryPublisher: ERROR: Total time is zero! Aborting."
            return

        # Save the start time. This is used to compute the relative time in the
        # subsequent for loop.
        startTime = rospy.Time.now()

        while not rospy.is_shutdown():

            deltaTime = (rospy.Time.now() - startTime).to_sec()

            timeOffset = deltaTime % totalTime
            print "TrajectoryVisualizer: Publishing trajectory for time range [0, {0}]".format(
                timeOffset)

            # Determine which trajectory we should be following
            deltaTime = 0
            trajectoryIndex = -1

            for ii in range(0, len(self.trajectories)):
                if (trajectoryIndex == -1) and (deltaTime + self.trajectories[ii].duration > timeOffset):
                    trajectoryIndex = ii
                else:
                    deltaTime = deltaTime + self.trajectories[ii].duration

            if trajectoryIndex == -1:
                print "TrajectoryPublisher: ERROR: Unable to find current trajectory index for time offset {0}.".format(
                    timeOffset)
                return
            else:
                print "TrajectoryPublisher: Trajectory index: {0}".format(trajectoryIndex)

            # Define the messages
            rightHandMsg = MarkerArray()
            leftHandMsg = MarkerArray()

            prevTrajTime = 0

            wayPointCount = 0

            # Fill messages with trajectories up to but not including the current trajectory
            if trajectoryIndex > 0:
                for ii in range(0, trajectoryIndex):
                    print "TrajectoryPublisher: Adding whole trajectory {0}".format(ii)
                    currentTraj = self.trajectories[ii]
                    currentTG = self.trajGenerators[ii]
                    currentTime = 0

                    prevTrajTime = prevTrajTime + currentTraj.duration

                    while currentTime < currentTraj.duration:

                        pointRH = currentTG[0].getPoint(currentTime)
                        pointLH = currentTG[1].getPoint(currentTime)

                        mmRH = Marker()
                        mmRH.header.frame_id = "world"
                        mmRH.header.stamp = rospy.Time.now()
                        mmRH.ns = "right_hand_cartesian_position"
                        mmRH.id = wayPointCount
                        mmRH.type = Marker.SPHERE
                        mmRH.action = Marker.ADD
        
                        mmRH.pose.position.x = pointRH[0]
                        mmRH.pose.position.y = pointRH[1]
                        mmRH.pose.position.z = pointRH[2]
        
                        mmRH.scale.x = 0.01  # in meters
                        mmRH.scale.y = 0.01
                        mmRH.scale.z = 0.01
        
                        mmRH.color.r = 1.0
                        mmRH.color.g = 1.0
                        mmRH.color.b = 1.0
                        mmRH.color.a = 1.0
        
                        mmRH.lifetime = rospy.Duration(0.2)
        
                        rightHandMsg.markers.append(mmRH)

                        mmLH = Marker()
                        mmLH.header.frame_id = "world"
                        mmLH.header.stamp = rospy.Time.now()
                        mmLH.ns = "left_hand_cartesian_position"
                        mmLH.id = wayPointCount
                        mmLH.type = Marker.SPHERE
                        mmLH.action = Marker.ADD
        
                        mmLH.pose.position.x = pointLH[0]
                        mmLH.pose.position.y = pointLH[1]
                        mmLH.pose.position.z = pointLH[2]
        
                        mmLH.scale.x = 0.01  # in meters
                        mmLH.scale.y = 0.01
                        mmLH.scale.z = 0.01
        
                        mmLH.color.r = 1.0
                        mmLH.color.g = 1.0
                        mmLH.color.b = 1.0
                        mmLH.color.a = 1.0
        
                        mmLH.lifetime = rospy.Duration(0.2)
        
                        leftHandMsg.markers.append(mmLH)

                        currentTime = currentTime + 0.1  # 10Hz
                        wayPointCount = wayPointCount + 1

            # Store the points in the current trajectory
            currentTraj = self.trajectories[trajectoryIndex]
            currentTG = self.trajGenerators[trajectoryIndex]
            currentTime = 0

            while currentTime < timeOffset - prevTrajTime:

                pointRH = currentTG[0].getPoint(currentTime)
                pointLH = currentTG[1].getPoint(currentTime)

                mmRH = Marker()
                mmRH.header.frame_id = "world"
                mmRH.header.stamp = rospy.Time.now()
                mmRH.ns = "right_hand_cartesian_position"
                mmRH.id = wayPointCount
                mmRH.type = Marker.SPHERE
                mmRH.action = Marker.ADD

                mmRH.pose.position.x = pointRH[0]
                mmRH.pose.position.y = pointRH[1]
                mmRH.pose.position.z = pointRH[2]

                mmRH.scale.x = 0.01  # in meters
                mmRH.scale.y = 0.01
                mmRH.scale.z = 0.01

                mmRH.color.r = 1.0
                mmRH.color.g = 1.0
                mmRH.color.b = 1.0
                mmRH.color.a = 1.0

                mmRH.lifetime = rospy.Duration(0.2)

                rightHandMsg.markers.append(mmRH)

                mmLH = Marker()
                mmLH.header.frame_id = "world"
                mmLH.header.stamp = rospy.Time.now()
                mmLH.ns = "left_hand_cartesian_position"
                mmLH.id = wayPointCount
                mmLH.type = Marker.SPHERE
                mmLH.action = Marker.ADD

                mmLH.pose.position.x = pointLH[0]
                mmLH.pose.position.y = pointLH[1]
                mmLH.pose.position.z = pointLH[2]

                mmLH.scale.x = 0.01  # in meters
                mmLH.scale.y = 0.01
                mmLH.scale.z = 0.01

                mmLH.color.r = 1.0
                mmLH.color.g = 1.0
                mmLH.color.b = 1.0
                mmLH.color.a = 1.0

                mmLH.lifetime = rospy.Duration(0.2)

                leftHandMsg.markers.append(mmLH)

                currentTime = currentTime + 0.1  # 10Hz
                wayPointCount = wayPointCount + 1

            rhCartTrajPublisher.publish(rightHandMsg)
            lhCartTrajPublisher.publish(leftHandMsg)


            rospy.sleep(0.1)
