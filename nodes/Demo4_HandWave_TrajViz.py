#!/usr/bin/env python

'''
Publishes the Cartesian trajectories used in Demo4.
'''

import Demo4_HandWave
import TrajectoryVisualizer
import rospy

rospy.init_node('Demo4_HandWave_TrajViz', anonymous=True)

demo = Demo4_HandWave.Demo4_HandWave()
trajPub = TrajectoryVisualizer.TrajectoryVisualizer(name="Demo4_HandWave", trajectories=demo.getTrajectories())
trajPub.start()
