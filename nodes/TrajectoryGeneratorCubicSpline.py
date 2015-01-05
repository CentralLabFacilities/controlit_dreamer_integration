#!/usr/bin/env python

import math
import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class TrajectoryGeneratorCubicSpline:
    def __init__(self, waypoints):
        '''
        layout of waypoints: 
            [[x1, y1, z1], [x2, y2, z2], ...]

        layout of self.waypoints:
            [[x1, x2, ...], [y1, y2, ...], [z1, x2, ...]]
        '''

        self.numDim = len(waypoints[0])  # number of dimensions per waypoint
        self.numWaypoints = len(waypoints) # number of waypoints

        self.waypoints = []
        for ii in range(self.numDim):
            self.waypoints.append([])

        for ii in range(self.numDim):
            for jj in range(self.numWaypoints):
                self.waypoints[ii].append(waypoints[jj][ii])

        # print "self.waypoints = {0}".format(self.waypoints)


    def generateTrajectory(self, totalTime):
        '''
        Generates a trajectory based on the waypoints provided.

        totalTime - The number of seconds the entire trajectory should last.
        '''
        # print "generating trajectory, total time is {0}...".format(totalTime)

        self.totalTime = totalTime

        # Define the time axis
        tt = np.linspace(0, totalTime, self.numWaypoints)  # start time, end time, number of values

        # Define the interpolation function for each dimension
        interpFunctions = []
        for ii in range(self.numDim):
            ff = interp1d(tt, self.waypoints[ii], kind='cubic')
            interpFunctions.append(ff)

        # Define the interpolated time axis with millisecond resolution
        ttHighDef = np.linspace(0, totalTime, totalTime * 1000)

        # Generate the interpolations at millisecond resolution
        self.trajectories = []
        for ii in range(self.numDim):
            traj = interpFunctions[ii](ttHighDef)
            self.trajectories.append(traj)

    def getPoint(self, deltaTime):
        # print "getting point at deltaTime {0}".format(deltaTime)

        result = []

        for ii in range(self.numDim):
            result.append(self.linearInterpolate(self.trajectories[ii], deltaTime))

        return result

    def getLastPoint(self):
        '''
        Returns the last point in the trajectory.
        '''
        result = []

        for ii in range(self.numDim):
            result.append(self.trajectories[ii][len(self.trajectories[ii]) - 1])

        return result        

    def linearInterpolate(self, trajectory, deltaTime):
        """
        Computes the position along the trajectory at the specified deltaTime.
        The trajectory is assumed to have millisecond resolution.
        deltaTime is assumed to be in seconds.
        """
        lowerIndex = math.floor(deltaTime * 1000)
        if lowerIndex >= len(trajectory):
            print "linearInterpolate: WARNING: deltaTime is beyond end of trajectory!\n"\
                  "  - length of trajectory: {0}\n"\
                  "  - delta time: {1}\n"\
                  "Returning last position in trajectory.".format(len(trajectory), deltaTime)
            return trajectory[len(trajectory) - 1]
        elif lowerIndex == len(trajectory) - 1:
            return trajectory[len(trajectory) - 1]
        else:
            beforePoint = trajectory[lowerIndex]
            afterPoint = trajectory[lowerIndex + 1]

            # compute fraction of milliseconds that have elapsed
            fractionElapsed = (deltaTime * 1000) - math.floor(deltaTime * 1000)

            # do linear interpolation
            return (afterPoint - beforePoint) * fractionElapsed + beforePoint

# Main method for testing and debugging purposes
if __name__ == "__main__":

    # Note waypoints are formatted as: [[x, y, z], ...]
    waypoints = []
    waypoints.append([0.034156182237965314, -0.2536961667775097, 0.7942923566248334])
    waypoints.append([-0.03852115301282585, -0.36702885542756375, 1.0044042662878492])
    waypoints.append([-0.0275400056213187, -0.4346278435022028, 1.109258357008881])
    waypoints.append([0.16786527968278075, -0.48763818929105546, 1.2849643133074693])
    waypoints.append([0.2561600552895205, -0.36355117909588747, 1.2737345840311838])

    print waypoints

    tg = TrajectoryGeneratorCubicSpline(waypoints)
    tg.generateTrajectory(15)
    wp10 = tg.getPoint(10)
    print "Waypoint at time 10 is: {0}".format(wp10)