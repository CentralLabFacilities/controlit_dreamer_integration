#!/usr/bin/env python

class Trajectory:
    def __init__(self, name, duration):
        '''
        The constructor.
        '''

        self.name = name
        self.duration = duration

        # Waypoints are formatted as: [[x, y, z], ...]
        self.rhCartWP = []     # right hand Cartesian position trajectory
        self.rhOrientWP = []   # right hand orientation trajectory
        self.lhCartWP = []     # left hand Cartesian position trajectory
        self.lhOrientWP = []   # left hand orientation trajectory
        self.jPosWP = []       # posture trajectory

    def setInitRHCartWP(self, wp):
        ''' 
        Sets the first waypoint in the right hand Cartesian space trajectory.
        '''
        self.rhCartWP.insert(0, wp)

    def setInitLHCartWP(self, wp):
        ''' 
        Sets the first waypoint in the left hand Cartesian space trajectory.
        '''
        self.lhCartWP.insert(0, wp)

    def setInitRHOrientWP(self, wp):
        ''' 
        Sets the first waypoint in the right hand orientation space trajectory.
        '''
        self.rhOrientWP.insert(0, wp)

    def setInitLHOrientWP(self, wp):
        ''' 
        Sets the first waypoint in the left hand orientation space trajectory.
        '''
        self.lhOrientWP.insert(0, wp)

    def setInitPostureWP(self, wp):
        ''' 
        Sets the first waypoint in the posture space trajectory.
        '''
        self.jPosWP.insert(0, wp)

    def addRHCartWP(self, wp):
        '''
        Add a waypoint to the right hand Cartesian space trajectory.
        '''
        self.rhCartWP.append(wp)

    def addLHCartWP(self, wp):
        '''
        Add a waypoint to the left hand Cartesian space trajectory.
        '''
        self.lhCartWP.append(wp)

    def addRHOrientWP(self, wp):
        '''
        Add a waypoint to the right hand orientation space trajectory.
        '''
        self.rhOrientWP.append(wp)

    def addLHOrientWP(self, wp):
        '''
        Add a waypoint to the right hand orientation space trajectory.
        '''
        self.lhOrientWP.append(wp)

    def addPostureWP(self, wp):
        '''
        Add a waypoint to the posture space trajectory.
        '''
        self.jPosWP.append(wp)

    def makeRHCartStatic(self, prevTraj):
        ''' 
        Fix the right hand Cartesian space trajectory.
        '''
        self.rhCartWP.append(prevTraj.getFinalRHCartPos())
        self.rhCartWP.append(prevTraj.getFinalRHCartPos())
        self.rhCartWP.append(prevTraj.getFinalRHCartPos())
        self.rhCartWP.append(prevTraj.getFinalRHCartPos())

    def makeRHOrientStatic(self, prevTraj):
        ''' 
        Fix the right hand orientation space trajectory.
        '''
        self.rhOrientWP.append(prevTraj.getFinalRHOrient())
        self.rhOrientWP.append(prevTraj.getFinalRHOrient())
        self.rhOrientWP.append(prevTraj.getFinalRHOrient())
        self.rhOrientWP.append(prevTraj.getFinalRHOrient())

    def makeLHCartStatic(self, prevTraj):
        ''' 
        Fix the left hand Cartesian space trajectory.
        '''
        self.lhCartWP.append(prevTraj.getFinalLHCartPos())
        self.lhCartWP.append(prevTraj.getFinalLHCartPos())
        self.lhCartWP.append(prevTraj.getFinalLHCartPos())
        self.lhCartWP.append(prevTraj.getFinalLHCartPos())

    def makeLHOrientStatic(self, prevTraj):
        ''' 
        Fix the left hand orientation space trajectory.
        '''
        self.lhOrientWP.append(prevTraj.getFinalLHOrient())
        self.lhOrientWP.append(prevTraj.getFinalLHOrient())
        self.lhOrientWP.append(prevTraj.getFinalLHOrient())
        self.lhOrientWP.append(prevTraj.getFinalLHOrient())

    def getFinalRHCartPos(self):
        '''
        Returns the final Cartesian position of the right hand.
        '''
        return self.rhCartWP[len(self.rhCartWP) - 1]

    def getFinalLHCartPos(self):
        '''
        Returns the final Cartesian position of the left hand.
        '''
        return self.lhCartWP[len(self.lhCartWP) - 1]

    def getFinalRHOrient(self):
        '''
        Returns the final orientation of the right hand.
        '''
        return self.rhOrientWP[len(self.rhOrientWP) - 1]

    def getFinalLHOrient(self):
        '''
        Returns the final orientation of the left hand.
        '''
        return self.lhOrientWP[len(self.lhOrientWP) - 1]

    def getFinalPosture(self):
        '''
        Returns the final posture of the robot.
        '''
        return self.jPosWP[len(self.jPosWP) - 1]

    def setPrevTraj(self, traj):
        '''
        Updates this trajectory to start in the same configuration
        as the end of the previous trajectory.
        '''
        self.rhCartWP.insert(0, traj.getFinalRHCartPos())
        self.lhCartWP.insert(0, traj.getFinalLHCartPos())
        self.rhOrientWP.insert(0, traj.getFinalRHOrient())
        self.lhOrientWP.insert(0, traj.getFinalLHOrient())
        self.jPosWP.insert(0, traj.getFinalPosture())