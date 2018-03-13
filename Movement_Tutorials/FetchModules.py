#!/usr/bin/python

import fetchpy, rospy, or_trajopt, sys, subprocess, actionlib
from prpy.planning import Sequence, OMPLPlanner
from or_trajopt import TrajoptPlanner
import numpy as np
from math import pi
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class FetchArmObj():
  def __init__(self):
    self.client = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    self.client.wait_for_server()
    self.jointNames = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    self.trajectory = JointTrajectory()
    self.trajectory.joint_names = self.jointNames[:]
    self.goal = FollowJointTrajectoryGoal()
    self.timeStep = 0.5
    self.timeVector = None
    self.env, self.robot = fetchpy.initialize(None, None, 'qtcoin')
    self.activeDOFs = self.robot.arm.GetArmIndices()
    self.robot.SetActiveDOFs(self.activeDOFs)
    self.planner = Sequence(TrajoptPlanner(), OMPLPlanner('RRTConnect'))
    self.traj = None
    self.waypoints = None
    self.waypointVel = None
    self.waypointAcc = None
  def planTrajectory(self, initJointState, finalJointState):
    self.robot.arm.SetDOFValues(initJointState)
    self.traj = self.planner.PlanToConfiguration(self.robot, finalJointState)
    self.waypoints = self.traj.GetAllWaypoints2D()
    self.timeVector = [i*self.timeStep for i in range(len(self.waypoints))]
    self.interpolatePoints()
    for i in range(len(self.waypoints)):
      self.addJointPosition(self.waypoints[i], self.waypointVel[i], self.waypointAcc[i], self.timeVector[i])
  def interpolatePoints(self):
    # form x vector from time step
    x = np.array(self.timeVector)
    # get hermite coefficients
    c = np.polynomial.hermite.hermfit(x, self.waypoints, 2)
    # get hermite first derivate coefficients
    cdot = np.polynomial.hermite.hermder(c)
    # get hermite second derivative coefficients
    cddot = np.polynomial.hermite.hermder(cdot)
    # get velocity targets
    self.waypointVel = np.polynomial.hermite.hermval(self.timeVector, cdot).T
    # get acceleration targets
    self.waypointAcc = np.polynomial.hermite.hermval(self.timeVector, cddot).T
  def addJointPosition(self, point, vel, acc, time):
    self.trajectory.points.append(JointTrajectoryPoint())
    self.trajectory.points[-1].positions = point[:]
    self.trajectory.points[-1].velocities =  vel[:]
    self.trajectory.points[-1].accelerations = acc[:]
    self.trajectory.points[-1].time_from_start = rospy.Duration(time)
  def moveToGoal(self):
    self.goal.trajectory = self.trajectory
    self.goal.goal_time_tolerance = rospy.Duration(0.0)
    self.client.send_goal(self.goal)
    self.client.wait_for_result(rospy.Duration(self.timeVector[-1] + self.timeStep))

class JointObj():
  def __init__(self):
    self.jointNames = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    self.pos = [0.0]*len(self.jointNames)
    self.exit = 0
    self.subscriber = None
  def ReportPos(self, state):
    for joint in state.name:
      if joint in self.jointNames:
        stateIdx = state.name.index(joint)
        idx = self.jointNames.index(joint)
        self.pos[idx] = state.position[stateIdx]
    if (self.pos != [0.0]*len(self.jointNames)):
      self.exit = 1
      self.subscriber.unregister()

def GetJointState():
  try:
    jointState = JointObj()
    rate = rospy.Rate(50)
    jointState.subscriber = rospy.Subscriber('/joint_states', JointState, jointState.ReportPos)
    while not jointState.exit:
      rate.sleep()
    return jointState.pos
  except rospy.ROSInterruptException:
    pass
