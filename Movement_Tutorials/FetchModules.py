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
    self.timeStep = 0.0
    self.env, self.robot = fetchpy.initialize(None, None, 'qtcoin')
    self.activeDOFs = self.robot.arm.GetArmIndices()
    self.robot.SetActiveDOFs(self.activeDOFs)
    self.planner = Sequence(TrajoptPlanner(), OMPLPlanner('RRTConnect'))
    self.traj = None
    self.waypoints = None
  def planTrajectory(self, initJointState, finalJointState):
    self.robot.arm.SetDOFValues(initJointState)
    self.traj = self.planner.PlanToConfiguration(self.robot, finalJointState)
    self.waypoints = self.traj.GetAllWaypoints2D()
    for waypoint in self.waypoints:
      self.addJointPosition(np.array(waypoint[:]))
  def addJointPosition(self, point):
    self.trajectory.points.append(JointTrajectoryPoint())
    self.trajectory.points[-1].positions = point[:]
    self.trajectory.points[-1].velocities =  [0.0] * len(self.jointNames)
    self.trajectory.points[-1].accelerations = [0.0] * len(self.jointNames)
    self.timeStep = self.timeStep + 1.0
    self.trajectory.points[-1].time_from_start = rospy.Duration(self.timeStep)
  def moveToGoal(self):
    self.goal.trajectory = self.trajectory
    self.goal.goal_time_tolerance = rospy.Duration(0.0)
    self.client.send_goal(self.goal)
    self.client.wait_for_result(rospy.Duration(self.timeStep + 5.0))

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
