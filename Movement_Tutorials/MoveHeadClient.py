#!/usr/bin/python

import rospy, actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

joints = ["head_pan_joint", "head_tilt_joint"]

############################################
## MoveHeadClient.py                      ##
##                                        ##
## Move the head of the Fetch Robot       ##
## using a simple action client           ##
##                                        ##
## This is the preferred method to        ##
## control the head of the Fetch Robot    ##
##                                        ##
## Ian de Vlaming, dvlaming@stanford.edu  ##
## 02/03/2018                             ##
############################################

# moveHead()
# handles head control
def moveHead():
  # create a simple action client headClient
  # which publishes over and subscribes to
  # the parent topic head_controller/follow_joint_trajectory
  # with control_msgs/FollowJointTrajectory Action and Goal
  headClient = actionlib.SimpleActionClient("head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
  # wait for server to respond
  headClient.wait_for_server()
  # create empty trajectory_msgs/JointTrajectory message
  trajectory = JointTrajectory()
  # set name, position, velocity, acceleration, and duration
  # of head trajectory
  trajectory.joint_names = joints
  trajectory.points.append(JointTrajectoryPoint())
  trajectory.points[0].positions = [1.0, 1.0]
  trajectory.points[0].velocities = [0.0] * len(joints)
  trajectory.points[0].accelerations = [0.0] * len(joints)
  trajectory.points[0].time_from_start = rospy.Duration(5.0)
  # create empty control_msgs/GripperCommandGoal message
  goal = FollowJointTrajectoryGoal()
  # set goal trajectory
  goal.trajectory = trajectory
  # set goal time tolerance
  goal.goal_time_tolerance = rospy.Duration(0.0)
  # send the goal to the action server (gripper)
  headClient.send_goal(goal)
  # wait for result to report that the goal is set
  headClient.wait_for_result(rospy.Duration(4.0))

if __name__ == '__main__':
  try:
    # create a ROS node moveHead
    rospy.init_node('moveHead')
    # move the head
    moveHead()
  except rospy.ROSInterruptException:
    pass
