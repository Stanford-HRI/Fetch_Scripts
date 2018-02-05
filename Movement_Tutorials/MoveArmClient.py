#!/usr/bin/python

import rospy, actionlib
from math import pi
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

joint_names = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

############################################
## MoveArmClient.py                       ##
##                                        ##
## Move the arm of the Fetch Robot        ##
## using a simple action client           ##
##                                        ##
## This is the preferred method to        ##
## control the arm of the Fetch Robot     ##
##                                        ##
## Ian de Vlaming, dvlaming@stanford.edu  ##
## 02/03/2018                             ##
############################################

# initPos(sensor_msgs/JointState msg)
# respond to messages published over the
# /joint_states topic to initialize
# the arm trajectory 
def initPos(state):
  # immediately unsubscribe to the
  # /joint_states topic
  sub.unregister()
  # initialize the position vector
  initPos = [0.0]*len(joint_names)
  # loop over all robot joints
  for joint in state.name:
    # if the joint is in the arm
    if joint in joint_names:
      # add its initial value to the position vector
      stateIdx = state.name.index(joint)
      idx = joint_names.index(joint)
      initPos[idx] = state.position[stateIdx]
  # move the arm
  moveArm(initPos[:])

# moveArm(float64[] initPos)
# handles control of the arm
def moveArm(initPos):
  # create a simple action client armClient
  # which publishes over and subscribes to
  # to parent topic arm_with_torso_controller/follow_joint_trajectory
  # with control_msgs/FollowJointTrajectory
  # Action and Goal messages
  armClient = actionlib.SimpleActionClient('arm_with_torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
  # wait for the server to respond
  armClient.wait_for_server()
  # create an empty trajectory_msgs/JointTrajectory message
  trajectory = JointTrajectory()
  # set the name, position, velocity, acceleration,
  # and duration of the trajectory points
  # (mostly just playing around with positions so that
  # the robot doesn't mutilate itself)
  trajectory.joint_names = joint_names
  trajectory.points.append(JointTrajectoryPoint())
  trajectory.points[0].positions = initPos[:]
  
  trajectory.points[0].positions[3] = pi
  trajectory.points[0].positions[0] = 0.5
  
  trajectory.points[0].velocities =  [0.0] * len(joint_names)
  trajectory.points[0].accelerations = [0.0] * len(joint_names)
  trajectory.points[0].time_from_start = rospy.Duration(0.0)
  
  pos_array = [1, 4, 2, 3, 6, 5, 7]
  ind = 0
  for i in pos_array:
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[-1].positions = trajectory.points[-2].positions[:]
    trajectory.points[-1].positions[i] = 0.0
    trajectory.points[-1].velocities =  [0.0] * len(joint_names)
    trajectory.points[-1].accelerations = [0.0] * len(joint_names)
    trajectory.points[-1].time_from_start = rospy.Duration((ind + 1)*4.0)
    ind = ind + 1
  
  # create empty control_msgs/FollowJointTrajectoryGoal message
  goal = FollowJointTrajectoryGoal()
  # set goal trajectory
  goal.trajectory = trajectory
  # set goal time tolerance
  goal.goal_time_tolerance = rospy.Duration(0.0)
  # send the goal to the server
  armClient.send_goal(goal)
  # wait for acknowledgement of goal
  armClient.wait_for_result(rospy.Duration((len(joint_names) + 2)*2.0))

if __name__ == '__main__':
  try:
    # create a ROS node moveArm
    rospy.init_node('moveArm')
    # create a subscriber to listen for sensor_msgs/JointState
    # messages over the /joint_states topic with initPos as
    # the message handler
    sub = rospy.Subscriber('/joint_states', JointState, initPos)
    # action isn't taken until an initial state is read,
    # so we have to hold the node open with rospy.spin()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
