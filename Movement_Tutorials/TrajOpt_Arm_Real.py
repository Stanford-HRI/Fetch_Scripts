#!/usr/bin/python

# Run the following in a command line terminal before attempting to run this script:
# export ROS_MASTER_URI=http://192.168.1.10

fetchHome = [1.5748163485321045, 1.4966412754675293, 3.0627143893334963, -1.7174540554089355, 0.0007014063332796104, -1.5383961232360839, 0.0007150011326789857]

import fetchpy, rospy, or_trajopt, sys, subprocess, actionlib
import numpy as np
from math import pi
from FetchModules import GetJointState, FetchArmObj
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

if __name__ == '__main__':
  try:
      rospy.init_node('TrajOpt_Demo')
      initJointState = GetJointState()
      arm = FetchArmObj()
      finalJointState = initJointState[:]
      finalJointState[2] = -0.3
      #finalJointState = np.array([0.0]*len(arm.jointNames))
      arm.planTrajectory(initJointState, finalJointState)
      arm.moveToGoal()
      #rospy.spin()
  except rospy.ROSInterruptException:
    pass

''' 
rospy.init_node('TrajOpt_Demo')
initJointState = GetJointState()
arm = FetchArmObj()
finalJointState = initJointState[:]
finalJointState[2] = -2.0
#finalJointState = np.array([0.0]*len(arm.jointNames))
arm.planTrajectory(initJointState, finalJointState)
'''
