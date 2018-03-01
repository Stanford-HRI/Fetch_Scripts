#!/usr/bin/python

import fetchpy, rospy, or_trajopt, sys, subprocess
import numpy as np
from math import pi

def main(initJointState, rate):
  # Initialize fetchpy
  env, robot = fetchpy.initialize(None, None, 'qtcoin')
  # Set the arm as the active portion of the robot
  activeDOFs = robot.arm.GetArmIndices()
  robot.SetActiveDOFs(activeDOFs)
  # initialize fetchpy robot joint state
  robot.arm.SetDOFValues(initJointState)
  # Set joint target
  joint_target = np.array([0., 0., 0, 0., 0., 0., 0.])
  # Set the TrajOpt planner
  planner = or_trajopt.TrajoptPlanner()
  # Plan motion
  traj = planner.PlanToConfiguration(robot, joint_target)
  waypoints = traj.GetAllWaypoints2D()
  for waypoint in waypoints:
    robot.arm.SetDOFValues(np.array(waypoint))
    rate.sleep()

def parseCharList(charList):
  # define acceptable characters
  isNumeric = ['+', '-', '.', 'e', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9']
  # initialize variable
  numArray = np.array([])
  lastChar = 0
  # loop through chars
  for char in charList:
    # if char is numeric and the last char wasn't
    if (char in isNumeric) and (lastChar == 0):
      # initialize string with char
      newString = char
      # set last char to numeric
      lastChar = 1
    # else if char is numeric and last char was
    elif (char in isNumeric) and (lastChar == 1):
      # append char to string
      newString = newString + char
    # else if char is not numeric and last char was
    elif (char not in isNumeric) and (lastChar == 1):
      # convert string to float
      newFloat = float(newString)
      # append float to array
      numArray = np.append(numArray, newFloat)
      # set last char to not numeric
      lastChar = 0
  # return array
  return numArray
  
if __name__ == '__main__':
  try:
      initJointState = subprocess.check_output('../Modules/GetJointState.py')
      initJointState = parseCharList(initJointState[:])
      #initJointState = np.array([0.]*(len(sys.argv) - 1))
      #for i in range(1, len(sys.argv)):
      #  initJointState[i - 1] = float(sys.argv[i])
      rospy.init_node('trajopt_demo')
      rate = rospy.Rate(1)
      main(initJointState, rate)
      rospy.spin()
  except rospy.ROSInterruptException:
    pass
    
    #[1.3025348210128784, 1.4138060780187989, -0.2131015744116211, 1.6465658153491212, 0.016041214406174422, 1.6430804697814942, 0.008001410048794746]
