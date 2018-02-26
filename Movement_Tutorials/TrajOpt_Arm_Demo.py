#!/usr/bin/python

import fetchpy, rospy, or_trajopt
import numpy as np
from math import pi

def main():
  env, robot = fetchpy.initialize(None, None, 'qtcoin')
  x = robot.arm.GetDOFValues()
  joint_target = np.array([0., 1., 3.14159265, 2., 0., -2., 0.])
  z = robot.arm.GetArmIndices()
  #robot.SetDOFValues(y, z)
  planner = or_trajopt.TrajoptPlanner()
  robot.SetActiveDOFs(z)
  traj = planner.PlanToConfiguration(robot, joint_target)
  waypoints = traj.GetAllWaypoints2D()
  for waypoint in waypoints:
    robot.arm.SetDOFValues(np.array(waypoint))
    rospy.sleep(1.)

if __name__ == '__main__':
  try:
    rospy.init_node('trajopt_test')
    main()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
