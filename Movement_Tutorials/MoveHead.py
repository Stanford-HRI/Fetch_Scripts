#!/usr/bin/python

import sys, rospy, actionlib, math
from control_msgs.msg import PointHeadActionGoal
from actionlib_msgs.msg import GoalStatusArray
from sensor_msgs.msg import JointState

head_flag = False
goalPoint = [-1.0, -1.0, -1.0]

############################################
## MoveHead.py                            ##
##                                        ##
## Move the head of the Fetch Robot       ##
## using a simple ROS publisher           ##
##                                        ##
## Ian de Vlaming, dvlaming@stanford.edu  ##
## 02/03/2018                             ##
############################################

# aimHead()
# handle moving the head
def aimHead():
  # create an empty control_msgs/PointHeadActionGoal message
  msg = PointHeadActionGoal()
  # not really necessary, but you can set
  # the time stamp if you feel like it
  msg.header.stamp = rospy.Time.now()
  # specify the robot frame
  msg.header.frame_id = "base_link"
  # setting time stamps for fun
  msg.goal.target.header.stamp = rospy.Time.now()
  # deja vu
  msg.goal.target.header.frame_id = "base_link"
  # aim the head along a trajectory using
  # x, y, z coordinates (not the same as the
  # bellows, head pan, and head tilt joint
  # angles)
  msg.goal.target.point.x = goalPoint[0]
  msg.goal.target.point.y = goalPoint[1]
  msg.goal.target.point.z = goalPoint[2]
  # set the desired arrival time. Must be
  # of the type duration
  msg.goal.min_duration = rospy.Duration(1.8)
  # publish the goal
  pub.publish(msg)

# callback(actionlib_msgs/GoalStatusArray msg)
# respond to messages published over the
# /head_controller/point_head/status topic
def callback(msg):
  # if the goal has been acknowledged
  if msg.status_list[-1].status == 1:
    # set the head_flag
    set_pos()
    # stop listening to the /head_controller/point_head/status topic
    sub.unregister()

# set_pos()
# set the global head_flag
def set_pos():
  global head_flag
  head_flag = True
  
if __name__ == '__main__':
  try:
    # create ROS node Move_Head
    rospy.init_node('Move_Head')
    # set publish rate to 10Hz
    rate = rospy.Rate(10)
    # create a ROS publisher to control the head
    # by sending aontrol_msgs/PointHeadActionGoal messages
    # over the head_controller/point_head/goal topic
    pub = rospy.Publisher('/head_controller/point_head/goal', PointHeadActionGoal, queue_size=10)
    # create a ROS subscriber to detect acknowledgement
    # of goals by checking status == 1 in 
    # actionlib_msgs/GoalStatusArray messages
    # published over the /head_controller/point_head/status
    # topic with callback as the response function
    sub = rospy.Subscriber('/head_controller/point_head/status', GoalStatusArray, callback)
    # publish until flag is raised
    while not head_flag:
      aimHead()
      rate.sleep()
  except rospy.ROSInterruptException:
    pass
