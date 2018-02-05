#!/usr/bin/python

import rospy, math
from control_msgs.msg import GripperCommandActionGoal
from actionlib_msgs.msg import GoalStatusArray

gripper_closing = 0.0
gripper_effort = 10.
gripper_flag = False

############################################
## MoveGripper.py                         ##
##                                        ##
## Move the gripper of the Fetch Robot    ##
## using a simple ROS publisher           ##
##                                        ##
## Ian de Vlaming, dvlaming@stanford.edu  ##
## 02/03/2018                             ##
############################################


# gripper()
# handles gripper movement
def gripper():
  # create an empty control_msgs/GripperCommandActionGoal message
  msg = GripperCommandActionGoal()
  # position range is 0-1 (discreet, not sure why the data type is float)
  # 0 commands the gripper to close
  # 1 commands the gripper to open
  msg.goal.command.position = gripper_closing
  # effort indicates the maximum force which should be
  # exerted to open or close the gripper
  msg.goal.command.max_effort = gripper_effort
  # publish the message
  pub.publish(msg)
    
# callback(actionlib_msgs/GoalStatusArray msg)
# subscriber callback function for
# gripper_controller/gripper_action/status topic
def callback(msg):
  # if the goal is acknowledged:
  if msg.status_list[-1].status == 1:
    # set a gripper flag
    set_pos()
    # stop receiving messages over this topic
    sub.unregister()

# set_pos()
# set the global variable gripper_flag
def set_pos():
  global gripper_flag
  gripper_flag = True

if __name__ == '__main__':
  try:
    # initialize ROS node gripper
    rospy.init_node('gripper')
    # create a ROS subscriber:
    # Listen for goal acceptance
    # published as status == 1 in
    # GoalStatusArray messages over the 
    # gripper_controller/gripper_action/status topic
    sub = rospy.Subscriber('/gripper_controller/gripper_action/status', GoalStatusArray, callback)
    # create a ROS publisher:
    # the gripper accepts control_msgs/GripperCommandActionGoal messages
    # over the gripper_controller/gripper_action/goal topic
    # The gripper_controller/gripper_action topic is the parent topic
    # created using an action client - see MoveGripperClient.py
    # for the preferred control method of the gripper
    pub = rospy.Publisher('gripper_controller/gripper_action/goal', GripperCommandActionGoal, queue_size = 10)
    # set the publishing rate to 10Hz
    rate = rospy.Rate(10)
    # the Fetch robot doesn't seem to always respond to
    # the first gripper command, so sending it
    # multiple times until open/closed may be required
    while not gripper_flag:
      gripper()
      rate.sleep()
  except rospy.ROSInterruptException:
    pass
