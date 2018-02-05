#!/usr/bin/python

from geometry_msgs.msg import Twist, Vector3
import rospy

############################################
## MoveBase.py                            ##
##                                        ##
## Move the base of the Fetch Robot using ##
## a simple ROS publisher                 ##
##                                        ##
## This is the preferred method to        ##
## control the base of the Fetch Robot    ##
##                                        ##
## Ian de Vlaming, dvlaming@stanford.edu  ##
## 02/03/2018                             ##
############################################

# moveBase()
# handles base movement
def moveBase():
  # initialize a ROS node Move_Base
  rospy.init_node('Move_Base')
  # create a ROS publisher:
  # published geometry_msgs/Twist messages
  # over the topic base_controller/command
  pub = rospy.Publisher('base_controller/command', Twist, queue_size = 10)
  # set the publishing rate to 10Hz
  rate = rospy.Rate(10)
  # create an empty geometry_msgs/Twist message
  msg = Twist()
  # create an empty geometry_msgs/Vector3 message
  # for controlling linear velocity
  # the Fetch only responds to the linear velocity
  # in the x direction
  linear = Vector3()
  linear.x = 0.5
  linear.y = 0.0
  linear.z = 0.0
  # create an empty geometry_msgs/Vector3 message
  # for controlling angular velocity
  # the Fetch only responds to the angular velocity
  # in the z direction
  angular = Vector3()
  angular.x = 0.0
  angular.y = 0.0
  angular.z = 1.0
  # set the linear and angular component of the message
  msg.linear = linear
  msg.angular = angular
  # repeatedly publish the message
  # the Fetch robot base defaults to zero linear and angular
  # velocity when it stops receiving messages
  while not rospy.is_shutdown():
    pub.publish(msg)
    rate.sleep()

if __name__ == '__main__':
  try:
    moveBase()
  except rospy.ROSInterruptException:
    pass
    
