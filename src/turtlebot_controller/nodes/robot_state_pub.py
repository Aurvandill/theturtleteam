#!/usr/bin/env python

from turtlebot_controller import main


import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Quaternion
from turtlebot_controller.srv import ToggleMovement, GetLinearSpeed
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry
import math
import random



robot_state_topic = rospy.get_param("robot_state_topicname","/robot_marker")
robot_state_pub = rospy.Publisher(robot_state_topic, Marker, queue_size = 1)
#robot_state_pub = rospy.Publisher(robot_state_topic, MarkerArray, queue_size = 1)

#marker related vars

#markerarray = MarkerArray()
marker = Marker()

linecolor = ColorRGBA()
linecolor.r = 1.0
linecolor.a = 1

marker.header.frame_id = "/map"
marker.id = 0
marker.type = 4
marker.action = marker.ADD
marker.scale.x = 0.1
#marker.scale.y = 1
#marker.scale.z = 1
marker.color.a = 1
marker.color.r = 1.0
marker.pose.orientation.w = 1.0
marker.pose.position.x = 0
marker.pose.position.y = 0
marker.pose.position.z = 0
marker.color = linecolor




#for out dirty hack
max_counter = 20
counter = 0

def callback(msg):
    global marker
    global max_counter
    global counter
    p = Point()


    if counter > max_counter:
        #marker.pose.position.x = msg.pose.pose.position.x
        #marker.pose.position.y = msg.pose.pose.position.y
        p.x = msg.pose.pose.position.x
        p.y = msg.pose.pose.position.y
        p.z = 0
        marker.points.append(p)
    
        counter = 0
    else:
        counter += 1
    

print "robot_state publisher started"

rospy.init_node("robot_state_pub")


odom_topic = rospy.get_param("odom_topicname","/odom")
odom_sub = rospy.Subscriber(str(odom_topic), Odometry, callback)

r = rospy.Rate(10)

main()

while not rospy.is_shutdown():

    robot_state_pub.publish(marker)
    r.sleep()



