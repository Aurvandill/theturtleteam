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



robot_state_topic = rospy.get_param("robot_state_topicname","/marker")
robot_state_pub = rospy.Publisher(robot_state_topic, Marker, queue_size = 1)
#robot_state_pub = rospy.Publisher(robot_state_topic, MarkerArray, queue_size = 1)

#marker related vars

#markerarray = MarkerArray()
marker = Marker()

linecolor = ColorRGBA()
linecolor.r = 1.0
linecolor.a = 1

marker.header.frame_id = "/odom"
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

p0 = Point()
p0.x = 1.0
p0.y = 1.0
p0.z = 0
marker.points.append(p0)

p1 = Point()
p1.x = 2.0
p1.y = 1.0
p1.z = 0
marker.points.append(p1)

p2 = Point()
p2.x = 2.0
p2.y = 2.0
p2.z = 0
marker.points.append(p2)

p3 = Point()
p3.x = 3.0
p3.y = 2.0
p3.z = 0
marker.points.append(p3)

p4 = Point()
p4.x = 3.0
p4.y = 1.0
p4.z = 0
marker.points.append(p4)

p5 = Point()
p5.x = 1.0
p5.y = 0.0
p5.z = 0
marker.points.append(p5)



#for out dirty hack
max_counter = 20
counter = 0
    

print "robot_test marker pub starte"

rospy.init_node("robot_testmarker_pub")
r = rospy.Rate(50)

main()

while not rospy.is_shutdown():

    robot_state_pub.publish(marker)
    r.sleep()



