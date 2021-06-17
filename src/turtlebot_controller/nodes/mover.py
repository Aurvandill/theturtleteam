#!/usr/bin/env python

from turtlebot_controller import main

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
import math
import tf


x_val = 0.0
y_val = 0.0

x_desired = 0.0
y_desired = 0.0

#for out dirty hack
max_counter = 50
counter = 0

#bool to see if the first vel command came
vel = False

#targeted speed
speed = Twist()

vel_topic = rospy.get_param("vel_topicname","/cmd_vel")
speed_pub = rospy.Publisher(vel_topic, Twist)

#orientation values
roll = 0
pitch = 0
yaw = 0

#position values
x_pos = 0
y_pos = 0


def callback_marker(msg):
    print "received marker message"

    marker_x = 10
    marker_y = 5

    #build triangle sides
    global x_pos
    global y_pos

    x_dif = x_pos - marker_x
    y_dif = y_pos - marker_y


    #calculate 


def callback_odom(msg):
    x = msg.pose.pose.orientation.x
    y = msg.pose.pose.orientation.y
    z = msg.pose.pose.orientation.z
    w = msg.pose.pose.orientation.w
    global roll
    global pitch
    global yaw

    global x_pos
    global y_pos

    x_pos = float(msg.pose.pose.position.x)
    y_pos = float(msg.pose.pose.position.x)

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([x,y,z,w])
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)
    print "roll,pitch,yaw: "+str(roll)+" "+str(pitch)+" "+str(yaw)
    


rospy.init_node('odom_sub')

odom_topic = rospy.get_param("odom_topicname","/odom")
marker_topic = rospy.get_param("marker_topicname","/marker")

print marker_topic
print odom_topic

odom_sub = rospy.Subscriber(str(odom_topic), Odometry, callback_odom)
marker_sub = rospy.Subscriber(str(marker_topic), Twist, callback_marker)

print "mover subscriber started"

while not rospy.is_shutdown():
    rospy.spin()