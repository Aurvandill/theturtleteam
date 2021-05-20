#!/usr/bin/env python

from turtlebot_controller import main

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
import math


x_val = 0.0
y_val = 0.0
#we need the old values because we want to calc travled distance as triangle
x_old = 0.0
y_old = 0.0

#to store the diff between actual value and old value
x_dif = 0.0
y_dif = 0.0

#var to store total distance in
total_distance = 0.0

#to check if we are doing our first run
firstrun = True

#for out dirty hack
max_counter = 50
counter = 0

#bool to see if the first vel command came
vel = False

def callback_vel(msg):

    global vel
    if not vel:
        speed_x = msg.linear.x
        speed_y = msg.linear.y

        if not speed_x == 0 or not speed_y == 0:
            vel = True


def callback(msg):
    #to use global vars in here
    global x_val
    global y_val
    global x_old
    global y_old
    global x_dif
    global y_dif
    global total_distance
    global firstrun
    global max_counter
    global counter
    global vel

    #get current vlaue from odometry
    x_val = float(msg.pose.pose.position.x)
    y_val = float(msg.pose.pose.position.x)
    
    if vel:
        if not firstrun:
            x_dif = abs(x_val-x_old)
            y_dif = abs(y_val-y_old)
    
    x_old = x_val
    y_old = y_val

    firstrun = False

    if x_val > 0.0 and y_val > 0.0 and vel:
        try:
            total_distance += math.sqrt(pow(x_dif,2)+pow(y_dif,2))
        except:
            print "an error occured -> couldnt calc travel"
    else:
        if vel:
            total_distance += x_dif+y_dif

    formated_travel = "{:.2f}".format(total_distance)
    
    #print vel
    #to print not every message to a a dirty hack here
    if counter > max_counter and vel:
        print "\nTotal Travel Distance: "+str(formated_travel)+"m\n"
        counter = 0
    else:
        counter += 1


rospy.init_node('odom_sub')

odom_topic = rospy.get_param("odom_topicname","/odom")
vel_topic = rospy.get_param("vel_topicname","/cmd_vel")

print vel_topic
print odom_topic

odom_sub = rospy.Subscriber(str(odom_topic), Odometry, callback)
vel_sub = rospy.Subscriber(str(vel_topic), Twist, callback_vel)

print "odom subscriber started"
main()

while not rospy.is_shutdown():
    rospy.spin()