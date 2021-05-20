#!/usr/bin/env python

from turtlebot_controller import main


import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Quaternion
from turtlebot_controller.srv import ToggleMovement, GetLinearSpeed, SetMoving
from visualization_msgs.msg import Marker
import math
import random



max_distance = rospy.get_param("brake_distance", 0.1)
state = True

def laserdistance(tuple_msg):

    global max_distance
    global state

    sorted_list = list(tuple_msg)
    sorted_list.sort()

    if sorted_list[0] < max_distance:
        state = False
        rospy.wait_for_service('setmoving')
        getspeedproxy = rospy.ServiceProxy('setmoving', SetMoving)
        resp1 = getspeedproxy(state,)

    

def callback(msg):
    #print len(msg.ranges)
    #print wut
    #print msg.ranges
    laserdistance(msg.ranges)
    #print msg

def callback2(msg, wut):
    #print len(msg.ranges)
    #print wut
    #print msg.ranges
    laserdistance(msg.ranges)
    #print msg


print "scan subscriber started"

topic = rospy.get_param("scan_topicname","/scan")
queue_size = rospy.get_param("queue_size","0")

rospy.init_node("emergency_break")

if int(queue_size) != 1:
    scan_sub = rospy.Subscriber(topic, LaserScan, callback)
else:
    scan_sub = rospy.Subscriber(topic, LaserScan, callback2, int(queue_size))


print "odom subscriber started with " +str(topic) +" as topic and "+str(queue_size)+" as queue size"
main()

while not rospy.is_shutdown():
    rospy.spin()