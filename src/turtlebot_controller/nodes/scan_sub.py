#!/usr/bin/env python

from turtlebot_controller import main


import rospy
from sensor_msgs.msg import LaserScan
import math


def laserdistance(tuple_msg):
    sorted_list = list(tuple_msg)
    sorted_list.sort()
    print "shortest laserdistance: " + str(sorted_list[0]) + "\n"

def callback(msg):
    print len(msg.ranges)
    #print wut
    #print msg.ranges
    laserdistance(msg.ranges)

def callback2(msg, wut):
    print len(msg.ranges)
    #print wut
    #print msg.ranges
    laserdistance(msg.ranges)



print "scan subscriber started"

topic = rospy.get_param("scan_topicname","/scan")
queue_size = rospy.get_param("queue_size","0")

rospy.init_node("scan_sub")

if int(queue_size) != 1:
    scan_sub = rospy.Subscriber(topic, LaserScan, callback)
else:
    scan_sub = rospy.Subscriber(topic, LaserScan, callback2, int(queue_size))


print "odom subscriber started with " +str(topic) +" as topic and "+str(queue_size)+" as queue size"
main()

while not rospy.is_shutdown():
    rospy.spin()