#!/usr/bin/env python

from turtlebot_controller import main


import rospy
from sensor_msgs.msg import Imu
from turtlebot_controller.srv import ToggleMovement, GetLinearSpeed, SetMoving
from visualization_msgs.msg import Marker
import math
import random



max_dec = rospy.get_param("max_dec", -0.1)
state = True
    

def callback(msg):
    global max_dec
    global state

    #print msg.linear_acceleration

    deceleration = abs(msg.linear_acceleration.x) + abs(msg.linear_acceleration.y)

    if deceleration > max_dec:
        state = False
        rospy.wait_for_service('setmoving')
        getspeedproxy = rospy.ServiceProxy('setmoving', SetMoving)
        resp1 = getspeedproxy(state,)
        print "crash detected - stopping"


topic = rospy.get_param("crash_topicname","/imu")

rospy.init_node("crash_detect")

imu_sub = rospy.Subscriber(topic, Imu, callback)

main()

while not rospy.is_shutdown():
    rospy.spin()