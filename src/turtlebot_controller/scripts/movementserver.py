#!/usr/bin/env python

import rospy
from turtlebot_controller.srv import ToggleMovement, GetLinearSpeed, GetMoving, GetTarget, SetTarget, SetMoving
from geometry_msgs.msg import Twist, Point, Quaternion



max_speed = rospy.get_param("speed",0.4)
speed = 0
move = False
target = False

def callback(req):

    global max_speed
    global speed
    #global moving_allowed

    if req.data:
        speed = max_speed
        #moving_allowed = True
        return True, "set speed to "+str(max_speed)
    else:
        speed = 0
        #moving_allowed = False
        return True, "set speed to 0"


def return_speed(req):
    global speed
    return speed

def return_moving(req):
    global move
    #return True
    return move

def return_target(req):
    global target
    return target

def set_moving(req):

    global move

    if req.data:
        move = True
        return True, "enabled moving"
    else:
        move = False
        return True, "disabled moving"


def set_target(req):

    global target

    if req.data:
        target = True
        return True, "enabled targeting"
    else:
        target = False
        return True, "disabled targeting"


rospy.init_node("speedservice")
s = rospy.Service('togglemovement', ToggleMovement, callback)

speed_s = rospy.Service('getspeed', GetLinearSpeed, return_speed)

moving_getter_s = rospy.Service('getmoving', GetMoving, return_moving)
moving_setter_s = rospy.Service('setmoving', SetMoving, set_moving)
target_getter_s = rospy.Service('gettarget', GetTarget, return_target)
target_setter_s = rospy.Service('settarget', SetTarget, set_target)

rospy.spin()




