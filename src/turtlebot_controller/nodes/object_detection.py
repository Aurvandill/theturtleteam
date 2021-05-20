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

initedlaser_list = False
laser_list = []

def laserdistance(tuple_msg):

    global initedlaser_list
    global laser_list

    if not initedlaser_list:
        laser_list = list(tuple_msg)
        initedlaser_list = True

def seperate(raw_list):

    objects = []
    points = []
    last_point_was_data = False
    last_point = 0
    iterator = 0
    for point in raw_list:
        iterator += 1

        if last_point_was_data:
            points = []
            last_point_was_data = False
            #print "emptying points array"
        
        if str(point) != "inf":
            points.append(point)
            #print "appending point"

        if str(point) == "inf" and str(last_point) != "inf" and iterator > 1:
            objects.append(points)
            last_point_was_data = True
            print "object ended appended point array to objects"
            print str(point)
            print str(iterator)

        last_point = point


    return objects  

def get_index_of_closest(point_list):

    iterator = 0
    buffer = 99999
    index = 0

    for point in point_list:
        if point < buffer:
            buffer = point
            index = iterator


        iterator += 1


    return index

def cut_array(point_list, index):
    if len(point_list)/2 > index:
        return point_list[index:]
    else:
        new_list = point_list[:index+1]
        new_list.reverse()
        return new_list

def triangle_calc(a, beta,c):

    #convert to radians for correct results
    beta = math.radians(beta)

    #calc b
    b = math.sqrt(a**2 + c**2 - 2 * a * c * math.cos(beta))
    #print a
    #print b
    #print c
    #print beta

    gamma = math.acos((a**2 + b**2 - c**2) / (2 * a * b))
    gamma = math.degrees(gamma)

    return gamma


def form_detection(point_list, angle_per_point):

    #get len of third line
    length = len(point_list)

    if length >= 3:
        half = int(length/2)
        a = point_list[0]
        c = point_list[half-1]
        gamma1 = triangle_calc(a,half*angle_per_point,c)
        c2 = point_list[length-1]
        gamma2 = triangle_calc(a,length*angle_per_point,c2)

        #print gamma1
        #print gamma2

        #angle deviation allowed should be gathered from conf server ~
        angle_deviation = 10

        if abs(gamma1-gamma2) > angle_deviation:
            return "probably not a square"
        else:
            return "probably a square"

    return "yeet we dont have enough points"

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

rospy.init_node("object_detection")

if int(queue_size) != 1:
    scan_sub = rospy.Subscriber(topic, LaserScan, callback)
else:
    scan_sub = rospy.Subscriber(topic, LaserScan, callback2, int(queue_size))

main()

executed_once = False

while not rospy.is_shutdown():
    #if not initedlaser_list:
    #    print "laserlist was not inited"
    if initedlaser_list and not executed_once:
        print "inited laserlist"
        objects = []
        objects = seperate(laser_list)
        #print laser_list

        for form in objects:
            print "we got an object"
            print "object len is: " + str(len(form))
            index = get_index_of_closest(form)
            #cutting array
            #we cut the array to only work with the larger area
            form = cut_array(form, index)
            #print "new length = " + str(len(form))

            #second parameter is angle per point -> should get this from parameter server but its sunday so i dont care that much
            print form_detection(form, 1)
            print "------------\n"



        executed_once = True

    if executed_once:
        exit()