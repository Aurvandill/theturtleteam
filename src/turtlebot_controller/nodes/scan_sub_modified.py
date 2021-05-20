#!/usr/bin/env python

from turtlebot_controller import main


import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Quaternion
from turtlebot_controller.srv import ToggleMovement, GetLinearSpeed, GetTarget, GetMoving
from visualization_msgs.msg import Marker
import math
import random

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
            #print "object ended appended point array to objects"
            #print str(point)
            #print str(iterator)

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

    if length >= 5:
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




def getmoving():
    rospy.wait_for_service('getmoving')
    getspeedproxy = rospy.ServiceProxy('getmoving', GetMoving)
    resp1 = getspeedproxy()

    return resp1.movement_allowed

def gettarget():
    rospy.wait_for_service('gettarget')
    getspeedproxy = rospy.ServiceProxy('gettarget', GetTarget)
    resp1 = getspeedproxy()

    return resp1.target
    

class PRegulator:

    max_speed = rospy.get_param("speed",0.4)

    #static values
    min_angle = rospy.get_param("min_angle",0.0)
    max_angle = rospy.get_param("max_angle",360.0)
    max_correction = rospy.get_param("max_angle_correction",1.0)

    #targeted speed
    speed = Twist()
    #speed.angular.z = 0.1
    speed.linear.x = 0.0

    #marker point relative
    point = Point()
    point.x = 0
    point.y = 0
    point.z = 0

    #marker related vars
    marker = Marker()

    marker.header.frame_id = "/base_scan"
    marker.id = 0
    marker.type = 3
    marker.action = marker.ADD
    marker.scale.x = 5
    marker.scale.y = 5
    marker.scale.z = 5
    marker.color.a = 0.1
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0


    #publishers and listenerns
    vel_topic = rospy.get_param("vel_topicname","/cmd_vel")
    speed_pub = rospy.Publisher(vel_topic, Twist)

    marker_topic = rospy.get_param("marker_topicname","/visualization_marker")
    marker_pub = rospy.Publisher(marker_topic, Marker, queue_size = 100)
    
    def get_angle(self, tuple_distance, shortest):
        tuple_size = len(tuple_distance)
        angle_max = self.max_angle - self.min_angle
        angle_per_point = tuple_size / angle_max

        angle_list = list(tuple_distance)
        angle = angle_list.index(shortest)
        angle = angle*angle_per_point

        return angle, angle_per_point
    
    def correct_angle(self, angle, angle_error):
        if gettarget():
        
            if (angle > self.min_angle+angle_error) and (angle < self.max_angle-angle_error):

                #if our angle is below 180 we steer to the other side (positive z)
                if angle < self.max_angle/2:
                    self.speed.angular.z = 0.0+((self.max_correction/(self.max_angle/2))*angle)

                #if our angle is larger than 180 we steer to the right side (negative z)
                else:
                    self.speed.angular.z = 0.0-((self.max_correction/(self.max_angle/2))*(self.max_angle-angle))

            else:
                if self.speed.angular.z > 0:
                    self.speed.angular.z = self.speed.angular.z -0.01
                else:
                    self.speed.angular.z = 0
        else:
            self.speed.angular.z = 0
                
    def update_lin_speed(self):
        try:
            if getmoving():
                #print getmoving()
                self.speed.linear.x = self.max_speed
            else:
                self.speed.linear.x = 0
        except:
            self.speed.linear.x = 0

    def publish_speed(self):
        #publish z
        self.speed_pub.publish(self.speed)

    def set_speed(self, speed):
        self.speed.linear.x = speed
        #print "speed set to" +str(speed)
        #print "reeeeee"
        

    def cylinder_position(self, angle, distance):

        x_mult = 1
        y_mult = 1

        #we add 2.5meters t othe distance for the diameter(which is 5m) of the pillar

        distance = distance +2.5
        swap = True
        #we wanna swap values when angle >= 180 degree
        if angle >= 270:
            angle = angle - 270
            y_mult = -1
            swap = False
        if angle >= 180:
            angle = angle - 180
            y_mult = -1
            x_mult = -1
            swap = False
        if angle >= 90:
            angle = angle - 90
            x_mult = -1

        #convert to radians so sin and cos give correct results...
        alpha = math.radians(90-angle)

        b = math.cos(alpha) * distance * x_mult #x
        a = math.sin(alpha) * distance * y_mult #y

        if swap:
            self.marker.pose.position.x = a
            self.marker.pose.position.y = b
        else:
            self.marker.pose.position.x = b
            self.marker.pose.position.y = a

        self.marker_pub.publish(self.marker)
        return self.marker.pose.position.x,self.marker.pose.position.y
        

# GLOBAL VARS

#for out dirty hack
max_counter = 10
counter = 0

#p regulator for travelling into the pillar
p_reg = PRegulator()


def laserdistance(tuple_msg):

    global counter
    global max_counter
    global p_reg

    sorted_list = list(tuple_msg)
    sorted_list.sort()

    unsorted_list = list(tuple_msg)

    objects = []
    objects = seperate(unsorted_list)

    angle_offset =0
    for form in objects:
        #print "we got an object"
        #print "object len is: " + str(len(form))
        index = get_index_of_closest(form)
        #cutting array
        #we cut the array to only work with the larger area
        form = cut_array(form, index)
        #print "new length = " + str(len(form))

        #second parameter is angle per point -> should get this from parameter server but its sunday so i dont care that much
        detected = form_detection(form, 1)
        #print "------------\n"
        if detected == "probably a square":
            angle_offset = 270
            #print "yeeeeeeeeeeeet"


    angle, angle_error = p_reg.get_angle(tuple_msg,sorted_list[0])
    p_reg.correct_angle((angle+angle_offset),angle_error)
    p_reg.update_lin_speed()
    p_reg.publish_speed()
    #a,b = p_reg.cylinder_position(angle, sorted_list[0])

    #if(sorted_list[0] < 0.5):
    #    p_reg.set_speed(0)
    if counter > max_counter:
        print "\n----------------------------------"
        print "shortest laserdistance: " + str(sorted_list[0])
        print "amount of laserdata: "+str(len(tuple_msg))
        print "angle: " + str(angle)
        print "angle_error: "+ str(angle_error)
        print "speed linear: "+ str(p_reg.speed.linear.x)
        print "speed angular: "+ str(p_reg.speed.angular.z)
        #print "pillar position:\n\ta: "+str(a)+"\n\tb: "+str(b)
        counter = 0
    else:
        counter += 1
    

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

rospy.init_node("scan_sub")

if int(queue_size) != 1:
    scan_sub = rospy.Subscriber(topic, LaserScan, callback)
else:
    scan_sub = rospy.Subscriber(topic, LaserScan, callback2, int(queue_size))


print "odom subscriber started with " +str(topic) +" as topic and "+str(queue_size)+" as queue size"
main()

while not rospy.is_shutdown():
    rospy.spin()




