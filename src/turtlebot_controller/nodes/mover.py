#!/usr/bin/env python

from turtlebot_controller import main

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from visualization_msgs.msg import Marker, MarkerArray
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
theta = 0

#some defines

min_angle = rospy.get_param("min_angle",0.0)
max_angle = rospy.get_param("max_angle",360.0)
max_correction = rospy.get_param("max_angle_correction",3.0)
angle_offset = rospy.get_param("angle_offset",5.0)
distance_tolerance = rospy.get_param("distance_tolerance",0.2)
distance_deviation = rospy.get_param("distance_deviation",0.05)

acc_time = rospy.get_param("acceleration_time",10.0)
max_speed = rospy.get_param("max_speed",1.0)

#important for calculation of linear speed
started_acc = False
start_time = 0
acc_finished = False
break_distance = rospy.get_param("break_distance",0.2)

#point counter
p_counter = 0



def get_correction(angle):

    print "correction_angle: "+str(angle)
    #angle = abs(angle)
    angular_speed = 0.0
    global max_angle
    global max_correction
    global min_angle

    if angle < max_angle/2:
        angular_speed = 0.0+((max_correction/(max_angle/2))*angle)

        #if our angle is larger than 180 we steer to the right side (negative z)
    else:
        angular_speed = 0.0-((max_correction/(max_angle/2))*(max_angle-angle))

    return angular_speed

def angle_calc(goal_x,goal_y,self_x,self_y):
    
    #angle
    angle = 0.0
    angle = math.degrees(math.atan2(goal_y - self_y, goal_x - self_x))

    #distance
    distance = 0.0
    distance = math.sqrt(pow((goal_x-self_x),2)+pow((goal_y-self_y),2))
    
    return angle,distance

def get_speed(distance, angle_difference):

    global start_time
    global started_acc
    global angle_offset
    global acc_finished
    global max_speed
    global break_distance
    global distance_tolerance
    
    now = rospy.get_time()
    reached_goal = False

    if distance < distance_tolerance:
        reached_goal = True
    
    #set start time
    if not started_acc:
        start_time = now
        started_acc = True

    #stop moving if angle is to large
    if abs(angle_difference+360) > angle_offset+360:
        started_acc = False
        acc_finished = False
        print "---speed: 0"
        return 0, reached_goal

    #acceleration
    if started_acc and not acc_finished:
        time_dif = abs(now - start_time)
        if time_dif < acc_time:
            speed = (max_speed/acc_time) * time_dif
            print "---accspeed: " +str(speed)
            return speed, reached_goal
            

    #distance regulator
    if distance > break_distance:
        distance = break_distance
    if distance < distance_tolerance:
        started_acc = False
        acc_finished = True
    speed = abs((max_speed/break_distance)*distance)
    print "---speed: " +str(speed)

    return speed, reached_goal


def callback_marker(msg):

    global p_counter
    #build triangle sides
    global x_pos
    global y_pos
    global yaw
    global angle_offset
    global theta


    marker_x = msg.points[p_counter].x
    marker_y = msg.points[p_counter].y
    

    print "target point x/y: "+str(marker_x)+"/"+str(marker_y)
    print "our location x/y: "+str(x_pos)+"/"+str(y_pos)

    #calculate angle to point we want to move to
    target_angle, distance = angle_calc(marker_x,marker_y,x_pos,y_pos)

    #create speed so we can let the robot move
    speed = Twist()

    #speed.angular.z = get_correction(target_angle-yaw+angle_correction)
    speed.angular.z = get_correction(target_angle-yaw)
    speed.linear.x, reached_goal = get_speed(distance,target_angle-yaw)
    #speed.linear.x = 0.2
    if reached_goal:
        print "reached checkpoint"
        p_counter += 1
        if p_counter >= len(msg.points):
            p_counter = 0

    #publish speed
    speed_pub.publish(speed)

    print "target distance: "+str(distance)
    print "target_angle: " + str(target_angle)
    print "orientation: " + str(yaw)

def callback_odom(msg):
    
    #vars we will use
    x = msg.pose.pose.orientation.x
    y = msg.pose.pose.orientation.y
    z = msg.pose.pose.orientation.z
    w = msg.pose.pose.orientation.w
    
    #globals
    global roll
    global pitch
    global yaw
    global x_pos
    global y_pos

    x_pos = float(msg.pose.pose.position.x)
    y_pos = float(msg.pose.pose.position.y)

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([x,y,z,w])
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)

    #print "roll,pitch,yaw: "+str(roll)+" "+str(pitch)+" "+str(yaw)
    
rospy.init_node('odom_sub')

odom_topic = rospy.get_param("odom_topicname","/odom")
marker_topic = rospy.get_param("marker_topicname","/marker")

print marker_topic
print odom_topic

odom_sub = rospy.Subscriber(str(odom_topic), Odometry, callback_odom)
marker_sub = rospy.Subscriber(str(marker_topic), Marker, callback_marker)

print "mover subscriber started"

while not rospy.is_shutdown():
    rospy.spin()