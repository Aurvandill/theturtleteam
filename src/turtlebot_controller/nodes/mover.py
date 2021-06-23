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

#vars for angle correction
min_angle = rospy.get_param("min_angle",0.0)
max_angle = rospy.get_param("max_angle",360.0)
max_correction = rospy.get_param("max_angle_correction",5.0)

#maximum angle difference to start moving
angle_offset = rospy.get_param("angle_offset",20.0)

#tolerance to goal point were we consider it reached
distance_tolerance = rospy.get_param("distance_tolerance",0.05)

#time over which we want to accelerate
acc_time = rospy.get_param("acceleration_time",1.5)

#maximum robot moving speed
max_speed = rospy.get_param("max_speed",1.2)

#important for calculation of linear speed
break_distance = rospy.get_param("break_distance",1.0)
old_time = 0.0
speed = 0.0


#point counter
p_counter = 0



def get_correction(angle):
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

def get_speed(distance, angle_deviation):

    #speed = 0.0
    now = rospy.get_time()
    reached_goal = False

    global break_distance
    global distance_tolerance
    global old_time
    global speed

    #make max speed dependant on angle deviation
    #the larger the deviation the correction speed is to 0
    #the smaller the closer it is to 1
    if angle_deviation > angle_offset:
        angle_deviation = angle_offset
    cor_factor = 1-((1/angle_offset)*abs(angle_deviation))

    #if the correctiuon factor is belov 
    if cor_factor < 0:
        cor_factor = 0

    if angle_deviation != 0:
        angle_correction_speed = max_speed*cor_factor
    else:
        angle_correction_speed = max_speed

    if angle_correction_speed > max_speed:
        angle_correction_speed = max_speed

    #breaking
    if distance < break_distance:
        speed = (angle_correction_speed/break_distance) * distance
    else:
        #default speed + acceleration
        if old_time > 0.0:
            
            speed += angle_correction_speed/acc_time *  (now-old_time)

    ##if angle deviation is too high
    #if abs(angle_deviation) > angle_offset:
    #    speed = 0.0
    #    print "---------angle offset to large"

    #if we reached the goal
    if distance <= distance_tolerance:
        reached_goal = True
        speed = 0.0
        #print "----------imeaditly stopping checkpoint to close"

    #check if our speed is larger than our max speed
    if speed > angle_correction_speed:
        speed = angle_correction_speed

    old_time = now

    #print "---speed: " +str(speed)
    return speed,reached_goal

def callback_marker(msg):

    global p_counter
    #build triangle sides
    global x_pos
    global y_pos
    global yaw
    global angle_offset


    marker_x = msg.points[p_counter].x
    marker_y = msg.points[p_counter].y
    
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
    print "-target point x/y: "+str(marker_x)+"/"+str(marker_y)
    print "-our location x/y: "+str(x_pos)+"/"+str(y_pos)
    print "------------speed: " + str(speed.linear.x)
    print "--target distance: "+str(distance)
    print "-----target_angle: " + str(target_angle)
    print "------orientation: " + str(yaw)
    print "-correction_angle: "+str(target_angle-yaw)
    print "-------------------------------------------"

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

print "mover started"

while not rospy.is_shutdown():
    rospy.spin()