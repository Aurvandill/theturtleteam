#!/usr/bin/env python2

from turtlebot_controller import main

import rospy
from sensor_msgs.msg import LaserScan
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
max_correction = rospy.get_param("max_angle_correction",2.5)

#maximum angle difference to start moving
angle_offset = rospy.get_param("angle_offset",15.0)

#tolerance to goal point were we consider it reached
distance_tolerance = rospy.get_param("distance_tolerance",0.02)

#time over which we want to accelerate
acc_time = rospy.get_param("acceleration_time",2.5)

#maximum robot moving speed
max_speed = rospy.get_param("max_speed",1.0)

#important for calculation of linear speed
break_distance = rospy.get_param("break_distance",0.3)
old_time = 0.0
speed = 0.0

#variable to see if we reached the last checkpoint
finished = False

#variable to store laser data array in
laser_vals = list()

#how far after the half we want to evaluate laser data for exit location
laser_angle_offset = rospy.get_param("laser_angle_offset",160)

#var how many free points are considered an exit
min_free_spots = rospy.get_param("min_free_spots",20)

#var to store if we calculated our exit point
calculated_exit = False
#final x and y coords
final_x = 0
final_y = 0

reached_finish = False

#point counter for our checkpoints
p_counter = 0

def get_final_point(laser_list):
    global x_pos
    global y_pos
    global laser_angle_offset
    global min_free_spots
    global yaw
    global final_x
    global final_y

    end_x = 0
    end_y = 0

    new_list = list()
    
    print("laenge lsite laserdaten: "+str(len(laser_list)))
    i = len(laser_list) - laser_angle_offset
    while i < len(laser_list):
        new_list.append(laser_list[i])
        i += 1

    i = 0
    while i < laser_angle_offset:
        new_list.append(laser_list[i])
        i += 1
    
    print ("laenge lsite laserdaten verarbeitet: "+str(len(new_list)))

    i = 0
    last_val = "9999"
    start_of_hole = 9999
    end_of_hole = 9999
    while i < len(new_list):
        print(last_val)
        print (i)
        print (str(new_list[i]))
        print ("--------------------")
        if last_val == "inf" and start_of_hole == 9999:
            start_of_hole = i-1
        
        if str(new_list[i]) != "inf" and last_val =="inf" and start_of_hole != 9999:
            end_of_hole = i-1

        last_val = str(new_list[i])
        i += 1

    #angle calculation
    print ("we found "+str(abs(start_of_hole-end_of_hole))+" consecutive free spots in laser data")
    print ("start of hole: "+str(start_of_hole))
    print ("end of hole: "+str(end_of_hole))

    if abs(start_of_hole-end_of_hole) > min_free_spots:
        print ("hole is large enough")

        #get middle of hole
        middle = start_of_hole + (abs(end_of_hole-start_of_hole)/2)

        #get angle
        #we assume one point is 1 degree
        angle = middle - laser_angle_offset + yaw

        print ("middle of hole: " + str(middle))
        print ("angle relative to robot: " + str(middle - laser_angle_offset))
        print ("world angle: " + str(angle))
        #calculate new point

        final_x = x_pos + (0.75 * math.cos(math.radians(angle)))
        final_y = y_pos + (0.75 * math.sin(math.radians(angle))) 

        print ("finaler punkt x/y:" +str(final_x)+"/"+str(final_y))

    else:
        print ("hole is not large enough to be classified as exit")

def callback_laser(msg):
    global laser_vals
    laser_vals = list(msg.ranges)

def get_correction(angle):
    #angle = abs(angle)
    angular_speed = 0.0
    global max_angle
    global yaw
    global max_correction
    global min_angle

    #print angle
    if angle < 0:

        if abs(angle) < max_angle/2:
            #lefthandturn
            angular_speed = 0.0-((max_correction/(max_angle/2))*abs(angle))

        #if our angle is larger than 180 we steer to the right side (negative z)
        else:
            #righthandturn
            angular_speed = 0.0+((max_correction/(max_angle/2))*(max_angle-abs(angle)))
    else:
        if abs(angle) < max_angle/2:
            #righthandturn
            angular_speed = 0.0+((max_correction/(max_angle/2))*abs(angle))

        #if our angle is larger than 180 we steer to the right side (negative z)
        else:
            #lefthandturn
            angular_speed = 0.0-((max_correction/(max_angle/2))*(max_angle-abs(angle)))

    #angular_speed = 1*(math.radians(angle)-math.radians(yaw))
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
    global max_angle

    if angle_deviation > max_angle/2:
        angle_deviation = abs(max_angle-angle_deviation)

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
        angle_correction_speed = 0

    if angle_correction_speed > max_speed:
        angle_correction_speed = max_speed
    
    
    #stuff
    break_speed = 9999
    acc_speed = 9999

    #breaking
    if distance < break_distance:
        break_speed = (angle_correction_speed/break_distance) * distance
    else:
        #default speed + acceleration
        if old_time > 0.0:
            
            acc_speed = speed+ angle_correction_speed/acc_time *  (now-old_time)

    if break_speed > acc_speed:
        speed = acc_speed
        print ("using acc speed")
    else:
        speed = break_speed
        print ("using break speed")

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
    global finished
    global laser_vals
    global reached_finish

    if not finished:
        marker_x = msg.points[p_counter].x
        marker_y = msg.points[p_counter].y
    else:
        
        marker_x = final_x
        marker_y = final_y

    
    #calculate angle to point we want to move to
    target_angle, distance = angle_calc(marker_x,marker_y,x_pos,y_pos)

    #create speed so we can let the robot move
    speed = Twist()

    #speed.angular.z = get_correction(target_angle-yaw+angle_correction)
    if not reached_finish:
        speed.angular.z = get_correction(target_angle-yaw)
        #speed.angular.z = get_correction(target_angle)
        speed.linear.x, reached_goal = get_speed(distance,abs(target_angle-yaw))
    else:
        speed.angular.z = 0
        speed.linear.x, reached_goal = 0, False
    #speed.linear.x = 0.2
    if reached_goal:
        print ("reached checkpoint")
        p_counter += 1
        if finished:
            print ("finished moving to target")
            reached_finish = True
        if p_counter >= len(msg.points):
            if not finished:
                get_final_point(laser_vals)
                finished = True

          

    #publish speed
    speed_pub.publish(speed)
    if not reached_finish:
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
marker_topic = rospy.get_param("marker_topicname","/visualization_marker")
scan_topic = rospy.get_param("scan_topicname","/scan")

print (marker_topic)
print (odom_topic)
print (scan_topic)

odom_sub = rospy.Subscriber(str(odom_topic), Odometry, callback_odom)
marker_sub = rospy.Subscriber(str(marker_topic), Marker, callback_marker)
scan_sub = rospy.Subscriber(str(scan_topic),LaserScan, callback_laser)

print ("mover started")

while not rospy.is_shutdown():
    rospy.spin()