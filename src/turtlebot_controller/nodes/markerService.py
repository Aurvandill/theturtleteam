#!/usr/bin/env python2

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point as Coordinates
from turtlebot_controller.srv import turtle_srvs 
from std_msgs.msg import ColorRGBA

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

#extract corner points from list of points
def cleanup_maplist():
    global mapList
    x_changed = False
    y_changed = False
    last_x = 0
    last_y = 0
    newlist = []
    #add point to newlist if x and y changed
    for item in mapList:
        cur_x = item.get_x()
        cur_y = item.get_y()
        if last_x != cur_x:
            x_changed = True
        if last_y != cur_y:
            y_changed = True
        if x_changed and y_changed:
            tmp_y = 0.0
            #tune the values
            if last_y > 0.0:
                tmp_y = float(last_y) +0.025
            else:
                tmp_y = float(last_y) -0.025
            newlist.append(Point(last_x, tmp_y))
            x_changed = False
            y_changed = False
        last_x = item.get_x()
        last_y = item.get_y()
    #get last point
    last_x = mapList[len(mapList) - 1].get_x()
    last_y = mapList[len(mapList) - 1].get_y()
    #tune value
    if last_y > 0.0:
        last_y = float(last_y) +0.025
    else:
        last_y = float(last_y) -0.025
    newlist.append(Point(last_x, last_y))
    #return list with corner points
    return newlist


#publish marker
def pubValues():
    global pub
    global mapList
    #initialize marker
    linecolor = ColorRGBA()
    linecolor.b = 1.0
    linecolor.a = 1
    marker = Marker()
    marker.header.frame_id = "/odom"
    marker.id = 0
    marker.type = 4
    marker.action = marker.ADD
    marker.scale.x = 0.03
    marker.color.a = 1
    marker.color.r = 1.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.color = linecolor
    #add points to marker
    for item in cleanup_maplist():
        coordinate = Coordinates()
        coordinate.x = float(item.get_x())
        coordinate.y = float(item.get_y())
        coordinate.z = 0
        marker.points.append(coordinate)
    #publish marker
    rate = rospy.Rate(15)
    while not rospy.is_shutdown():
        pub.publish(marker)
        rate.sleep()


#handle from service
def handle(request):
    global mapList
    #extract coordinates from request and put them into mapList
    data = str(request.data)
    tmpList = data.split(";")
    for item in tmpList:
        item = str(item)
        value = item.split(",")
        mapList.append(Point(value[0], value[1]))
    pubValues()


#initialize node, service, publisher
def init():
    global pub
    rospy.init_node("marker_service")
    rospy.Service("set_marker_service", turtle_srvs, handle)
    pub = rospy.Publisher("/visualization_marker", Marker, queue_size=100)
    rospy.spin()


if __name__ == "__main__":
    mapList = []
    init()