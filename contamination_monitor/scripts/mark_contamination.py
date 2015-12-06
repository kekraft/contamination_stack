#!/usr/bin/env python
"""
This script creates rviz markers representing people.
The marker color is varied based on the contamination level/belief of the people.

It relies on the PersonLocation2D msg.
Subscribes to : persons_location
Publishes to : persons_marker

The marker gets the same frame_id as the received PersonLocation2D msg.
"""


import rospy
from std_msgs.msg import *
from geometry_msgs.msg import Pose, Quaternion, Point, Vector3
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

from math import sin,cos,atan2,pi,sqrt

from ellipse2d import Ellipse2d
from contamination_monitor.msg import PersonLocation2D, PersonLocation2DArray

import hashlib

class PersonMarker():
    def __init__(self, scale_factor=0.5, person_height=0.5, multi_case = False):
        self.scale_factor = scale_factor #scale, in meters
        self.person_height = person_height

        self.multi_case = multi_case

        if multi_case:
            self.people_locations_sub = rospy.Subscriber("people_locations", PersonLocation2DArray, self.people_locations_cb)        
            self.people_marker_pub = rospy.Publisher("multiperson_markers", MarkerArray, queue_size=10)
            self.people_label_marker_pub = rospy.Publisher("multperson_label_markers", MarkerArray, queue_size=10)

        else:
            self.person_location_sub = rospy.Subscriber('persons_location', PersonLocation2D, self.person_locations_cb)
            self.person_marker_pub = rospy.Publisher("persons_marker", Marker, queue_size=10)

        
    def people_locations_cb(self, data):
        ## Handles an array of personlocation2d 
        ## creates a marker for each and publishes an
        ## array of the markers

        ## holds the objects representing people
        marker_array = MarkerArray()
        marker_array.markers = []

        ## contains labels for people
        marker_label_array = MarkerArray()
        marker_label_array.markers = []

        for person in data.people_location_2d:
            ## in future have a heat map color marker
            color = self.contamination_color(person.contamination)

            marker = self.create_person_marker(person, color)
            marker_label = self.create_person_label_marker(person, color)

            marker_array.markers.append(marker)
            marker_label_array.markers.append(marker_label)

        self.people_marker_pub.publish(marker_array)
        self.people_label_marker_pub.publish(marker_label_array)
            


    def person_locations_cb(self, data):
        ## Handles a singler personLocation2d topic cb
        ## in future have a heat map color marker
        color = self.contamination_color(data.contamination)

        marker = self.create_person_marker(data, color)
        self.person_marker_pub.publish(marker)

    def contamination_color(self, contam_belief):
        red_val = contam_belief
        blue_val = 1.0 - contam_belief
        color = ColorRGBA(red_val, 0.0, blue_val, 1.0)
        # if contam_belief > 0.5:
        #     color = ColorRGBA(1.0,0.0,0.0,1.0)
        # else:
        #     color = ColorRGBA(0.0,0.0,1.0,1.0)    
        
        return color
    
    def create_person_marker(self, person, color):
        h = Header()
        h.frame_id = person.header.frame_id #tie marker visualization to laser it comes from
        h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
        
        #create marker:person_marker, modify a red cylinder, last indefinitely
        mark = Marker()
        mark.header = h
        mark.ns = "person_marker"

        ## simple hack to map persons name to integer value for unique marker id
        char_list = list(person.name)
        char_int_list = [str(ord(x)) for x in char_list]
        char_int_str = "".join(char_int_list)
        char_int = int(char_int_str) & 255
        print 
        print "str to int"
        print char_int_list
        print char_int
        print "Char int binary) ", bin(char_int)

        # mark.id = int(char_int / 10000)
        mark.id = char_int
        mark.type = Marker.CYLINDER # Marker.TEXT_VIEW_FACING 
        mark.action = 0
        mark.scale = Vector3(person.ellipse_a * self.scale_factor, person.ellipse_b * self.scale_factor, 1) 
        mark.color = color 
        print "person name: ", person.name
        mark.text = str(1)

        pose = Pose(Point(person.pose.x, person.pose.y, self.person_height), Quaternion(0.0,0.0,1.0,cos(person.pose.theta/2)))
        mark.pose = pose

        return mark

    def create_person_label_marker(self, person, color):
        h = Header()
        h.frame_id = person.header.frame_id #tie marker visualization to laser it comes from
        h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
        
        #create marker:person_marker, modify a red cylinder, last indefinitely
        mark = Marker()
        mark.header = h
        mark.ns = "person_label_marker"
        
        ## simple hack to map persons name to integer value for unique marker id
        char_list = list(person.name)
        char_int_list = [str(ord(x)) for x in char_list]
        char_int_str = "".join(char_int_list)
        char_int = int(char_int_str) & 255
        print 
        print "str to int"
        print char_int_list
        print char_int
        print "Char int binary) ", bin(char_int)

        # mark.id = int(char_int / 10000)
        mark.id = char_int
        mark.type = Marker.TEXT_VIEW_FACING 
        mark.action = 0
        mark.scale = Vector3(self.scale_factor * 0.5, self.scale_factor * 0.5, 1) 
        mark.color = color 
        print "person name: ", person.name
        mark.text = person.name

        pose = Pose(Point(person.pose.x + 0.75, person.pose.y + 0.5, self.person_height + 0.75), Quaternion(0.0,0.0,1.0,cos(person.pose.theta/2)))
        mark.pose = pose

        return mark

if __name__ == '__main__':
    rospy.init_node("person_marker_creator")

    scale = rospy.get_param('~scale_factor')
    person_height = rospy.get_param('~person_height')
    multiperson_case = rospy.get_param('~multiperson') # true == multiperson tracking, false == single person tracking

    person_marker = PersonMarker(scale, person_height, multiperson_case)
    
    rospy.spin()