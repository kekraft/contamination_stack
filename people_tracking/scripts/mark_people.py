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
from people_tracking.msg import PersonLocation, PersonLocationArray

class PersonMarker():
    def __init__(self, scale_factor=2, person_height=1.75, laser_height = 1.2192):
        self.scale_factor = scale_factor #scale, in meters
        self.person_height = person_height
        self.laser_height = 2

        

        self.people_locations_sub = rospy.Subscriber("people_locations", PersonLocationArray, self.people_locations_cb)        
        self.people_marker_pub = rospy.Publisher("multiperson_markers", MarkerArray, queue_size=10)
        self.people_label_marker_pub = rospy.Publisher("multperson_label_markers", MarkerArray, queue_size=10)
        
        self.person_locs = dict() # dict k = ppls names, v = lists of position tuples
        self.people_line_loc_pub = rospy.Publisher("multiperson_line", Marker, queue_size=10)
        self.people_locations_sub = rospy.Subscriber("people_locations", PersonLocationArray, self.people_locations_line_cb)



        
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

        for person in data.people_location:
            ## in future have a heat map color marker
            # color = self.contamination_color(person.contamination)
            color = ColorRGBA(0.2, 0.5, 0.7, 1.0)

            marker = self.create_cylindrical_person_marker(person, color)
            marker_label = self.create_person_label_marker(person, color)

            marker_array.markers.append(marker)
            marker_label_array.markers.append(marker_label)

        self.people_marker_pub.publish(marker_array)
        self.people_label_marker_pub.publish(marker_label_array)
            


    def person_locations_cb(self, data):
        ## Handles a singler personLocation2d topic cb
        ## in future have a heat map color marker
        # color = self.contamination_color(data.contamination)
        color = ColorRGBA(0.2, 0.5, 0.7, 1.0)

        marker = self.create_cylindrical_person_marker(data, color)
        self.person_marker_pub.publish(marker)

    # def contamination_color(self, contam_belief):
    #     red_val = contam_belief
    #     blue_val = 1.0 - contam_belief
    #     color = ColorRGBA(red_val, 0.0, blue_val, 1.0)
    #     # if contam_belief > 0.5:
    #     #     color = ColorRGBA(1.0,0.0,0.0,1.0)
    #     # else:
    #     #     color = ColorRGBA(0.0,0.0,1.0,1.0)    
        
    #     return color
    
    def create_cylindrical_person_marker(self, person, color):
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
        print "p name ", int(float(person.name))
        mark.id = int(float(person.name)) #char_int
        print "mark id" , mark.id
        mark.type = Marker.CYLINDER # Marker.TEXT_VIEW_FACING 
        mark.action = 0
        mark.scale = Vector3(person.ellipse_a * self.scale_factor, person.ellipse_b * self.scale_factor, 1) 
        mark.color = color 
        mark.lifetime = rospy.Duration(0.5,0)
        print "person name: ", person.name
        mark.text = str(1)

        pose = Pose(Point(person.pose.position.x, person.pose.position.y, self.person_height - self.laser_height), Quaternion(0.0,0.0,1.0,cos(person.pose.position.z/2)))
        mark.pose = pose

        return mark

    @staticmethod
    def create_line_list_marker(person, color, line_points):
        h = Header()
        h.frame_id = person.header.frame_id #tie marker visualization to laser it comes from
        h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
        
        #create marker:person_marker, modify a red cylinder, last indefinitely
        mark = Marker()
        mark.header = h
        mark.ns = "person_marker"
        mark.id = int(float(person.name)) #char_int
        mark.type = Marker.LINE_STRIP# Marker.TEXT_VIEW_FACING 
        mark.action = 0
        mark.scale.x = 0.2 
        mark.color = color
        # mark.lifetime = rospy.Duration(0.5,0)
        print "person name: ", person.name
        mark.text = str(1)

        points = [Point(x[0], x[1], x[2]) for x in line_points]
        mark.points = points

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
        mark.id = int(float(person.name)) #char_int
        mark.type = Marker.TEXT_VIEW_FACING 
        mark.action = 0
        mark.scale = Vector3(self.scale_factor * 0.5, self.scale_factor * 0.5, 1) 
        mark.color = color 
        mark.lifetime = rospy.Duration(0.5,0)
        print "person name: ", person.name
        mark.text = person.name

        pose = Pose(Point(person.pose.position.x + 0.75, person.pose.position.y + 0.5, self.person_height + 0.75), Quaternion(0.0,0.0,1.0,cos(person.pose.position.z/2)))
        mark.pose = pose

        return mark

    def people_locations_line_cb(self, data):
        ## creates a line marker for person locatoin


        for person in data.people_location:
            ## in future have a heat map color marker
            # color = self.contamination_color(person.contamination)
            color = ColorRGBA(0.2, 0.5, 0.7, 1.0)
            person_loc = (person.pose.position.x, person.pose.position.y, person.pose.position.z)
            person.name 

            if person.name in self.person_locs:
                # Add location to that person's location list
                self.person_locs[person.name].append(person_loc) 
            else:
                # Add person into dictionary and add the loction as the first item in their list
                self.person_locs[person.name] = [person_loc]

            line_list_marker = self.create_line_list_marker(person, color, self.person_locs[person.name])
            self.people_line_loc_pub.publish(line_list_marker)
            
        

if __name__ == '__main__':
    rospy.init_node("person_marker_creator")

    try:
        scale_factor = rospy.get_param('~scale_factor')
        person_height = rospy.get_param('~person_height')

        person_marker = PersonMarker(scale_factor, person_height)
    except KeyError:
        person_marker = PersonMarker()
    
    rospy.spin()