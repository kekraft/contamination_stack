#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import Pose, Quaternion, Point, Vector3
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

from math import sin,cos,atan2,pi,sqrt
import random
import matplotlib.pyplot as plt

from ellipse2d import Ellipse2d
from contamination_monitor.msg import PersonLocation2D

class PersonLocationGenerator(object):
    """docstring for PersonLocationGenerator"""
    def __init__(self):
        self.x = 3.0
        self.y = 3.0
        self.theta = 1.57
        self.ellipse_a = 0.5
        self.ellipse_b = 0.35
        self.scan_frame_id = "laser"

        self.person_location_pub = rospy.Publisher("persons_location", PersonLocation2D, queue_size=10)

        self.generator_random_locations()

        
    def generator_random_locations(self):
        h = Header()
        person = PersonLocation2D()

        rate = rospy.Rate(.5)
        while not rospy.is_shutdown():
            # Shift data in the future

            ## generate location data
            # h = Header()
            h.frame_id = self.scan_frame_id
            h.stamp = rospy.Time.now()

            # person = PersonLocation2D()
            person.header = h
            person.pose.x = random.randint(0,5)
            person.pose.y = random.randint(0,5)
            person.pose.theta = random.random()
            person.ellipse_a = self.ellipse_a
            person.ellipse_b = self.ellipse_b
            person.contamination = 0.05

            ## publish
            self.person_location_pub.publish(person)

            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("person_location_generator")
    person_location_generator = PersonLocationGenerator()
    rospy.spin()