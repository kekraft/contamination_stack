#!/usr/bin/env python

import numpy as np
import sys
import re
from math import cos, sin, acos, sqrt, ceil

import yaml
import threading
import copy

import rospy
import tf
import geometry_msgs.msg
from std_msgs.msg import *
from nav_msgs.msg import OccupancyGrid, MapMetaData
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Polygon, PolygonStamped, PointStamped
import rospkg
import tf
from people_tracking.msg import PersonLocation, PersonLocationArray
from contamination_monitor.msg import ContamPersonLocation, ContamPersonLocationArray

from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from tf.transformations import quaternion_inverse
from tf import transformations

import os
import timeit
import subprocess


### For this, we don't care so much about ppl, so much as moving objects, thus keep track of object id
###   and contamination level

class WarnSystem():
    NOT_CONTAMINATED = -1
    FULLY_CONTAMINATED = 100
    UNKNOWN_CONTAMINATED = 50
    
    def __init__(self):
        self.contam_level = -1 # neg 1 corresponds to unkonwn in occupancy grid
        self.lock = threading.Lock()

        self.ogrid = OccupancyGrid()
        self.step = 0
        self.offset = (0,0)

        self.area_extension = 0.5

        self.objects_contam_level = {} # stores tracked objects contamination level, k=ob_id, v=contam level

        self.tf_listener = tf.TransformListener()
        
        self.ppl_warnings = dict()

        # Setup default filepath 
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('contamination_monitor')
         
        self.occ_grid_sub = rospy.Subscriber("contamination_grid", OccupancyGrid, self.occ_grid_update, queue_size=10)        
        self.contam_person_sub = rospy.Subscriber("contam_people_locations", ContamPersonLocationArray, self.contam_person_update, queue_size=10)
           

    def occ_grid_update(self, ogrid_msg):
        with self.lock:
            self.ogrid = ogrid_msg
            self.step = ogrid_msg.info.resolution
            self.offset = (ogrid_msg.info.origin.position.x, ogrid_msg.info.origin.position.y)

    def contam_person_update(self, cont_person_loc_msg):     
        for contam_person_loc in cont_person_loc_msg.contam_people_location:

            person_loc = contam_person_loc.person_location
            contam_level = contam_person_loc.contamination_level
            # print "person: ", person_loc.name
            # print "contam_level: ", contam_level
            
            person_id = person_loc.name
            x = person_loc.pose.position.x
            y = person_loc.pose.position.y
            z = person_loc.pose.position.z
            orientation = person_loc.pose.orientation
            a = person_loc.ellipse_a
            b = person_loc.ellipse_b
            theta = person_loc.ellipse_theta
            frame_id = person_loc.header.frame_id

            # Check to see if human needs to be warned
            if self.needs_warning(person_id, contam_level, (x,y,z) , a, b, theta):
                self.send_warning(person_id, (x,y,z))
                self.log_warning(person_id, (x,y,z), rospy.Time.now())

                        



    def needs_warning(self, person_id, contam_level, ellipse_center, ellipse_a, ellipse_b, ellipse_theta):
        '''  Checks to see if human needs to be warned.
              
              In the current basic form that just see's if the human is near an area
               that it needs to see.

              In future, get person's trajectory
               make boundary around contaminated areas
               see if the trajectory, as a ray, intersects 
               with the boundary areas

        '''
        # print "checking for warning for: ", person_id
        
        # if the human is not fully contaminated
        #  check to see if they are near a contaminated area
        if contam_level < self.FULLY_CONTAMINATED:

            ## see if the human has been warned recently and is still in the same area they were warned in previously,
            ##    if so, don't warn them now
            if person_id in self.ppl_warnings:
                ### check to see if location is within the last warning area within the threshold
                warnings = self.ppl_warnings[person_id]
                warned_in_area = warnings[-1][0]
                ## check to see if duration is within the warning time threshold
                warned_recently = warnings[-1][1]

                if warned_in_area or warned_recently:
                    return False
            
            # see if human near contaminated area
            #outline square that fits ellipse and then find points within ellipse and infect those pts on the grid
            for x in np.arange(ellipse_center[0] - ellipse_a - self.area_extension, 
                               ellipse_center[0] + ellipse_a + self.area_extension, self.step):            
                for y in np.arange(ellipse_center[1] - ellipse_a - self.area_extension, 
                               ellipse_center[1] + ellipse_a + self.area_extension, self.step):

                    try:
                        index = self._xy_to_cell((x, y))
                        if self.ogrid.data[index] > self.UNKNOWN_CONTAMINATED:
                            print "Near contamination!"
                            return True
                    except IndexError:
                        pass

        return False

    def send_warning(self, person_id, center):
        print "WARNING, APPROACHING CONTAMINATION"
        subprocess.call(["espeak", "-a", "200", "Warning"]) #call(["/usr/bin/meshlabserver", "-i", "points_raw.xyz", "-o", "points_mesh.ply", "-s", "xyz_to_mesh.mlx"])

    def log_warning(self, person_id, center, time):
        '''  Logs the warning sent to a human
        '''
        if person_id not in self.ppl_warnings:
            self.ppl_warnings[person_id] = list()

        self.ppl_warnings[person_id].append((center, time))

    @todo
    def clear_warnings(self, person_id):
        ''' Clear either all the warnings or the one related to the person_id given '''
        pass



    def _xy_to_cell(self, xy):
        #translate xy to cell - each cell is <resolution> meters wide
        x = int(round((xy[0] - self.offset[0])/self.step))
        y = int(round((xy[1] - self.offset[1])/self.step))
        return y * self.ogrid.info.width + x
        #return x, y

    def _snap_to_cell(self, xy):
        #snap xy coordinate to cell and return modified xy
        return (round(xy[0]/self.step) * self.step,
                round(xy[1]/self.step) * self.step)

        
    #turn ellipse into points
    def tf_person_2_contam_grid(self,x, y, theta, orientation, frame_id):
        #transform from ellipse frame to map frame and get ellipse data
        #FORMULA ASSUMES LASER AND MAP ARE ROTATED ONLY AROUND Z AXIS
        try:
            self.tf_listener.waitForTransform(self.ogrid_frame_id, frame_id, rospy.Time(0), rospy.Duration(0.5))
            (trans,rot) = self.tf_listener.lookupTransform(self.ogrid_frame_id, '/' + frame_id, rospy.Time(0))
        
            angle = acos(rot[3]) * 2
            center = (x*cos(angle) - y*sin(angle) + trans[0], x*sin(angle) + y*cos(angle) + trans[1])            
            q = orientation
            w = (q.w*rot[3] - q.x*rot[0] - q.y*rot[1] - q.z*rot[2])
            theta = acos(w)*2
            
            '''
            x = ellipse.pose.position.x
            y = ellipse.pose.position.y
            angle = acos(rot[3]) * 2

            center = (x*cos(angle) - y*sin(angle) + trans[0], x*sin(angle) + y*cos(angle) + trans[1])
            (a, b) = (ellipse.scale.x/2, ellipse.scale.y/2)
            
            q = ellipse.pose.orientation
            w = (q.w*rot[3] - q.x*rot[0] - q.y*rot[1] - q.z*rot[2])
            theta = acos(w)*2
            '''
            
            return center, theta

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "no tf found"
            return None

    #return distance from point to ellipse
    def _e_dist(self, point, center, a, b, theta):
        return (pow((abs(cos(theta)*(point[0] - center[0])) + abs(sin(theta)*(point[1] - center[1])))/a, 2) +
                pow((abs(sin(theta)*(point[0] - center[0])) + abs(cos(theta)*(point[1] - center[1])))/b, 2))



if __name__ == '__main__':

    rospy.init_node('warning_system')

    warn = WarnSystem()

    rospy.spin()