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
from geometry_msgs.msg import Polygon, PolygonStamped
import rospkg

import os

from contamination_monitor.srv import LoadContaminationGrid, SaveContaminationGrid

#This class converts ellipse data into an OccupancyGrid message
class Transmission():
    BINARY = 0
    GAUSSIAN = 1

class ContaminationGrid2D():
    NOT_CONTAMINATED = -1
    FULLY_CONTAMINATED = 100
    
    def __init__(self):
        self.contam_level = -1 # neg 1 corresponds to unkonwn in occupancy grid
        self.lock = threading.Lock()

        self.transmission_method = Transmission.BINARY

        self.ogrid = OccupancyGrid()
        self.step = 0
        self.offset = (0,0)

        #Efficiency of cleaning robot
        self.power = 0.0
        #Contaminant picked up from environment
        self.infectivity = 0.0
        #Contaminant transfered upon moving
        self.transfer = 0.0
        #Threshold distance from a cell that a person can pick up the contamination
        self.dist_thresh = 1.0

        #set of coordinates with contamination
        self.contam = {}

        metadata = rospy.wait_for_message("map_metadata", MapMetaData, 120) # this topic name should be passed in via parameter or config file
        self.init_empty_contam_map(metadata)

        self.listener = tf.TransformListener()
        
        self.ppl_contam_levels = dict()
        self.layout = MultiArrayLayout([MultiArrayDimension(label="contam", stride=1)], 0)

        self.occ_grid_pub = rospy.Publisher("contamination_grid", OccupancyGrid, queue_size=10)
        self.contam_pub = rospy.Publisher("contam_array", Float32MultiArray, queue_size=10)

        
        self.tracking_marker_array_sub = rospy.Subscriber("multiperson_markers", MarkerArray, self.update_contam)
        # self.bot_marker_sub = rospy.Subscriber("cleaner_bot", Marker, self._clean_contam)

        self.add_contam_sub = rospy.Subscriber("added_contamination_polygon", PolygonStamped, self.add_contam_cb)

        # services for loading/saving occ grid
        self.load_contam_grid_service = rospy.Service('contamination_monitor/load_contam_grid', LoadContaminationGrid, self.load_contam_grid)
        self.save_contam_grid_service = rospy.Service('contamination_monitor/save_contam_grid', SaveContaminationGrid, self.save_contam_grid)
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('contamination_monitor')
        self.default_file_path =  os.path.join(pkg_path, "extra", "contam_occ_grid")     

        self.reset(True)

        self.pub_map_continually()



        # while True:
        #     self.occ_grid_pub.publish(self.ogrid)

    def pub_map_continually(self):
        
        r = rospy.Rate(10)

        while True:

            self.lock.acquire()
            self.occ_grid_pub.publish(self.ogrid)
            self.lock.release()

            r.sleep()




    def add_contam_cb(self, msg):
        ''' Makes these cells in the occ grid contaminated.
        '''
        lower_left = msg.polygon.points[0]
        lower_left = [lower_left.x, lower_left.y]
        upper_right = msg.polygon.points[2]
        upper_right = [upper_right.x, upper_right.y]

        print "Adding contam"
        self._add_contam(lower_left, upper_right, intensity=self.FULLY_CONTAMINATED)
        print "contam added"
        
        

    def update_contam(self, ellipse_array):     

        for ellipse in ellipse_array.markers:

            ## add in any new found people to the contam levels that are being tracked
            if not self.ppl_contam_levels.keys() or ellipse.id not in self.ppl_contam_levels.keys():
                self.ppl_contam_levels[ellipse.id] = int(ellipse.text)


            ellipse_data = self._get_ellipse_data(ellipse) #convert marker to points
            
            if ellipse_data is not None:

                (center, a, b, theta) = ellipse_data

                with self.lock:

                    if self.transmission_method == Transmission.BINARY:
                        # Update for spreading person contamination to environment
                        grid_copy, contam_update = self.binary_person_2_env(ellipse.id, center, a, b, theta)

                        # Update for spreading environment contamination to person
                        self.binary_env_2_person(ellipse.id, center, a, b, theta)

                        # update occ grid and contamination after person spread
                        self.ogrid = grid_copy
                        self.contam.update(contam_update)

                    elif self.transmission_method == Transmission.GAUSSIAN:
                        pass
                
        
        # self.ogrid.header = Header(stamp=rospy.Time.now(),frame_id = "map")
        # self.occ_grid_pub.publish(self.ogrid)

        # #publish contamination to show correct colors
        # self.layout.dim[0].size = len(self.ppl_contam_levels)
        # self.contam_pub.publish(Float32MultiArray(self.layout, self.ppl_contam_levels))
        pass


    def binary_person_2_env(self, ellipse_id, ellipse_center, ellipse_a, ellipse_b, ellipse_theta):
        ''' This method spreads contamination from the person to the environment,
                if the person is contaminated.
        '''
        grid_copy = copy.deepcopy(self.ogrid)
        contam_locs = {}

        current_per_contam_level = self.ppl_contam_levels[ellipse_id]
        
        if current_per_contam_level < self.FULLY_CONTAMINATED:
            # no contamination to spread
            return grid_copy, contam_locs


        #outline square that fits ellipse and then find points within ellipse and infect those pts on the grid
        for x in np.arange(ellipse_center[0]-ellipse_a, ellipse_center[0]+ellipse_a, self.step):
            
            for y in np.arange(ellipse_center[1]-ellipse_a, ellipse_center[1]+ellipse_a, self.step):

                # distance = self._e_dist((x, y), ellipse_center, ellipse_a, ellipse_b, ellipse_theta)
                index = self._xy_to_cell((x, y))
                
                # # if area within ellipse, transfer some disease
                # if distance < self.dist_thresh:
                grid_copy.data[index] = self.FULLY_CONTAMINATED
                
                contam_locs[index] = (x, y) #marked as contam spot now

        return grid_copy, contam_locs


    def gauss_person_2_env(self, ellipse_center, ellipse_a, ellipse_b, ellipse_theta):
        ''' This method spreads contamination from the person to the environment,
                if the person is contaminated.
        '''

        grid_copy = copy.deepcopy(self.ogrid)

        for k, v in self.contam.iteritems():
            distance = self._e_dist(v, ellipse_center, ellipse_a, ellipse_b, ellipse_theta)

            if self.ppl_contam_levels[ellipse.id] != self.NOT_CONTAMINATED:

                #outline square that fits ellipse and then find points within ellipse
                for x in np.arange(ellipse_center[0]-ellipse_a, ellipse_center[0]+ellipse_a, self.step):
                    
                    for y in np.arange(ellipse_center[1]-ellipse_a, ellipse_center[1]+ellipse_a, self.step):

                        distance = self._e_dist((x, y), ellipse_center, ellipse_a, ellipse_b, ellipse_theta)
                        index = self._xy_to_cell((x, y))
                        
                        #if area within ellipse, transfer some disease
                        if distance < self.dist_thresh:
                            self.ogrid.data[index] += int(ceil(self.ppl_contam_levels[ellipse.id] * self.transfer))
                            
                            if self.ogrid.data[index] > self.FULLY_CONTAMINATED:
                                self.ogrid.data[index] = self.FULLY_CONTAMINATED
                            
                            if index not in self.contam:
                                self.contam[index]=(x, y)
                
                self.ppl_contam_levels[ellipse.id] *= 1-self.transfer


    def binary_env_2_person(self, ellipse_id, ellipse_center, ellipse_a, ellipse_b, ellipse_theta):
        #     # if person is in area increase relative contamination
        #     #print ellipse.id, v, center
        #     if distance < self.dist_thresh and self.ppl_contam_levels[ellipse.id] < self.ogrid.data[k]:
        #         self.ppl_contam_levels[ellipse.id] = self.ogrid.data[k] * self.infectivity
        #         #print ellipse.id, self.ppl_contam_levels[ellipse.id]
        #         self.ogrid.data[k] = int(self.ogrid.data[k] * (1-0.5 * self.infectivity))
        current_per_contam_level = self.ppl_contam_levels[ellipse_id]

        if current_per_contam_level == self.FULLY_CONTAMINATED:
            # person already contaminated, return
            return

        for k, v in self.contam.iteritems():
            distance = self._e_dist(v, ellipse_center, ellipse_a, ellipse_b, ellipse_theta)

            if distance < self.dist_thresh and current_per_contam_level < self.ogrid.data[k]:
                self.ppl_contam_levels[ellipse_id] = self.FULLY_CONTAMINATED
                return      

    def reset(self, run):
        '''
        
        '''
        self.contam_level = -1
        self.contam = {}
        self.ogrid.data = [0 for i in xrange(self.ogrid.info.width * self.ogrid.info.height)]
        #print sys.argv
        with open(sys.argv[1]) as f:
            for k, v in yaml.load(f.read()).iteritems():
                if k == "transfer":
                    self.transfer = v
                    print "transfer = {0}".format(v)
                elif k == "infectivity":
                    self.infectivity = v
                    print "infectivity = {0}".format(v)
                elif k == "cleaning_power":
                    self.power = v
                    print "cleaning power = {0}".format(v)
                elif re.match("c[0-9]+", k):
                    self._add_contam(v["lower_left"], v["upper_right"], v["intensity"])


        self.ppl_contam_levels = dict()


    def _xy_to_cell(self, xy):
        #translate xy to cell - each cell is <resolution> meters wide
        x=int(round((xy[0] - self.offset[0])/self.ogrid.info.resolution))
        y=int(round((xy[1] - self.offset[1])/self.ogrid.info.resolution))
        return y * self.ogrid.info.width + x
        #return x, y

    def _snap_to_cell(self, xy):
        #snap xy coordinate to cell and return modified xy
        return (round(xy[0]/self.ogrid.info.resolution) * self.ogrid.info.resolution,
                round(xy[1]/self.ogrid.info.resolution) * self.ogrid.info.resolution)

    def init_empty_contam_map(self, metadata):
        ''' create empty grid, according to metadata params '''
        data = [0 for i in xrange(metadata.width * metadata.height)]
        self.set_contam_map(metadata, data)


    def set_contam_map(self, metadata, data):
        ''' Create map according to meta data params with the given data for the grid cells.'''
        self.step = metadata.resolution
        self.offset = (metadata.origin.position.x, metadata.origin.position.y)

        self.ogrid.info = metadata
        self.ogrid.data = data

    #add initial contamination (rectangles)
    def _add_contam(self, lower_left, upper_right, intensity, print_stuff = False):

        self.lock.acquire()

        lower_left = self._snap_to_cell(lower_left)
        upper_right = self._snap_to_cell(upper_right)
        print "Lower left ", lower_left
        print "Upper right ", upper_right

        for x in np.arange(lower_left[0], upper_right[0], self.step):
            for y in np.arange(lower_left[1], upper_right[1], self.step):
                index = self._xy_to_cell((x, y))
                if print_stuff:
                    print index
                self.ogrid.data[index] = intensity
                self.contam[index] = (x, y)

        self.lock.release()
                
        
    #turn ellipse into points
    def _get_ellipse_data(self,ellipse):
        #transform from ellipse frame to map frame and get ellipse data
        #FORMULA ASSUMES LASER AND MAP ARE ROTATED ONLY AROUND Z AXIS
        try:
            (trans,rot) = self.listener.lookupTransform('/map', '/' + ellipse.header.frame_id, rospy.Time(0))
            
            x = ellipse.pose.position.x
            y = ellipse.pose.position.y
            angle = acos(rot[3]) * 2

            center = (x*cos(angle) - y*sin(angle) + trans[0], x*sin(angle) + y*cos(angle) + trans[1])
            (a, b) = (ellipse.scale.x/2, ellipse.scale.y/2)
            
            q = ellipse.pose.orientation
            w = (q.w*rot[3] - q.x*rot[0] - q.y*rot[1] - q.z*rot[2])
            theta = acos(w)*2
            
            return center, a, b, theta

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "no tf found"

    #return distance from point to ellipse
    def _e_dist(self, point, center, a, b, theta):
        return (pow((abs(cos(theta)*(point[0] - center[0])) + abs(sin(theta)*(point[1] - center[1])))/a, 2) +
                pow((abs(sin(theta)*(point[0] - center[0])) + abs(cos(theta)*(point[1] - center[1])))/b, 2))


    def _clean_contam(self, ellipse):
        ''' This method updates the occ grid based on the presence of a cleaner cleaner_bot
            in the area. This method depends on modeling the cleaning surface of the bot as a 
            an ellipse. 
        '''
        (center, a, b, theta) = self._get_ellipse_data(ellipse)
        #print center, a, b, theta
        for x in np.arange(center[0] - a, center[0] + a, self.step):
            for y in np.arange(center[1] - a, center[1] + a, self.step):
                distance = self._e_dist((x, y), center, a, b, theta)
                index = self._xy_to_cell((x, y))
                
                #if area within ellipse, remove disease
                if distance < self.dist_thresh:
                    self.ogrid.data[index] *= (1 - self.power)
                    if self.ogrid.data[index] > 100: 
                        self.ogrid.data[index] = 100
                    elif self.ogrid.data[index] <= 0:
                        self.ogrid.data[index] = 0
                        if index in self.contam: del self.contam[index]
                    #print self.ogrid.data[index]
        
        self.ogrid.header = Header(stamp=rospy.Time.now(), frame_id = "map")
        self.occ_grid_pub.publish(self.ogrid)


    def save_contam_grid(self, srv_cmd):
        is_written = False

        if srv_cmd.save_contam_grid == True:

            fp = ""
            if srv_cmd.from_default_file == True:
                fp = self.default_file_path
            else:
                fp = srv_cmd.file_path

            self.lock.acquire()

            data = dict(
                Header = self.ogrid.header,
                MapMetaData = self.ogrid.info,
                GridFile = fp + ".csv",
                )

            #### get grid data as numpy and save as seperate file       
            with open(fp + ".yaml", 'w') as outfile:
                print "Saving to FP ", fp
                outfile.write( yaml.dump(data, default_flow_style=False) )
                
                
                np.savetxt(fp + ".csv", np.array(self.ogrid.data, dtype=np.int8), delimiter=",")
                print "Saved"
                is_written = True
                
            self.lock.release()     

        print "Saved and released"
        return is_written

    def load_contam_grid(self, srv_cmd):

        is_loaded = False

        if srv_cmd.load_contam_grid == True:

            fp = ""
            if srv_cmd.from_default_file == False:
                fp = srv_cmd.file_path
            else:
                fp = self.default_file_path               


            with open(fp + ".yaml", "r") as infile:
                self.lock.acquire()
                for k, v in yaml.load(infile.read()).iteritems():
                    if k == "Header":
                        self.ogrid.header = v

                    elif k == "MapMetaData":
                        self.ogrid.info = v
                        self.step = v.resolution
                        self.offset = (v.origin.position.x, v.origin.position.y)

                    elif k == "GridFile":
                        self.default_file_path = v

                    else: 
                        raise TypeError("Unexpected key type in yaml file: " + fp)

                self.ogrid.data = list(np.loadtxt(fp + ".csv", dtype=np.int8, delimiter=","))

                self.lock.release()
                is_loaded = True

        print "Is loaded: ", is_loaded
        
        return is_loaded


if __name__ == '__main__':

    rospy.init_node('contamination_grid')

    grid = ContaminationGrid2D()
   
    rospy.spin()

'''  old update method
    def update_contam(self, ellipse_array):
        #check to see if they have become contaminated
        #print self.ppl_contam_levels

        ## commented out 2 strs below
        # while len(self.ppl_contam_levels) < len(ellipse_array.markers):
        #     self.ppl_contam_levels.append(-1)
        

        for ellipse in ellipse_array.markers:

            ## add in any new found people to the contam levels that are being tracked
            if not self.ppl_contam_levels.keys() or ellipse.id not in self.ppl_contam_levels.keys():
                self.ppl_contam_levels[ellipse.id] = int(ellipse.text)


            ellipse_data = self._get_ellipse_data(ellipse) #convert marker to points
            
            if ellipse_data is not None:

                (center, a, b, theta) = ellipse_data

                with self.lock:

                    # Update for spreading person contamination to environment
                    self.update_person_2_env(center, a, b, theta)

                    # Update for spreading environment contamination to person
                    self.update_env_2_person()
                
                    # for k, v in self.contam.iteritems():
                    #     distance = self._e_dist(v, center, a, b, theta)
                        
                    #     # if person is in area increase relative contamination
                    #     #print ellipse.id, v, center
                    #     if distance < self.dist_thresh and self.ppl_contam_levels[ellipse.id] < self.ogrid.data[k]:
                    #         self.ppl_contam_levels[ellipse.id] = self.ogrid.data[k] * self.infectivity
                    #         #print ellipse.id, self.ppl_contam_levels[ellipse.id]
                    #         self.ogrid.data[k] = int(self.ogrid.data[k] * (1-0.5 * self.infectivity))
                    
                    # #if person is contaminated after that check, amend contamination levels in points
                    # if self.ppl_contam_levels[ellipse.id] > -1:
                    #     #contamination levels decrease as person distributes sickness around
                    #     #self.ppl_contam_levels[ellipse.id] = self.ppl_contam_levels[ellipse.id]*self.rate
                    #     #outline square that fits ellipse and then find points within ellipse
                    #     for x in np.arange(center[0]-a, center[0]+a, self.step):
                            
                    #         for y in np.arange(center[1]-a, center[1]+a, self.step):

                    #             distance = self._e_dist((x, y), center, a, b, theta)
                    #             index = self._xy_to_cell((x, y))
                                
                    #             #if area within ellipse, transfer some disease
                    #             if distance < self.dist_thresh:
                    #                 self.ogrid.data[index] += int(ceil(self.ppl_contam_levels[ellipse.id] * self.transfer))
                                    
                    #                 if self.ogrid.data[index] > 100:
                    #                     self.ogrid.data[index] = 100
                                    
                    #                 if index not in self.contam:
                    #                     self.contam[index]=(x, y)
                        
                    #     self.ppl_contam_levels[ellipse.id] *= 1-self.transfer
        
        self.ogrid.header = Header(stamp=rospy.Time.now(),frame_id = "map")
        self.occ_grid_pub.publish(self.ogrid)

        #publish contamination to show correct colors
        self.layout.dim[0].size = len(self.ppl_contam_levels)
        self.contam_pub.publish(Float32MultiArray(self.layout, self.ppl_contam_levels))
'''