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

from contamination_monitor.srv import LoadContaminationGrid, SaveContaminationGrid, ResetContaminationGrid



#This class converts ellipse data into an OccupancyGrid message
class Transmission():
    BINARY = 0
    GAUSSIAN = 1

### For this, we don't care so much about ppl, so much as moving objects, thus keep track of object id
###   and contamination level

class ContaminationGrid2D():
    NOT_CONTAMINATED = -1
    FULLY_CONTAMINATED = 100
    UNKNOWN_CONTAMINATED = 50
    
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

        metadata = rospy.wait_for_message("map_metadata", MapMetaData) # this topic name should be passed in via parameter or config file
        self.init_empty_contam_map(metadata)
        self.ogrid_frame_id = "map"

        # Gets the inverse transform just using a lookup...
        # This should be done with a config setting
        self.tf_listener = tf.TransformListener()
        # # (trans,rot) = self.tf_listener.lookupTransform("/laser", "/map", rospy.Time(0))
        # trans = [-92.100, -92.400, 0.3 ]
        # rot = [0.003, 0.004, 0.764, 0.645]
        # self.tfros = tf.TransformerROS()
        # self.laser_2_map_tf_mat = self.tfros.fromTranslationRotation(trans, rot)
        # print "TF MATRIX"
        # print self.laser_2_map_tf_mat
        
        self.ppl_contam_levels = dict() # dictionary that holds ppl, their prev position, and their contam level
        self.ppl_contam_locs = dict() # dictionary that holds ppl, their prev position, and their contam level

        self.reset()

        # Setup default filepath 
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('contamination_monitor')
        self.default_file_path =  os.path.join(pkg_path, "extra", "contam_occ_grid") 

        # Load saved grid
        srv_msg = LoadContaminationGrid()
        srv_msg.load_contam_grid = True
        srv_msg.from_default_file = True
        srv_msg.file_path = "nan"
        # self.load_contam_grid(srv_msg)
        # print "Loaded saved map."

        # services for loading/saving occ grid
        self.load_contam_grid_service = rospy.Service('contamination_monitor/load_contam_grid', LoadContaminationGrid, self.load_contam_grid)
        self.save_contam_grid_service = rospy.Service('contamination_monitor/save_contam_grid', SaveContaminationGrid, self.save_contam_grid)
        self.reset_contam_grid_service = rospy.Service('contamination_monitor/reset_contam_grid', ResetContaminationGrid, self.reset_contam_grid)
         
        self.occ_grid_pub = rospy.Publisher("contamination_grid", OccupancyGrid, queue_size=10, latch = True)
        self.contam_people_locs_pub = rospy.Publisher("contam_people_locations", ContamPersonLocationArray, queue_size=10)
        
        self.tracking_marker_array_sub = rospy.Subscriber("people_locations", PersonLocationArray, self.update_contam)
        # self.bot_marker_sub = rospy.Subscriber("cleaner_bot", Marker, self._clean_contam)

        self.add_contam_sub = rospy.Subscriber("added_contamination_polygon", PolygonStamped, self.add_contam_cb)

        # self.pub_map_continually()
        self.pub_contam_grid_now()

    def pub_map_continually(self):
        
        r = rospy.Rate(2)

        while True:

            self.pub_contam_grid_now()

            r.sleep()
        

    def update_contam(self, person_array_msg):     
        # print "in update_contam"
        # start_time = timeit.default_timer()

        for person_loc in person_array_msg.people_location:
            
            person_id = person_loc.name
            x = person_loc.pose.position.x
            y = person_loc.pose.position.y
            z = person_loc.pose.position.z
            orientation = person_loc.pose.orientation
            a = person_loc.ellipse_a
            b = person_loc.ellipse_b
            theta = person_loc.ellipse_theta
            frame_id = person_loc.header.frame_id

            ## add in any new found people to the contam levels that are being tracked
            # if not self.ppl_contam_levels.keys() or person_id not in self.ppl_contam_levels.keys():
            if not self.ppl_contam_levels.keys() or person_id not in self.ppl_contam_levels:
                self.ppl_contam_levels[person_id] = self.UNKNOWN_CONTAMINATED

            try:
                # ellipse_data = self.tf_person_2_contam_grid(ellipse) #convert marker to points 
                ellipse_data = self.tf_person_2_contam_grid(x, y, theta, orientation, frame_id) # convert person data to workable ellipse 
                
                if ellipse_data is not None:

                    (center, theta) = ellipse_data
                    # (center, a, b, theta) = ellipse_data
                    # print "Center ", center
                    # print "a ", a
                    # print "b ", b
                    # print "theta ", theta

                    ## add in the persons location, relative to the occ grid
                    person_loc.pose.position.x = center[0]
                    person_loc.pose.position.y = center[1]
                    person_loc.ellipse_theta = theta
                    person_loc.header.frame_id = self.ogrid_frame_id

                    self.ppl_contam_locs[person_id] = person_loc # adding in person's location

                    with self.lock:

                        if self.transmission_method == Transmission.BINARY:
                            
                            # Update for spreading person contamination to environment
                            contam_grid_cells = self.binary_person_2_env(person_id, center, a, b, theta)

                            # Update for spreading environment contamination to person
                            self.binary_env_2_person(person_id, center, a, b, theta)

                            # update occ grid and contamination after person spread
                            # print "Num grid cells: ", len(contam_grid_cells)
                            self._add_contam_via_indices(contam_grid_cells)

                        elif self.transmission_method == Transmission.GAUSSIAN:
                            pass

            except ValueError:
                # Goofy ellipse, pass on this
                pass
                
        self.pub_contam_grid_now()
        self.pub_people_markers(person_array_msg)


        # code you want to evaluate
        # elapsed = timeit.default_timer() - start_time
        # print "Time elapsed: ", elapsed

        # publish contamination info of the ppl, includes their contam level and location
        self.pub_people_contam_locations()


    def binary_person_2_env(self, ellipse_id, ellipse_center, ellipse_a, ellipse_b, ellipse_theta):
        ''' This method spreads contamination from the person to the environment,
                if the person is contaminated.
        '''
        contam_indices = []

        current_per_contam_level = self.ppl_contam_levels.get(ellipse_id)


        # print "Ellipses:"
        # print self.ppl_contam_levels
        
        if current_per_contam_level < self.FULLY_CONTAMINATED:
            # no contamination to spread
            # return grid_copy, contam_locs
            return contam_indices

        else:
            #outline square that fits ellipse and then find points within ellipse and infect those pts on the grid
            for x in np.arange(ellipse_center[0]-ellipse_a, ellipse_center[0]+ellipse_a, self.step):            
                for y in np.arange(ellipse_center[1]-ellipse_a, ellipse_center[1]+ellipse_a, self.step):

                    # distance = self._e_dist((x, y), ellipse_center, ellipse_a, ellipse_b, ellipse_theta)
                    index = self._xy_to_cell((x, y))                
                    contam_indices.append(index)
                
        
        return contam_indices    


    def binary_env_2_person(self, ellipse_id, ellipse_center, ellipse_a, ellipse_b, ellipse_theta):
        #     # if person is in thresholded area of contamination, set them to full contaminated
        current_per_contam_level = self.ppl_contam_levels.get(ellipse_id)

        if current_per_contam_level == self.FULLY_CONTAMINATED:
            # person already contaminated, return
            return

        ## Loop through thresholded area of where person is standing. 
        ## If any of these areas are contaminated, mark the person as contaminated
        for x in np.arange(ellipse_center[0]-ellipse_a, ellipse_center[0]+ellipse_a, self.step):            
            for y in np.arange(ellipse_center[1]-ellipse_a, ellipse_center[1]+ellipse_a, self.step):
                    index = self._xy_to_cell((x, y))                
                    if self.ogrid.data[index] > self.NOT_CONTAMINATED:
                        self.ppl_contam_levels[ellipse_id] = self.FULLY_CONTAMINATED
                        return



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

    def reset(self):
        '''
        Loads the config file for the occupancy grid.
        Sets each grid cell.
        Clears prior knowledge of the peoeple.
        '''
        with self.lock:
            self.contam_level = -1
            self.contam = {}
            self.ogrid.data = [self.NOT_CONTAMINATED for i in xrange(self.ogrid.info.width * self.ogrid.info.height)]
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
                        #self._add_contam(v["lower_left"], v["upper_right"], v["intensity"])
                        pass

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

        if print_stuff:
            i = 0
        for x in np.arange(lower_left[0], upper_right[0], self.step):
            for y in np.arange(lower_left[1], upper_right[1], self.step):
                index = self._xy_to_cell((x, y))
                if print_stuff:
                    # print index
                    i += 1
                self.ogrid.data[index] = intensity
                self.contam[index] = (x, y)

        if print_stuff:
            print "Added ", i ," cells"

        self.lock.release()

    def _add_contam_via_indices(self, indices_list):
        for index in indices_list:

            self.ogrid.data[index] = self.FULLY_CONTAMINATED
                    
            # self.contam_locs[index] = (x, y) #marked as contam spot now
                
        
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


    def _clean_contam(self, ellipse):
        ''' This method updates the occ grid based on the presence of a cleaner cleaner_bot
            in the area. This method depends on modeling the cleaning surface of the bot as a 
            an ellipse. 
        '''
        (center, a, b, theta) = self.tf_person_2_contam_grid(ellipse)
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
        
        self.pub_contam_grid_now()


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
            with open(fp + ".yaml", 'w+') as outfile:
                print "Saving to FP ", fp
                outfile.write( yaml.dump(data, default_flow_style=False) )
                
                
                np.savetxt(fp + ".csv", np.array(self.ogrid.data, dtype=np.int8), delimiter=",")
                print "Saved"
                is_written = True
                
            self.lock.release()     

        print "Saved and released"
        self.pub_contam_grid_now()
        return is_written

    def load_contam_grid(self, srv_cmd):
        ''' Loads a contamination grid from a stored file.
            Either uses default filepath or one supplied by user.
        '''

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
                self.pub_contam_grid_now()

        print "Is loaded: ", is_loaded
        
        return is_loaded


        

    def reset_contam_grid(self, srv_cmd):
        ''' Loads the initial contam occ grid yaml settings 
            and sets every space to no contamination. '''

        is_reset = False
        if srv_cmd.reset_contam_grid == True:
            self.reset()
            is_reset = True
            self.pub_contam_grid_now()

        print "Reset? ", is_reset
        
        return is_reset

    def add_contam_cb(self, msg):
        ''' Makes these cells in the occ grid contaminated.
        '''
        ## Make sure the points are in the correct frame
        frame_id = msg.header.frame_id
        tf_mat = self.get_tf_mat_2_contam_grid(frame_id)

        ## Determine lower left and upper right since adding as a square currently
        lower_left = None
        upper_right = None
        for pt in msg.polygon.points:
            xyz = self.tf_pt_2_contam_grid(pt.x, pt.y, pt.z, frame_id, tf_mat)

            if (lower_left is None) or \
               (xyz[0] <= lower_left[0] and xyz[1] <= lower_left[1]):
                lower_left = xyz
            elif (upper_right is None) or \
                        (xyz[0] >= upper_right[0] and xyz[1] >= upper_right[1]):
                upper_right = xyz       

        print "Adding contam"
        self._add_contam(lower_left, upper_right, intensity=self.FULLY_CONTAMINATED, print_stuff=True)
        print "contam added"

        self.pub_contam_grid_now()


    def pub_contam_grid_now(self):
        with self.lock:
            self.ogrid.header = Header(stamp=rospy.Time.now(),frame_id = self.ogrid_frame_id)
            self.occ_grid_pub.publish(self.ogrid)


    def tf_pt_2_contam_grid(self, x, y, z, frame_id, tf_mat = None):
        point = np.array([x, y, z, 0])

        if tf_mat is None:
            tf_mat = self.get_tf_mat_2_contam_grid(frame_id)

        xyz = tuple(np.dot(tf_mat, np.array([x, y, z, 1.0])))[:3]
      
        return xyz

    def get_tf_mat_2_contam_grid(self, frame_id):

        self.tf_listener = tf.TransformListener()
        self.tf_listener.waitForTransform(frame_id, self.ogrid_frame_id, rospy.Time(0), rospy.Duration(4.0))
        (trans,rot) = self.tf_listener.lookupTransform(self.ogrid_frame_id, frame_id, rospy.Time(0))

        self.tfros = tf.TransformerROS()
        tf_mat = self.tfros.fromTranslationRotation(trans, rot)  

        return tf_mat        

    def pub_people_markers(self, person_array_msg):
        for person_loc in person_array_msg.people_location:
            
            person_id = person_loc.name
            x = person_loc.pose.position.x
            y = person_loc.pose.position.y
            z = person_loc.pose.position.z
            orientation = person_loc.pose.orientation
            a = person_loc.ellipse_a
            b = person_loc.ellipse_b
            theta = person_loc.ellipse_theta
            frame_id = person_loc.header.frame_id

            ### it would be great here to call person marker to create the new marker...
            ## without starting a new node.
            ## there should be a way to do that in mark_people to satisfy both things here

    def pub_people_contam_locations(self):
        contam_ppl_locs = ContamPersonLocationArray()

        for key, value in self.ppl_contam_levels.iteritems():

            contam_per_loc = ContamPersonLocation()
            contam_per_loc.person_location = self.ppl_contam_locs[key] 
            contam_per_loc.contamination_level = value

            contam_ppl_locs.contam_people_location.append(contam_per_loc)

        self.contam_people_locs_pub.publish(contam_ppl_locs)




if __name__ == '__main__':

    rospy.init_node('contamination_grid')

    grid = ContaminationGrid2D()
    # grid.pub_map_continually()
   
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


            ellipse_data = self.tf_person_2_contam_grid(ellipse) #convert marker to points
            
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
        self.contam_people_locations.publish(Float32MultiArray(self.layout, self.ppl_contam_levels))
'''