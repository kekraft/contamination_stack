#!/usr/bin/env python

import numpy as np
import sys
import re
from math import cos, sin, acos, sqrt, ceil

import yaml

import rospy
import tf
import geometry_msgs.msg
from std_msgs.msg import *
from nav_msgs.msg import OccupancyGrid, MapMetaData
from visualization_msgs.msg import Marker, MarkerArray

from contamination_grid import Contamination

#This class converts ellipse data into an OccupancyGrid message

class Contamination2(Contamination):
    
    def __init__(self):
        Contamination.__init__(self)
        
        self.ppl_contam_levels = dict()
        self.layout = MultiArrayLayout([MultiArrayDimension(label="contam", stride=1)], 0)

        metadata = rospy.wait_for_message("map_metadata", MapMetaData, 120)
        self._set_map_metadata(metadata)

        self.occ_grid_pub = rospy.Publisher("contamination_grid", OccupancyGrid, queue_size=10, latch=True)
        self.contam_pub = rospy.Publisher("contam_array", Float32MultiArray, queue_size=10)

        self.tracking_marker_array_sub = rospy.Subscriber("multiperson_markers", MarkerArray, self._check_contam)
        # self.bot_marker_sub = rospy.Subscriber("cleaner_bot", Marker, self._clean_contam)
        self.update_filter_sub = rospy.Subscriber("update_filter_cmd", Bool, self.reset)

        # self.listener = tf.TransformListener()
        
        # rate = rospy.Rate(10.0)
        
        self.reset(True)

    def _check_contam(self, ellipse_array):
        #check to see if they have become contaminated
        #print self.ppl_contam_levels

        ## commented out 2 strs below
        # while len(self.ppl_contam_levels) < len(ellipse_array.markers):
        #     self.ppl_contam_levels.append(-1)

        for ellipse in ellipse_array.markers:

            ## add in any new found people to the contam levels that are being tracked
            if not self.ppl_contam_levels.keys() or ellipse.id not in self.ppl_contam_levels.keys():
                self.ppl_contam_levels[ellipse.id] = int(ellipse.text)


            (center, a, b, theta) = self._get_ellipse_data(ellipse) #convert marker to points
            
            for k, v in self.contam.iteritems():
                distance = self._e_dist(v, center, a, b, theta)
                
                # if person is in area increase relative contamination
                #print ellipse.id, v, center
                if distance < 1 and self.ppl_contam_levels[ellipse.id] < self.ogrid.data[k]:
                    self.ppl_contam_levels[ellipse.id] = self.ogrid.data[k] * self.infectivity
                    #print ellipse.id, self.ppl_contam_levels[ellipse.id]
                    self.ogrid.data[k] = int(self.ogrid.data[k] * (1-0.5 * self.infectivity))
            
            #if person is contaminated after that check, amend contamination levels in points
            if self.ppl_contam_levels[ellipse.id] > -1:
                #contamination levels decrease as person distributes sickness around
                #self.ppl_contam_levels[ellipse.id] = self.ppl_contam_levels[ellipse.id]*self.rate
                #outline square that fits ellipse and then find points within ellipse
                for x in np.arange(center[0]-a, center[0]+a, self.step):
                    
                    for y in np.arange(center[1]-a, center[1]+a, self.step):

                        distance = self._e_dist((x, y), center, a, b, theta)
                        index = self._xy_to_cell((x, y))
                        
                        #if area within ellipse, transfer some disease
                        if distance < 1:
                            self.ogrid.data[index] += int(ceil(self.ppl_contam_levels[ellipse.id] * self.transfer))
                            
                            if self.ogrid.data[index] > 100:
                                self.ogrid.data[index] = 100
                            
                            if index not in self.contam:
                                self.contam[index]=(x, y)
                
                self.ppl_contam_levels[ellipse.id] *= 1-self.transfer
        
        self.ogrid.header = Header(stamp=rospy.Time.now(),frame_id = "map")
        self.occ_grid_pub.publish(self.ogrid)

        #publish contamination to show correct colors
        self.layout.dim[0].size = len(self.ppl_contam_levels)
        self.contam_pub.publish(Float32MultiArray(self.layout, self.ppl_contam_levels))

    def reset(self, run):
        Contamination.reset(self, run)
        self.ppl_contam_levels = dict()


if __name__ == '__main__':

    rospy.init_node('contamination_multitracking_grid', anonymous=True)

    grid = Contamination2()
   
    rospy.spin()