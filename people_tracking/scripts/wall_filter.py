#!/usr/bin/env python

import rospy
import numpy
from std_msgs.msg import *
import geometry_msgs.msg
from sensor_msgs.msg import LaserScan
from people_tracking.srv import LoadWalls, LearnWalls

import rospkg
import os
import json

import base64
import json
import numpy as np

class WallFilter:
    def __init__ (self, scan_topic):
        self.filter_set = False
        
        ### The PLS Model oscillates between returning 180 pts and 181 pts
        # note wall offsets slightly between each scan so two arrays are needed 
        self.walls = [[],[]]
        self.new_walls = [[]]
        self.reset_count = 0
        self.reset_thresh = 20
        self.switch = False

        self.scan_topic = scan_topic

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('people_tracking')
        self.fp_0 = os.path.join(pkg_path,"extra", "wall_locations_0.txt") # 2 files, again cause the janky 180/181 oscillating returned ranges
        self.fp_1 = os.path.join(pkg_path,"extra", "wall_locations_1.txt")

        ## Load the walls from the start
        load_walls_srv_req = LoadWalls()
        load_walls_srv_req.load_file = True
        self.load_walls(load_walls_srv_req)

        self.filter_scan_pub = rospy.Publisher("filtered_scan", LaserScan, queue_size=10)
        self.scan_sub = rospy.Subscriber(self.scan_topic, LaserScan, self._rm_walls, self.filter_scan_pub) #needs to be subscribed to wall laser topic

        self.load_walls_service = rospy.Service('wall_filter/load_walls', LoadWalls, self.load_walls)
        self.learn_walls_service = rospy.Service('wall_filter/learn_walls', LearnWalls, self.learn_walls)





    #When the room is empty call filter reset
    def reset_filter(self, run):
        self.filter_set = False

    def _rm_walls(self, data, publisher):
        variance = 0.1 #to account for noise
        switch = int(self.switch) ### The PLS Model oscillates between returning 180 pts and 181 pts, this toggle tracks that in a hacky way
        self.switch = not self.switch #toggle switch
        
        #if the filter is in place remove walls
        if self.filter_set:
            walls = self.walls[switch]
            ranges = data.ranges
            filtered_ranges = []
            
            for i in xrange(len(walls)):
                try:
                    if ranges[i] < (walls[i]-variance):
                        filtered_ranges.append(ranges[i])
                    else:
                        filtered_ranges.append(data.range_max+1) #invalidate the result at this point
                except IndexError:
                    filtered_ranges.append(data.range_max+1)
            
            #publish filtered_ranges as LaserScan
            filtered_scan = data
            h = std_msgs.msg.Header()
            h.stamp = data.header.stamp
            h.frame_id = data.header.frame_id
            filtered_scan.header = h
            filtered_scan.ranges = filtered_ranges
            publisher.publish(filtered_scan)
        
        #if the filter reset has been called use data to change filter instead
        else:
            #scan the room a specified # of times into an array
            # take the median of those scans at each index value 
            # once the minimum number of scans is reached
            if self.reset_count < self.reset_thresh:
                self.new_walls.append([])
                self.new_walls[self.reset_count] = data.ranges
                self.reset_count += 1
            
            elif self.reset_count == self.reset_thresh:
                #unzip new_walls (to go by point instead of dataset) - len should be ~180

                zipped0 = map(list, zip(*filter(None, self.new_walls[0::2])))
                zipped1 = map(list, zip(*filter(None, self.new_walls[1::2])))
                self.walls[0] = [numpy.median(z) for z in zipped0]
                self.walls[1] = [numpy.median(z) for z in zipped1]
                
                #reset vars
                self.reset_count = 0
                self.new_walls = [[]]
                self.filter_set = True

                #save the new wall data (as 2 seperate files since there is a 180 range and 181 range)
                wall_array_zero = np.array(self.walls[0], dtype=np.float64)
                wall_array_one = np.array(self.walls[1], dtype=np.float64)

                numpy.savetxt(self.fp_0, wall_array_zero, delimiter=",")
                numpy.savetxt(self.fp_1, wall_array_one, delimiter=",")

                print "New walls learned and saved"

    def load_walls(self, request):
        if request.load_file == True:
            wall_array_zero = numpy.loadtxt(self.fp_0, delimiter=",")
            wall_array_one = numpy.loadtxt(self.fp_1, delimiter=",")

            if len(wall_array_one) > 179 and len(wall_array_one) > 179:
                self.walls = [[],[]]
                self.walls[0] = wall_array_zero.tolist()
                self.walls[1] = wall_array_one.tolist()

                self.filter_set = True
                self.reset_count = 0
                self.new_walls = [[]]
                print "walls loaded"
                return True
        else:
            return False


    def learn_walls(self, request):
        ## The actual learning of the walls takes place in the _rm_walls function
        ##   Here the params are set to allow that to happen
        if request.learn_walls == True:
            self.reset_count = 0
            self.new_walls = [[]]
            self.filter_set = False

            self.reset_thresh = request.num_laser_msgs

            return True
        
        else:
            return False





if __name__ == '__main__':
    
    rospy.init_node("wall_filter", anonymous=False)

    scan_topic = rospy.get_param("~scan_topic")
    
    wall_filter = WallFilter(scan_topic)
    
    rospy.spin()
