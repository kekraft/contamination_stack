#!/usr/bin/env python

# Directs a robot to cleanning the occ grid
# Currently utilizes greedy evaluation

import numpy as np
import rospy
import tf
from std_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry, OccupancyGrid
from visualization_msgs.msg import Marker

import threading

class Type:
    SCRUBBER = 0
    UV = 1

class CleanerBot:
    def __init__(self):
        self.robot_diam = 0.354 #value in meters for turtlebot - can it be grabbed from other source?
        self.robot_height = 0.420 #value in meters for turtlebot - can it be grabbed from other source?
        
        self.type = Type.SCRUBBER

        self.current_pose = None
        self.current_map_pose = None

        self.amcl_pose_topic = "amcl_pose"
        self.odom_topic = "odom"
        self.use_amcl = True

        self.occ_grid = None

        self.occ_grid_sub = rospy.Subscriber("contamination_grid", OccupancyGrid, self.update_occ_grid)

        self.lock = threading.Lock()


        publisher = rospy.Publisher('cleaner_bot', Marker, queue_size=10)
        # rospy.Subscriber('odom', Odometry, self.clean, publisher)

        # publisher=rospy.Publisher('cleaner_bot', Marker, queue_size=10)
        # either go off amcl pose or odom
        

        if self.use_amcl:
            rospy.Subscriber(amcl_pose_topic, PoseWithCovarianceStamped, self.update_amcl_position)
        else:
            rospy.Subscriber(odom_topic, Odometry, self.update_odom_position)
        
    def update_occ_grid(self, data):
        with self.lock:
            print "Acquired cleaner bots occ grid"
            self.occ_grid = data 

    def update_amcl_position(self, data):
        # amcl returns pose with covariance stamped, lets just get the pose
        self.current_pose = data.pose
        # we need this in the map pose

    def update_odom_position(self, data):
        # odom has a pose with covariance in it
        self.current_pose = data.pose
        # we need this in the map pose

    def run_uv(self):
        ''' while not done cleaning:
                if not stationary:
                    get new goal and irradiation time 
                    go to goal and irradate
                else:
                    if current stationary duration >= irradiate_time:
                        break
        '''
        pass

    def run_scrubber(self):
        ''' 
        while not done cleaning:
           if at goal:
               get new goal
           else:
               goal = closest dirty, reachable pt
               go to goal and block
        '''
        pass

    def get_next_goal(self):
        goal = get_greedy_goal()

        return goal

    def get_greedy_goal(self):    
        '''
        Does BFS on the map
        '''
        start = (self.current_map_pose.x, self.current_map_pose.y)
        visited, queue = set(), [start]
        while queue:
            
            vertex = queue.pop(0) # this is slow, could be optimized with a priority queue
            
            if vertex == contaminated:
                return vertex
            
            else:
                visited.add(vertex)

                # get the children of the vertex
                children = 0 # get 4 connected children and check to see that they aren't in visited
                queue.extend(children)
            
        goal = self.current_map_pose
        return goal

    # def clean(self, odometry, publisher):
    #     #turn robot into cleaner-marker
    #     (x, y) = (odometry.pose.pose.position.x, odometry.pose.pose.position.y)
    #     h = std_msgs.msg.Header()
    #     h.frame_id = odometry.header.frame_id #tie marker visualization to source
    #     h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
    #     #publish marker:person_marker, modify a red cylinder, last indefinitely
    #     mark = Marker()
    #     mark.header = h
    #     mark.ns = "cleaner_bot"
    #     mark.id=0
    #     mark.type=3
    #     mark.action=0
    #     mark.pose=geometry_msgs.msg.Pose(geometry_msgs.msg.Point(x, y, self.robot_height/2),
    #                                      odometry.pose.pose.orientation)
    #     mark.scale=geometry_msgs.msg.Vector3(self.robot_diam,self.robot_diam,self.robot_height)
    #     mark.color=ColorRGBA(0, 0, 1, 1) #marker is blue
    #     publisher.publish(mark)


if __name__ == '__main__':
    rospy.init_node('cleaner_bot')

    cleaner = CleanerBot()

    cleaner.clean()

    rospy.spin()


    
