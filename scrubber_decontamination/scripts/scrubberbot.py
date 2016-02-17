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
from move_base_msgs.msg import *

from scrubber_decontamination.srv import Clean
# from actionlib import SimpleActionClient, GoalStatus
import actionlib

import threading


class ScrubberBotPlanner:
    def __init__(self):
        self.robot_diam = 0.354 #value in meters for turtlebot - can it be grabbed from other source?
        self.robot_height = 0.420 #value in meters for turtlebot - can it be grabbed from other source?
        self.goal_wait_time = 20 # Amount of time (in seconds) to wait till a goal is reached before being canceled
        

        self.current_pose = None
        self.current_map_pose = None

        self.map_topic = "contamination_grid"
        self.amcl_pose_topic = "amcl_pose"
        self.odom_topic = "odom"

        self.use_amcl = True
        self.occ_grid = None

        self.lock = threading.Lock()

        self.occ_grid_sub = rospy.Subscriber(self.map_topic, OccupancyGrid, self.update_occ_grid)
        # self.pose_sub = rospy.Subscriber(self.amcl_pose_topic, PoseWithCovarianceStamped, self.update_pose)

        self.move_client = actionlib.SimpleActionClient('scrubber_bot/move_base', MoveBaseAction)
        

        self.has_goal = False

        publisher = rospy.Publisher('cleaner_bot', Marker, queue_size=10)

        self.goal_marker_pub = rospy.Publisher('scrubber_bot/goal_marker', Marker, queue_size=10, latch=True)
        # rospy.Subscriber('odom', Odometry, self.clean, publisher)

        # publisher=rospy.Publisher('cleaner_bot', Marker, queue_size=10)
        # either go off amcl pose or odom        

        if self.use_amcl:
            rospy.Subscriber(self.amcl_pose_topic, PoseWithCovarianceStamped, self.update_amcl_position)
        else:
            rospy.Subscriber(self.odom_topic, Odometry, self.update_odom_position)

        self.in_cleaning_mode = False

        self.clean_service = rospy.Service('scrubber_bot/clean', Clean, self.set_mode)



        
    def set_mode(self, srv_msg):
        with self.lock:
            self.in_cleaning_mode = srv_msg.clean
            print "In cleaning mode"

        return True

    def update_occ_grid(self, data):
        with self.lock:
            self.occ_grid = data 

    def update_amcl_position(self, data):
        # amcl returns pose with covariance stamped, lets just get the pose
        self.current_pose = data.pose.pose
        # we need this in the map pose

    def update_odom_position(self, data):
        # odom has a pose with covariance in it
        self.current_pose = data.pose.pose
        # we need this in the map pose

    def clean(self):
        ''' 
        while not done cleaning:
           if at goal:
               get new goal
           else:
               goal = closest dirty, reachable pt
               go to goal and block
        '''
        while True:
            while self.in_cleaning_mode:
                print "in cleaning mode"

                goal = self.get_next_goal()
                print "Got goal"
                print goal

                marker = self.create_goal_marker(loc=(goal.target_pose.pose.position.x, 
                                                      goal.target_pose.pose.position.y, 
                                                      goal.target_pose.pose.position.z,
                                                      goal.target_pose.pose.orientation.x, 
                                                      goal.target_pose.pose.orientation.y, 
                                                      goal.target_pose.pose.orientation.z, 
                                                      goal.target_pose.pose.orientation.w))
                self.goal_marker_pub.publish(marker)

                print "Sending goal"
                self.move_client.send_goal(goal)

                self.move_client.wait_for_result(rospy.Duration.from_sec(self.goal_wait_time)) # blocking call

                print "Goal result received"
                print self.move_client.get_state()    
                # could check to make sure its not a failure nor a success
                # terminal_states = [GoalStatus.REJECTED, GoalStatus.ABORTED, GoalStatus.SUCCEEDED, GoalStatus.RECALLED]
                
                # # print "Terminal States: ", terminal_states
                # movement_state = self.client.get_state()

                # # print "Condition!!", self.condition

                # # print terminal_states
                # while movement_state not in terminal_states:
                #     movement_state = self.client.get_state()
                    
                #     if self.update_state() != Driver.NORMAL:
                #         break

                #     time.sleep(.1)

                # if movement_state == GoalStatus.SUCCEEDED:
                #    # get new goal
                rospy.sleep(0.3)

            rospy.sleep(0.3)



    def get_next_goal(self):
        goal_loc = self.get_greedy_goal()

        # create goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'

        goal.target_pose.pose.position.x = goal_loc[0]
        goal.target_pose.pose.position.y = goal_loc[1]
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 0.0

        goal.target_pose.header.stamp = rospy.Time.now()

        return goal

    def get_greedy_goal(self):    
        '''
        Does BFS on the map starting from the robot's current position
        As soon as dirty cell is found, that is returned
        '''
        ### May need to tf pose to map frame..
        # PoseWithCovariance.pose.position
        start = (self.current_pose.position.x, self.current_pose.position.y)
        visited, queue = set(), [start]
        # while queue:
            
        #     vertex = queue.pop(0) # this is slow, could be optimized with a priority queue
            
        #     if vertex == contaminated:
        #         return vertex
            
        #     else:
        #         visited.add(vertex)

        #         # get the children of the vertex
        #         children = 0 # get 4 connected children and check to see that they aren't in visited
        #         queue.extend(children)
            
        # goal = self.current_map_pose
        goal = (self.current_pose.position.x + 0.1, self.current_pose.position.y + 0.1)
        return goal


    # def mark_bot(self, odometry, publisher):
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



    def create_goal_marker(self, loc, marker_type=Marker.SPHERE, marker_action=Marker.ADD, marker_id=0, marker_color=ColorRGBA(1,0,0,1)):
        h = Header()
        h.frame_id = "map"
        h.stamp = rospy.Time.now()        
       
        mark = Marker()
        mark.header = h
        mark.ns = "goal_marker"
        mark.id = marker_id
        mark.type = marker_type
        mark.action = marker_action
        mark.scale = Vector3(0.25, 0.25, 0.25) 
        mark.color = marker_color 

        pose = Pose(Point(loc[0], loc[1], loc[2]), Quaternion(loc[3],loc[4],loc[5],loc[6]))
        mark.pose = pose
        
        return mark


if __name__ == '__main__':
    rospy.init_node('srubber_bot_planner')

    cleaner = ScrubberBotPlanner()

    cleaner.clean()

    rospy.spin()


    
