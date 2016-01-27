#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import *
from geometry_msgs.msg import Point, Quaternion, Pose, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import LaserScan
from sklearn.cluster import DBSCAN
from sklearn.neighbors import NearestNeighbors
from math import sin, cos, sqrt
from scipy.spatial.distance import pdist

from ellipse2d import Ellipse2d


from contamination_monitor.msg import PersonLocation2D, PersonLocation2DArray


class Person:
    def __init__(self, e=None, axis_a=0.9, center_a=0.1):
        if e is None:
            self.a = self.center = self.b = self.theta = None
        else:
            self.a = e.a
            self.b = e.b
            self.center = e.center
            self.theta = e.theta

        self.last_pos = None
        self.axis_alpha = axis_a
        self.center_alpha = center_a
        self.name = "unknown"

        self.contamination_level = -1
        
    def update(self, e):
        if self.a != None and self.b != None and self.center != None:
            self.center = [self.center[i]*self.center_alpha + e.center[i]*(1-self.center_alpha) for i in [0, 1]]
            self.a = self.a*self.axis_alpha + e.a*(1-self.axis_alpha)
            self.b = self.b*self.axis_alpha + e.b*(1-self.axis_alpha)
        
        else:
            self.a = e.a
            self.b = e.b
            self.center = e.center
        
        self.theta = e.theta

        print "Updated myself"
        print self

    def __str__(self):
        person_str = 'Name: {0}  \
                        \n  Contamination Level: {1}\n  Old position:{2} \n  Current Position: center: {3}, theta: {4}, a: {5}, b:{6}\
                       '.format(self.name, self.contamination_level, self.last_pos, \
                                  self.center, self.theta, self.a, self.b)
        return person_str

class Multitracker:
    def __init__ (self):

        self.people = []
        self.e_colors = []
        self.red = ColorRGBA(1, 0, 0, 1)
        self.green = ColorRGBA(0, 1, 0, 1)

        self.person_tracker = PersonTracker() ### allows us to get some methods from person tracker

        self.marker_pub = rospy.Publisher("multiperson_markers", MarkerArray, queue_size=10)
        self.people_locations_pub = rospy.Publisher("people_locations", PersonLocation2DArray, queue_size=10)

        self.scan_sub = rospy.Subscriber("filtered_scan", LaserScan, self.find_ppl_from_scan, self.marker_pub)
        # self.contam_array_sub = rospy.Subscriber("contam_array", Float32MultiArray, self.get_colors)
        
        self.filter_cmd_sub = rospy.Subscriber("update_filter_cmd", Bool, self.reset, self.marker_pub)

    def reset (self, run, pub):
        m = Marker(header=Header(stamp=rospy.Time.now(), frame_id="laser"), ns="person", id=0, type=3, action=3)
        pub.publish(MarkerArray([m]))
        self.people = []
        self.e_colors = []

    def _min_dist(self, dist, ind, sort, ellipse):
        ''' Match indices from knn algorithm to the ellipse data
        '''
        print "dist ", dist
        print "ind ", ind
        print "sort ", sort
        print "ellipse ", ellipse

        for i in xrange(len(ind[ellipse])):
            
            k = ind[ellipse, i]
            
            if k not in sort: #if the ideal position is available
                sort[k] = (ellipse, dist[ellipse, i])
                break
            
            elif sort[k][1] > dist[ellipse, i]: #if it's taken, but this ellipse is closer
                temp = sort[k][0]
                sort[k] = (ellipse, dist[ellipse, i])
                sort = self._min_dist(dist, ind, sort, temp)
                break
        
        return sort

    def match_people(self, new_ellipses):
        ''' Matches people from one timestep to another.

            People are currently represented as ellipses.
            Assumes that from one timestep to another, the
                people will only move just a little bit.

            Doesn't not gracefully handle cases when 2 people overlap closely.

        '''
        dist_threshold = 0.05 # distance 1 pt from another pt in timesteps to be considered just one pt
        ellipses = range(len(new_ellipses))

        print "\n\n\n"
        old_centers = np.asarray([e.center for e in self.people])
        print "Old c: \n", old_centers
        new_centers = np.asarray([e.center for e in new_ellipses])
        print "New c: \n", new_centers

        new_people = []
        for n_index, nc in enumerate(new_centers):

            new_is_matched = False
            for o_index, oc in enumerate(old_centers):
                print "Oc ", oc, " shape ", oc.shape
                print "Nc ", nc, " shape ", nc.shape
                print "nc type ", type(nc)

                pt_matrix = np.zeros((2,2))
                pt_matrix[[0],] = oc
                pt_matrix[[1],] = nc
                print "pt matrix: ", pt_matrix, "shape ", pt_matrix.shape
                print "nc: ", nc

                dist = pdist(pt_matrix, 'euclidean')
                print "distance ", dist
                if abs(dist) < dist_threshold:
                    # match between old timestep ellipse and the new ellipse
                    print "old person"
                    old_person = self.people[o_index]
                    print old_person
                    old_person.update(new_ellipses[n_index])
                    print "matched person!"
                    print old_person
                    new_people.append(old_person)
                    print "Found match"
                    print "length of new ppl ", len(new_people)
                    new_is_matched = True
                    break # the new one is matched, move on to another new one

            if not new_is_matched:
                # No match found, add it in
                print "no match found, adding one in"
                new_person = Person(new_ellipses[n_index])
                print new_person
                new_people.append(new_person)
                print "length of new ppl after no match ", len(new_people)

        self.people = new_people
        print "all people", len(new_people)
        print "".join([str(per) for per in self.people] )
                       
        
        '''
        ellipses = range(len(newc))
        dist = []
        ind = []
        
        try:    
            neighbors = NearestNeighbors(n_neighbors=2)
            neighbors.fit(oldc)
            print neighbors
            
            dist, ind = neighbors.kneighbors(newc) #ind matches index of oldc to match with
            print "dist: ", dist
            print "ind: ", ind #test
            # sort into dict: key=old index, value = (new index, distance)

            sort = {}
        
            for e in ellipses:
                sort = self._min_dist(dist, ind, sort, e)
            
            for old, new in sort.iteritems():
                self.people[old].update(new_ellipses[new[0]])
                ellipses.remove(new[0])
        
        except ValueError, e:
            print "In exception"
            # raise e
            pass

        #add any remaining ellipses to the list
        if ellipses:
            for e in ellipses:
                self.people.append(Person(new_ellipses[e]))
                self.e_colors.append(self.red)

        '''
        
        
        
       
        
    # def get_colors(self, contamination):
    #     data = contamination.data
    #     for c in xrange(len(data)):
    #         if data[c] < 0.5:
    #             self.e_colors[c] = self.red
    #         else:
    #             self.e_colors[c] = self.green

    #data to markers - does not link markers to past marker
    def find_ppl_from_scan(self, data, publisher):
        angle = data.angle_min
        incr = data.angle_increment
        max_range = data.range_max
        ranges = data.ranges
        points = []
        
        for r in ranges:
            #add all valid ranges to some xy range
            if r < max_range:
                points.append([cos(angle)*r, sin(angle)*r])
            angle += incr
        #eps = range, min_samples = min# of points in cluster.
        
        points = np.asarray(points)
        if len(points) > 3:
            db = DBSCAN(eps=0.5, min_samples=3).fit(points)
        else:
            return

        labels = db.labels_
        #return points, labels
        # Number of clusters in labels, ignoring noise if present.
        n_clusters = len(set(labels)) - (1 if -1 in db.labels_ else 0)
        print "Num clusters: ", n_clusters
        new_ellipses = []
        for n in xrange(n_clusters):
            xy = points[labels==n]
            e = Ellipse2d()
            e.fit(xy)
            print e

            ## check to see if its a valid Ellipse
            # if self.person_tracker.is_valid_person_ellipse(e):
            if e.is_valid():
                print "valid e : ", e
                new_ellipses.append(e)
        

        # **********************************
        # ******* troubleshooting **************
        # markers = MarkerArray()
        # for i in xrange(len(new_ellipses)):
        #     e = new_ellipses[i]

        #     # data = self.person_tracker.create_person_data(e.center[0], e.center[1], e.theta, e.a, e.b)
            
        #     m = Marker(ns="person", id=i + 100, type=3, action=0)
        #     m.header = Header(stamp=rospy.Time.now(), frame_id="laser")
        #     m.pose = Pose(Point(e.center[0], e.center[1], .5),
        #                 Quaternion(0.0,0.0,1.0,cos(e.theta/2)))
        #     m.scale = Vector3(e.a*2, e.b*2,1) #scale, in meters
        #     m.color = self.red #clean = red, infected = green

        #     markers.markers.append(m)

        # publisher.publish(markers)
        # *******************************
        


        # match new ellipses (representing ppl) to ones previously found in earlier time steps
        #
        print "New ellipses: ", new_ellipses
        people_data = PersonLocation2DArray()

        # try:
        self.match_people(new_ellipses)
        print "Num people found after matching: ", len(self.people)
        markers = MarkerArray()
        
        for i in xrange(len(self.people)):
            person = self.people[i]

            person_data = self.person_tracker.create_person_data(person.center[0], person.center[1], person.theta, person.a, person.b, "p"+ str(i))
            people_data.people_location_2d.append(person_data)

            # m = Marker(ns="person", id=i, type=3, action=0)
            # m.header = Header(stamp=rospy.Time.now(), frame_id="laser")
            # m.pose = Pose(Point(self.people[i].center[0], self.people[i].center[1], .5),
            #             Quaternion(0.0,0.0,1.0,cos(self.people[i].theta/2)))
            # m.scale = Vector3(self.people[i].a*2,self.people[i].b*2,1) #scale, in meters
            # m.color = self.green #e_colors[i] #clean = red, infected = green
            
            # markers.markers.append(m)
        
        #print len(new_ellipses)
        # publisher.publish(markers)
        
        self.people_locations_pub.publish(people_data)

        # except Exception, e:
        #     # raise e
        #     pass
        
class PersonTracker:
    def __init__(self, max_size=1.0, min_size=0.01, axis_a=0.9, center_a=0.1):
        self.max_size = max_size
        self.min_size = min_size
        self.axis_alpha = axis_a
        self.center_alpha = center_a
        self.last_a = None
        self.last_b = None
        self.last_center = None
        # self.last_x
        # self.last_y

        # self.red = ColorRGBA(1, 0, 0, 1)
        # self.green = ColorRGBA(0, 1, 0, 1)
        # self.color = self.red

        self.scan_frame_id = "laser"

        # self.person_marker_pub = rospy.Publisher("persons_marker", Marker, queue_size=10)
        # self.person_location_pub = rospy.Publisher("persons_location", PersonLocation2D, queue_size=10)
        # self.filtered_sub = rospy.Subscriber("filtered_scan", LaserScan, self.find_person_from_scan, self.person_location_pub)
        # self.updated_filter_cmd_sub = rospy.Subscriber("update_filter_cmd", Bool, self.reset)
        self.contam_colors_sub = rospy.Subscriber("contam", Float32, self.get_colors)

    def reset(self, run):
        self.last_a = None
        self.last_b = None
        self.last_center = None
        self.color = self.red

    def get_colors(self, data):
        if data.data < 0.5:
            self.color = self.red
        else:
            self.color = self.green

    #turn filtered laser data into markers
    def find_person_from_scan(self, data, pub):
        #print "fitting"
        ellipse_xy = []
        #points = [] #array to hold all points - FOR DEBUGGING

        angle = data.angle_min
        incr = data.angle_increment
        max_range = data.range_max
        ranges = data.ranges
        #polar >> cartesian

        for r in ranges:
            if r < max_range:
                ellipse_xy.append([cos(angle)*r, sin(angle)*r]) #make xy
            angle += incr

        #eliminate outlying points
        #x_avg = sum([xy[0] for xy in ellipse_xy])/len(ellipse_xy)
        #y_avg = sum([xy[1] for xy in ellipse_xy])/len(ellipse_xy)
        #for xy in ellipse_xy:
        #    if (abs(xy[0]-x_avg)<0.5 or abs(xy[1]-y_avg)<0.5):
        #        ellipse_xy.remove(xy)
        #fit ellipse to points - constrain size

        if len(ellipse_xy) > 1:
            
            ellipse = Ellipse2d()
            ellipse.fit(ellipse_xy)

            if self.is_valid_person_ellipse(ellipse, self.max_size, self.min_size): 
                #apply alpha to smooth changes over time, if old data exists
                if self.last_a != None and self.last_b != None and self.last_center != None:
                    ellipse.center = [self.last_center[i]*self.center_alpha + ellipse.center[i]*(1-self.center_alpha) for i in [0, 1]]
                    ellipse.a = self.last_a*self.axis_alpha + ellipse.a*(1-self.axis_alpha)
                    ellipse.b = self.last_b*self.axis_alpha + ellipse.b*(1-self.axis_alpha)
                
                self.last_center = ellipse.center
                self.last_b = ellipse.b
                self.last_a = ellipse.a

                # publish the marker associated with the person
                # marker = self.create_person_marker(ellipse.center[0], ellipse.center[1], ellipse.theta, ellipse.a, ellipse.b)
                # self.person_marker_pub.publish(marker)

                # publish the location of the people in the frame of the map
                person = self.create_person_data(ellipse.center[0], ellipse.center[1], ellipse.theta, ellipse.a, ellipse.b)
                pub.publish(person)
            
        
    def is_valid_person_ellipse(self, ellipse, max_size=1.0, min_size=0.01):
        # Validity is measured by it being a real ellipse, with 
        #   values in the plausible range for representing a human
        if (ellipse.is_valid() and 
            (min_size < ellipse.a < max_size) and 
            (min_size < ellipse.b < max_size)):
            print "valid person "
            return True
        else:
            print "invalid person "
            return False

    def create_person_data(self, pose_x, pose_y, pose_theta, ellipse_a, ellipse_b, name="unknown"):
        h = Header()
        h.frame_id = self.scan_frame_id
        h.stamp = rospy.Time.now()

        person = PersonLocation2D()
        person.header = h
        person.name = name
        person.pose.x = pose_x
        person.pose.y = pose_y
        person.pose.theta = pose_theta
        person.ellipse_a = ellipse_a
        person.ellipse_b = ellipse_b
        person.contamination = 0.05

        return person


if __name__ == '__main__':
    
    rospy.init_node("multiperson_tracker")

    multitracker = Multitracker()
          
    rospy.spin()