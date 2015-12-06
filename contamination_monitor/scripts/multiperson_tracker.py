#!/usr/bin/env python

import rospy
import numpy
from std_msgs.msg import *
from geometry_msgs.msg import Point, Quaternion, Pose, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import LaserScan
from sklearn.cluster import DBSCAN
from sklearn.neighbors import NearestNeighbors
from math import sin, cos, sqrt

from ellipse2d import Ellipse2d


from contamination_monitor.msg import PersonLocation2D, PersonLocation2DArray
from person_tracker import PersonTracker

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
        ''' Not sure what this minimum distance describes.
        '''
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
        if not self.people: #empty list
            self.people = [Person(n) for n in new_ellipses]
            self.e_colors = [self.red for n in new_ellipses]
            return
        
        oldc = numpy.asarray([e.center for e in self.people])
        #print oldc
        newc = numpy.asarray([e.center for e in new_ellipses])
        #print newc
        
        neighbors = NearestNeighbors(n_neighbors=2)
        neighbors.fit(oldc)
        print neighbors
        dist, ind = neighbors.kneighbors(newc) #ind matches index of oldc to match with
        print dist, ind #test
        # sort into dict: key=old index, value = (new index, distance)
        
        sort = {}
        ellipses = range(len(newc))
        for e in ellipses:
            sort = self._min_dist(dist, ind, sort, e)
        
        for old, new in sort.iteritems():
            self.people[old].update(new_ellipses[new[0]])
            ellipses.remove(new[0])
       
        #add any remaining ellipses to the list
        if ellipses:
            for e in ellipses:
                self.people.append(Person(new_ellipses[e]))
                self.e_colors.append(self.red)

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
        
        points = numpy.asarray(points)
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

        try:
            self.match_people(new_ellipses)
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

        except Exception, e:
            # raise e
            pass
        


if __name__ == '__main__':
    
    rospy.init_node("multiperson_tracker")

    multitracker = Multitracker()
          
    rospy.spin()