#!/usr/bin/env python
import argparse
import imutils
import cv2
import cv2.cv as cv
import numpy as np

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, PointStamped
from image_geometry import PinholeCameraModel
from visualization_msgs.msg import Marker
import tf
from std_msgs.msg import *

import sys
import cv2

""" cv_bridge_demo.py - Version 0.1 2011-05-29

    A ROS-to-OpenCV node that uses cv_bridge to map a ROS image topic and optionally a ROS
    depth image topic to the equivalent OpenCV image stream(s).
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2011 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html

    Some code taken from the above.
      
"""


# construct the argument parse and parse the arguments
# ap = argparse.ArgumentParser()
# ap.add_argument("-f", "--first", required=True,
#   help="path to the first image")
# ap.add_argument("-s", "--second", required=True,
#   help="path to the second image")
# args = vars(ap.parse_args())

# # load the two images and resize them to have a width of 400 pixels
# # (for faster processing)
# imageA = cv2.imread(args["first"])
# imageA = imutils.resize(imageA, width=400)
 

 
# show the images
# cv2.imshow("Image A", imageA)
# cv2.imshow("Image B", imageB)
# cv2.imshow("Keypoint Matches", vis)
# cv2.imshow("Result", result)
# cv2.waitKey(0)

class PixelConversion():
    def __init__(self):
        self.image_topic = "wall_camera/image_raw"
        self.image_info_topic = "wall_camera/camera_info"
        self.point_topic = "/clicked_point"
        self.frame_id = "/map"

        self.pixel_x = 350
        self.pixel_y = 215

        # What we do during shutdown
        rospy.on_shutdown(self.cleanup)

        # Create the OpenCV display window for the RGB image
        self.cv_window_name = self.image_topic
        cv.NamedWindow(self.cv_window_name, cv.CV_WINDOW_NORMAL)
        cv.MoveWindow(self.cv_window_name, 25, 75)

        # Create the cv_bridge object
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size = 1)
        self.image_info_sub = rospy.Subscriber(self.image_info_topic, CameraInfo, self.image_info_cb, queue_size = 1)
        self.point_sub = rospy.Subscriber(self.point_topic, PointStamped, self.point_cb, queue_size = 4)

        self.line_pub = rospy.Publisher("line", Marker, queue_size = 10)
        self.inter_pub = rospy.Publisher("inter", Marker, queue_size = 10)
        self.plane_pub = rospy.Publisher("plane", Marker, queue_size = 10)

        self.points = []
        self.ready_for_image = False
        self.has_pic = False
        self.camera_info_msg = None

        self.tf_listener = tf.TransformListener()

        # self.display_picture()
        self.camera_model = PinholeCameraModel()
        

    def point_cb(self, point_msg):
        if len(self.points) < 4:
            self.points.append(point_msg.point)

        if len(self.points) == 1:
            self.ready_for_image = True 
            print "Point stored from click: ", point_msg.point


    def image_cb(self, img_msg):
        if self.ready_for_image:
            # Use cv_bridge() to convert the ROS image to OpenCV format
            try:
                frame = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            except CvBridgeError, e:
                print e
            
            # Convert the image to a Numpy array since most cv2 functions
            # require Numpy arrays.
            frame = np.array(frame, dtype=np.uint8)
            print "Got array"
            
            # Process the frame using the process_image() function
            #display_image = self.process_image(frame)
                           
            # Display the image.
            self.img = frame 
            # self.has_pic = True
            

            if self.camera_info_msg is not None:
                print "cx ", self.camera_model.cx()
                print "cy ", self.camera_model.cy()
                print "fx ", self.camera_model.fx()
                print "fy ", self.camera_model.fy()

                # Get the point from the clicked message and transform it into the camera frame                
                self.tf_listener.waitForTransform(img_msg.header.frame_id, self.frame_id, rospy.Time(0), rospy.Duration(0.5))
                (trans,rot) = self.tf_listener.lookupTransform(img_msg.header.frame_id, self.frame_id, rospy.Time(0))
                self.tfros = tf.TransformerROS()
                tf_mat = self.tfros.fromTranslationRotation(trans, rot)
                point_in_camera_frame = tuple(np.dot(tf_mat, np.array([self.points[0].x, self.points[0].y, self.points[0].z, 1.0])))[:3]
                # Get the pixels related to the clicked point (the point must be in the camera frame)
                pixel_coords = self.camera_model.project3dToPixel(point_in_camera_frame)
                print "Pixel coords ", pixel_coords

                # Get the unit vector related to the pixel coords 
                unit_vector = self.camera_model.projectPixelTo3dRay((int(pixel_coords[0]), int(pixel_coords[1])))
                print "Unit vector ", unit_vector

                # TF the unit vector of the pixel to the frame of the clicked point from the map frame
                # self.tf_listener.waitForTransform(self.frame_id, img_msg.header.frame_id, rospy.Time(0), rospy.Duration(0.5))
                # (trans,rot) = self.tf_listener.lookupTransform(self.frame_id, img_msg.header.frame_id, rospy.Time(0))
                # self.tfros = tf.TransformerROS()
                # tf_mat = self.tfros.fromTranslationRotation(trans, rot)
                # xyz = tuple(np.dot(tf_mat, np.array([unit_vector[0], unit_vector[1], unit_vector[2], 1.0])))[:3]

                # print "World xyz ", xyz 

                # intersect the unit vector with the ground plane 

                red = (0,125,255)
                cv2.circle(self.img, (int(pixel_coords[0]), int(pixel_coords[1])), 30, red)
                cv2.imshow(self.image_topic, self.img)


                ''' trying better calculations for going from pixel to world '''

                # need plane of the map in camera frame points
                # (0, 0, 0), (1, 1, 0), (-1, -1, 0) = map plane in map frame coordinates
                p1 = [-94.0, -98.0, 0.0]
                p2 = [-92.0, -88.0, 0.0]
                p3 = [-84.0, -88.0, 0.0]
                # p2 = [1.0, -1.0, 0.0]
                # p3 = [-1.0, -1.0, 0.0]
                # point_marker = self.create_points_marker([p1, p2, p3], "/map")
                # self.plane_pub.publish(point_marker)

                self.tf_listener.waitForTransform(img_msg.header.frame_id, self.frame_id, rospy.Time(0), rospy.Duration(0.5))
                (trans,rot) = self.tf_listener.lookupTransform(img_msg.header.frame_id, self.frame_id,  rospy.Time(0))
                self.tfros = tf.TransformerROS()
                tf_mat = self.tfros.fromTranslationRotation(trans, rot)

                # pts in camera frame coodrds
                p1 = list(np.dot(tf_mat, np.array([p1[0], p1[1], p1[2], 1.0])))[:3]
                p2 = list(np.dot(tf_mat, np.array([p2[0], p2[1], p2[2], 1.0])))[:3]
                p3 = list(np.dot(tf_mat, np.array([p3[0], p3[1], p3[2], 1.0])))[:3]
                plane_norm = self.get_plane_normal(p1, p2, p3) ## maybe viz
                print "P1 ", p1
                print "P2 ", p2
                print "P3 ", p3
                print "Plane norm: ", plane_norm
                point_marker = self.create_points_marker([p1, p2, p3], img_msg.header.frame_id)
                self.plane_pub.publish(point_marker)

                
                # need unit vector to define ray to cast onto plane
                line_pt = list(unit_vector)
                line_norm = self.get_line_normal([0.0,0.0,0.0], np.asarray(line_pt) * 10) # just scale the unit vector for another point, maybe just use both unit vector for the point and the line

                inter = self.line_plane_intersection(line_pt, line_norm, p1, plane_norm)

                ## intersection is in the camera frame...
                ### tf this into map frame
                self.tf_listener.waitForTransform(self.frame_id, img_msg.header.frame_id, rospy.Time(0), rospy.Duration(0.5))
                (trans,rot) = self.tf_listener.lookupTransform(self.frame_id, img_msg.header.frame_id, rospy.Time(0))
                self.tfros = tf.TransformerROS()
                tf_mat = self.tfros.fromTranslationRotation(trans, rot)
                xyz = tuple(np.dot(tf_mat,np.array([inter[0], inter[1], inter[2], 1.0])))[:3]
                print "intersection pt: ", xyz
                point_marker = self.create_point_marker(xyz, "/map")
                self.inter_pub.publish(point_marker)

                line_marker = self.create_line_marker([0.0,0.0,0.0], np.asarray(unit_vector) * 30, img_msg.header.frame_id) 
                self.line_pub.publish(line_marker)



            self.ready_for_image = False
            self.points = []
            
            # Process any keyboard commands
            # self.keystroke = cv.WaitKey(5)
            # if 32 <= self.keystroke and self.keystroke < 128:
            #     cc = chr(self.keystroke).lower()
            #     if cc == 'q':
            #         # The user has press the q key, so exit
            #         rospy.signal_shutdown("User hit q key to quit.")

    def image_info_cb(self, msg):
        if self.camera_info_msg is None:
            print "Got camera info"
            self.camera_model.fromCameraInfo(msg)
            self.camera_info_msg = msg


    
    def process_image(self, frame):
        # Convert to greyscale
        grey = cv2.cvtColor(frame, cv.CV_BGR2GRAY)
        
        # Blur the image
        grey = cv2.blur(grey, (7, 7))
        
        # Compute edges using the Canny edge filter
        edges = cv2.Canny(grey, 15.0, 30.0)

        
        return edges


    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows() 

    def display_picture(self):
        while True:
            if self.has_pic:
                cv2.imshow("img", self.img)

    def get_plane_normal(self, p1, p2, p3):
        ''' Computes the normal vector for the plane defined by the 3 given points.
        '''
        # p1 = np.array([1, 2, 3])
        # p2 = np.array([4, 6, 9])
        # p3 = np.array([12, 11, 9])
        p1 = np.array(p1)
        p2 = np.array(p2)
        p3 = np.array(p3)

        # These two vectors are in the plane
        v1 = p3 - p1
        v2 = p2 - p1

        # the cross product is a vector normal to the plane
        norm = np.cross(v1, v2)
        return norm

    def get_line_normal(self, p1, p2):
        ''' Computes the normal vector for the plane defined by the 2 given points. 
        '''
        # dx=x2-x1 and dy=y2-y1, then the normals are (-dy, dx) and (dy, -dx)
        p1 = np.array(p1)
        p2 = np.array(p2)
        ln = p2 - p1
        return ln

    def line_plane_intersection(self, lp, ln, pp, pn):
        '''
        computes the point intersection of the line and plane from the point/normal pairs
        lp,ln - parameterized form of the line (point, dir) p = s*ln + lp
        s = (pp - lp).pn / (ln.pn)
        return: (intersection)

        From https://github.com/brindza/IUCS/blob/33cbc0c0ef5d15b5ef8124d22cb7f5a2d8a8faad/ros/python/MathUtil.py
        '''
        lp = np.array(lp)
        ln = np.array(ln)
        pp = np.array(pp)
        pn = np.array(pn)

        num = np.dot((pp - lp), pn)
        dem = np.dot(ln, pn)

        if (np.abs(dem) < 1e-10):
            if (np.abs(num) < 1e-10):
              # line is on the plane
              return (pp)
            else:
              # they are parallel
              print "Parallel"
              return (pp)

        return (lp + (num/dem)*ln)

    def create_line_marker(self, p1, p2, frame_id):
        h = Header()
        h.frame_id = frame_id #tie marker visualization to laser it comes from
        h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
        
        #create marker:person_marker, modify a red cylinder, last indefinitely
        mark = Marker()
        mark.header = h
        mark.ns = "unit_vector"
        mark.id = 0
        mark.type = Marker.LINE_STRIP
        mark.action = 0
        mark.scale.x = 0.2 
        mark.color = color = ColorRGBA(0.2, 0.5, 0.7, 1.0)
        mark.text = "unit_vector"

        points = []
        pt1 = Point(p1[0], p1[1], p1[2])
        pt2 = Point(p2[0], p2[1], p2[2])
        # print "Pt 1 ", pt1
        # print "Pt 2 ", pt2
        points.append(pt1)
        points.append(pt2)
        mark.points = points

        return mark

    def create_point_marker(self, p1, frame_id):
        h = Header()
        h.frame_id = frame_id #tie marker visualization to laser it comes from
        h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
        
        #create marker:person_marker, modify a red cylinder, last indefinitely
        mark = Marker()
        mark.header = h
        mark.ns = "inter"
        mark.id = 0
        mark.type = Marker.POINTS
        mark.action = 0
        mark.scale.x = 1.5
        mark.scale.y = 1.5
        mark.color = color = ColorRGBA(0.1, 0.1, 0.7, 1.0)
        mark.text = "inter"

        points = []
        pt1 = Point(p1[0], p1[1], p1[2])
        # print "Pt 1 ", pt1
        points.append(pt1)
        mark.points = points

        return mark

    def create_points_marker(self, points, frame_id):
        h = Header()
        h.frame_id = frame_id #tie marker visualization to laser it comes from
        h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
        
        #create marker:person_marker, modify a red cylinder, last indefinitely
        mark = Marker()
        mark.header = h
        mark.ns = "inter"
        mark.id = 10
        mark.type = Marker.POINTS
        mark.action = 0
        mark.scale.x = 0.9
        mark.scale.y = 0.9
        mark.color = color = ColorRGBA(0.9, 0.9, 0.9, 1.0)
        mark.text = "inter"

        print "points in marker", points
        points = [Point(x[0], x[1], x[2]) for x in points]
        mark.points = points

        return mark

    def get_matrixes(self, camera_frame, other_frame):
        self.tf_listener.waitForTransform(camera_frame, other_frame, rospy.Time(0), rospy.Duration(0.5))
        (trans,rot) = self.tf_listener.lookupTransform(camera_frame, other_frame, rospy.Time(0))
        self.tfros = tf.TransformerROS()
        tf_mat = self.tfros.fromTranslationRotation(trans, rot)


        self.tf_listener.waitForTransform(other_frame, camera_frame, rospy.Time(0), rospy.Duration(0.5))
        (inv_trans,inv_rot) = self.tf_listener.lookupTransform(other_frame, camera_frame, rospy.Time(0))
        self.tfros = tf.TransformerROS()
        inv_tf_mat = self.tfros.fromTranslationRotation(inv_trans, inv_rot)

        return tf_mat, inv_tf_mat


if __name__ == '__main__':
    rospy.init_node("image_2_world")

    pixel_conversion = PixelConversion()
          
    rospy.spin()