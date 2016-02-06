#!/usr/bin/env python
import argparse
import imutils
import cv2
import cv2.cv as cv
import numpy as np

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from image_geometry import PinholeCameraModel
import tf

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

class GridToImageConverter():
    def __init__(self):
        self.image_topic = "wall_camera/image_raw"
        self.image_info_topic = "wall_camera/camera_info"
        self.point_topic = "/clicked_point"
        self.frame_id = "/map"
        self.occupancy 

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
                self.tf_listener.waitForTransform(self.frame_id, img_msg.header.frame_id, rospy.Time(0), rospy.Duration(0.5))
                (trans,rot) = self.tf_listener.lookupTransform(self.frame_id, img_msg.header.frame_id, rospy.Time(0))
                self.tfros = tf.TransformerROS()
                tf_mat = self.tfros.fromTranslationRotation(trans, rot)
                xyz = tuple(np.dot(tf_mat, np.array([unit_vector[0], unit_vector[1], unit_vector[2], 1.0])))[:3]

                print "World xyz ", xyz 

                red = (0,125,255)
                cv2.circle(self.img, (int(pixel_coords[0]), int(pixel_coords[1])), 30, red)
                cv2.imshow(self.image_topic, self.img)


            self.ready_for_image = False
            self.points = []
            
            # Process any keyboard commands
            self.keystroke = cv.WaitKey(5)
            if 32 <= self.keystroke and self.keystroke < 128:
                cc = chr(self.keystroke).lower()
                if cc == 'q':
                    # The user has press the q key, so exit
                    rospy.signal_shutdown("User hit q key to quit.")

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


if __name__ == '__main__':
    rospy.init_node("occ_grid_to_img_pixels")

    pixel_conversion = GridToImageConverter()
          
    rospy.spin()