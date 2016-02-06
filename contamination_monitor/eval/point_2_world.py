#!/usr/bin/env python
import argparse
import imutils
import cv2
import cv2.cv as cv
import numpy as np

import rospy
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import OccupancyGrid, MapMetaData
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, PointStamped
from image_geometry import PinholeCameraModel
from visualization_msgs.msg import Marker
import tf
from std_msgs.msg import *

import sys
import cv2
import yaml
import array

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

    @staticmethod
    def get_plane_normal(p1, p2, p3):
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

    @staticmethod
    def get_line_normal(p1, p2):
        ''' Computes the normal vector for the plane defined by the 2 given points. 
        '''
        # dx=x2-x1 and dy=y2-y1, then the normals are (-dy, dx) and (dy, -dx)
        p1 = np.array(p1)
        p2 = np.array(p2)
        ln = p2 - p1
        return ln

    @staticmethod
    def line_plane_intersection(lp, ln, pp, pn):
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

    @staticmethod
    def create_line_marker(p1, p2, frame_id):
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

    @staticmethod
    def create_point_marker(p1, frame_id):
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

    @staticmethod
    def create_points_marker(points, frame_id):
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

    @staticmethod
    def get_matrices(camera_frame, other_frame):
        print "Getting matrices"
        tf_listener = tf.TransformListener()
        tfros = tf.TransformerROS()

        tf_listener.waitForTransform(camera_frame, other_frame, rospy.Time(0), rospy.Duration(2))
        (trans,rot) = tf_listener.lookupTransform(camera_frame, other_frame, rospy.Time(0))
        tf_mat = tfros.fromTranslationRotation(trans, rot)


        tf_listener.waitForTransform(other_frame, camera_frame, rospy.Time(0), rospy.Duration(2))
        (inv_trans,inv_rot) = tf_listener.lookupTransform(other_frame, camera_frame, rospy.Time(0))
        inv_tf_mat = tfros.fromTranslationRotation(inv_trans, inv_rot)

        return tf_mat, inv_tf_mat

    @staticmethod
    def store_all_image_info():
        image_topics = [("/image_raw", "/camera_info"), 
                        ("/wall_camera/image_raw", "/wall_camera/camera_info"), 
                        ("/door_camera/image_raw", "door_camera/camera_info")]

        for topic, info in image_topics:

            ## could have used wait for msg here
            rospy.Subscriber(info, CameraInfo, PixelConversion.store_image_info_cb, topic, queue_size = 1)
            print "subscriber setup"

    @staticmethod
    def store_image_info_cb(msg, topic_name):
        ''' Stores the topic name, pinhole camera model object, and
            matrices from the camera to the map in pickle fiels.
        '''
        import pickle
        import os
        import rospkg

        print "Storing info"

        pinhole_camera_model = PinholeCameraModel()
        pinhole_camera_model.fromCameraInfo(msg)

        matrices = PixelConversion.get_matrices(msg.header.frame_id, "/map")

        

        filename = topic_name.replace("/", "_") + ".pickle"

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('contamination_monitor')
        directory = os.path.join(pkg_path, "eval", "camera_map_tfs") 

        if not os.path.exists(directory):
            os.makedirs(directory)
        
        filepath = os.path.join(directory, filename)

        
        
        with open(filepath, 'wb+') as f:
            pickle.dump(topic_name, f)
            pickle.dump(pinhole_camera_model, f)
            pickle.dump(matrices, f)

            print "pickle dumped to ", filepath

    
    @staticmethod
    def load_all_image_info():
        import pickle
        import os
        import rospkg

        image_info = dict()

        image_topics = [("/image_raw", "/camera_info"), 
                        ("/wall_camera/image_raw", "/wall_camera/camera_info"), 
                        ("/door_camera/image_raw", "door_camera/camera_info")]

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('contamination_monitor')
        directory = os.path.join(pkg_path, "eval", "camera_map_tfs") 

        for topic, info in image_topics:
            filename = topic.replace("/", "_") + ".pickle"

            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path('contamination_monitor')
            directory = os.path.join(pkg_path, "eval", "camera_map_tfs") 

            filepath = os.path.join(directory, filename)

            with open(filepath, 'rb') as f:
                topic_name = pickle.load(f)
                pinhole_camera_model = pickle.load(f)
                matrices = pickle.load(f)


                image_info[topic_name] = (pinhole_camera_model, matrices)

                print "Info loaded from ", filepath

        return image_info

    @staticmethod
    def save_contam_grid(fp):

        ogrid = rospy.rospy.wait_for_message("contamination_grid", OccupancyGrid)

        data = dict(
            Header = ogrid.header,
            MapMetaData = ogrid.info,
            GridFile = fp + ".csv",
            )

        #### get grid data as numpy and save as seperate file       
        with open(fp + ".yaml", 'w+') as outfile:
            print "Saving to FP ", fp
            outfile.write( yaml.dump(data, default_flow_style=False) )

            np.savetxt(fp + ".csv", np.array(ogrid.data, dtype=np.int8), delimiter=",")
            
            print "Saved"
        

    @staticmethod
    def load_map(fp):
        
        ogrid = OccupancyGrid()
        step = None
        offset = None
        yaml_file = fp + ".yaml"
        csv_file = fp + ".csv"
        with open(yaml_file, "r") as infile:
            for k, v in yaml.load(infile.read()).iteritems():
                if k == "Header":
                    ogrid.header = v

                elif k == "MapMetaData":
                    ogrid.info = v
                    step = v.resolution
                    offset = (v.origin.position.x, v.origin.position.y)

                elif k == "GridFile":
                    print "GridFile: ", v
                else: 
                    print "Unexpected k : ", k
                    raise TypeError("Unexpected key type in yaml file: " + yaml_file)

            ogrid.data = list(np.loadtxt(csv_file, dtype=np.int8, delimiter=","))

        print "Map loaded"
        
        return ogrid, step, offset


    @staticmethod
    def build_image_map(map_file_path, img_fp, img_topic, save_file_prepend, save_dir):
        ''' Load the camera image info. 
            Load the images.

            For each pixel on the image that is green, 
                add it to the map.
            Overlay image onto map?
        '''
        orig_map, step, offset = PixelConversion.load_map(map_file_path)

        updated_grid = orig_map.data[:]
        resolution = orig_map.info.resolution
        offset_x = orig_map.info.origin.position.x
        offset_y = orig_map.info.origin.position.y
        height = orig_map.info.height
        width = orig_map.info.width

        image_info = PixelConversion.load_all_image_info()
       
        pinhole_camera_model, tf_matrices = image_info[img_topic]
        # for topic_name, (pinhole_camera_model, matrices) in image_info.iteritems():
        #     print "Image topic: ", topic_name

        # load image
        img = cv2.imread(img_fp)
        # Get the green parts of the image
        img, thresh_mask = PixelConversion.threshold_green(img)

        # convert image to map dimensions
        # print "matrices\n", tf_matrices
        print "Converting"

        camera_frame_map_frame_tf_mat = tf_matrices[0]
        map_frame_camera_frame_tf_mat = tf_matrices[1]

        # Define map plane
        p1 = [-94.0, -98.0, 0.0]
        p2 = [-92.0, -88.0, 0.0]
        p3 = [-84.0, -88.0, 0.0]

        # Tf to the camera frame
        # camera_frame - map frame tf
        # pts in camera frame coodrds
        p1 = list(np.dot(camera_frame_map_frame_tf_mat, np.array([p1[0], p1[1], p1[2], 1.0])))[:3]
        p2 = list(np.dot(camera_frame_map_frame_tf_mat, np.array([p2[0], p2[1], p2[2], 1.0])))[:3]
        p3 = list(np.dot(camera_frame_map_frame_tf_mat, np.array([p3[0], p3[1], p3[2], 1.0])))[:3]
        plane_norm = PixelConversion.get_plane_normal(p1, p2, p3)         
        # print "P1 ", p1
        # print "P2 ", p2
        # print "P3 ", p3
        # print "Plane norm: ", plane_norm
        # point_marker = self.create_points_marker([p1, p2, p3], img_msg.header.frame_id)
        # self.plane_pub.publish(point_marker)

        
        # Go through every pixel in the image and find where it hits in 
        # the map frame and save map to a file
        for u in xrange(img.shape[0]):
            for v in xrange(img.shape[1]):
        
                # Get the unit vector related to the pixel coords 
                unit_vector = pinhole_camera_model.projectPixelTo3dRay((u, v))
                # print "Unit vector ", unit_vector
                
                # need unit vector to define ray to cast onto plane
                line_pt = list(unit_vector)
                line_norm = PixelConversion.get_line_normal([0.0,0.0,0.0], np.asarray(line_pt) * 10) # just scale the unit vector for another point, maybe just use both unit vector for the point and the line

                inter = PixelConversion.line_plane_intersection(line_pt, line_norm, p1, plane_norm)

                ## intersection is in the camera frame so tf into map frame
                map_inter_pt = tuple(np.dot(map_frame_camera_frame_tf_mat,np.array([inter[0], inter[1], inter[2], 1.0])))[:3]
                # print "intersection pt: ", map_inter_pt

                #translate xy to cell - each cell is <resolution> meters wide
                x = int(round((map_inter_pt[0] - offset_x)/resolution))
                y = int(round((map_inter_pt[1] - offset_y)/resolution))
                cell =  y * width + x

                updated_grid[cell] = 50

        new_map_fp = save_dir + save_file_prepend + "_" + img_topic.replace("/", "_") + "_collated.pgm"  
     
        print "Writing camera coverage to file..."
        with open(new_map_fp, "wb+") as f:
            
            # define PGM Header
            pgmHeader = 'P5' + '\n' + str(width) + '  ' + str(height) + '  ' + str(255) + '\n'
            f.write(pgmHeader)

            buff = array.array('B')
            for y in xrange(0, height):
                for x in xrange(0, width):
                    i = x + (height - y - 1) * width
                    char = 205
                    if updated_grid[i] == 0:
                        char = 254
                    elif updated_grid[i] > 0.65: # makes it black
                        char = 000

                    buff.append(char)

            buff.tofile(f)

        print "File saved at ", new_map_fp  




        # Go through every pixel in the image that is green and mark that in the pgm
        # Save resulting map        
        green_points_grid = np.zeros(len(orig_map.data))
        pixels_contam = 0
        for u in xrange(thresh_mask.shape[0]):
            for v in xrange(thresh_mask.shape[1]):
                if thresh_mask[u,v] > 0:
                    pixels_contam += 1
        
                    # Get the unit vector related to the pixel coords 
                    unit_vector = pinhole_camera_model.projectPixelTo3dRay((u, v))
                    # print "Unit vector ", unit_vector
                    
                    # need unit vector to define ray to cast onto plane
                    line_pt = list(unit_vector)
                    line_norm = PixelConversion.get_line_normal([0.0,0.0,0.0], np.asarray(line_pt) * 10) # just scale the unit vector for another point, maybe just use both unit vector for the point and the line

                    inter = PixelConversion.line_plane_intersection(line_pt, line_norm, p1, plane_norm)

                    ## intersection is in the camera frame so tf into map frame
                    map_inter_pt = tuple(np.dot(map_frame_camera_frame_tf_mat,np.array([inter[0], inter[1], inter[2], 1.0])))[:3]
                    # print "intersection pt: ", map_inter_pt

                    #translate xy to cell - each cell is <resolution> meters wide
                    x = int(round((map_inter_pt[0] - offset_x)/resolution))
                    y = int(round((map_inter_pt[1] - offset_y)/resolution))
                    cell =  y * width + x

                    green_points_grid[cell] = 100

        print "pixels contaminated: ", pixels_contam
        new_map_fp = save_dir + save_file_prepend + "_" + img_topic.replace("/", "_") + "_contaminated_map.pgm"  
     
        ## Save the map to pgm image
        ## Contaminated areas as black and the rest as light grey
        print "Writing real contam to file..."
        with open(new_map_fp, "wb+") as f:
            
            # define PGM Header
            pgmHeader = 'P5' + '\n' + str(width) + '  ' + str(height) + '  ' + str(255) + '\n'
            f.write(pgmHeader)

            buff = array.array('B')
            for y in xrange(0, height):
                for x in xrange(0, width):
                    i = x + (height - y - 1) * width
                    char = 205
                    # if green_points_grid[i] == 0:
                    #     char = 254
                    if green_points_grid[i] > 0.65: # makes it black
                        char = 000

                    buff.append(char)

            buff.tofile(f)

        print "File saved at ", new_map_fp  



        ### Create an overlaid image of the real versus perceived
        ## Save it to ppm style map
        ## Contaminated areas as black and the rest as light grey
        print "Writing overlay to file..."
        new_map_fp = save_dir + save_file_prepend + "_" + img_topic.replace("/", "_") + "_contaminated_overlay_map.pgm"  
        with open(new_map_fp, "wb+") as f:
            
            # define PGM Header
            pgmHeader = 'P5' + '\n' + str(width) + '  ' + str(height) + '  ' + str(255) + '\n'
            f.write(pgmHeader)

            buff = array.array('B')
            for y in xrange(0, height):
                for x in xrange(0, width):
                    i = x + (height - y - 1) * width
                    char = 205 
                    if green_points_grid[i] > 0.65: 
                        char = 254 # confirmed contam areas now white
                    elif orig_map.data[i] > 0.65:
                        char = 000 # simulated contam areas now black
                    buff.append(char)

            buff.tofile(f)

        print "File saved at ", new_map_fp  


        ## Lastly save the original map it was built from
        print "Writing original map to file..."
        new_map_fp = save_dir + save_file_prepend + "_" + img_topic.replace("/", "_") + "orig_map.pgm"  
        with open(new_map_fp, "wb+") as f:
            
            # define PGM Header
            pgmHeader = 'P5' + '\n' + str(width) + '  ' + str(height) + '  ' + str(255) + '\n'
            f.write(pgmHeader)

            buff = array.array('B')
            for y in xrange(0, height):
                for x in xrange(0, width):
                    i = x + (height - y - 1) * width
                    char = 205
                    if orig_map.data[i] == 0:
                        char = 254
                    elif orig_map.data[i] > 65: # makes it black
                        char = 000

                    buff.append(char)

            buff.tofile(f)

        print "File saved at ", new_map_fp  



    @staticmethod
    def threshold_green(img):    
        ''' Takes the original image and returns a masked image of only the green parts.

        Returns the threshed image and the thresholding mask.

        Alg:
        1) Blur image
        2) Convert to hsv color space
        3) Define hsv limits
        4) Threshold the image with those hsv limits
        5) Mask the original image with the thresholded image as the mask

        ''' 
        blur_img = cv2.GaussianBlur(img,(9,9),0) 
        img_hsv = cv2.cvtColor(blur_img, cv2.COLOR_BGR2HSV)

        hsv_min = np.asarray([20, 24, 5])
        hsv_max = np.asarray([100, 255, 255])
        threshed = cv2.inRange(img_hsv, hsv_min, hsv_max)
        # cv2.imshow("Orig & Thresholded - 2", np.hstack([img, threshed]))
        # print threshed.shape
        # cv2.imshow("threshed - 2", threshed)

        ## now mask the image to only leave the green
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(blur_img,img, mask = threshed)
        cv2.imshow("Orig, Blurred, Masked", np.hstack([img, blur_img, res]))

        cv2.waitKey(0)
        cv2.destroyAllWindows()

        return res, threshed
        
    


if __name__ == '__main__':
    rospy.init_node("image_2_world")

    # pixel_conversion = PixelConversion() # Allows user to click on image in rviz with publish point tool and it shows the intersection 
    # PixelConversion.store_all_image_info() 
    # PixelConversion.get_all_image_info()
    # map_file_path = raw_input("Map File Path?\n")
    map_fp = "/nfs/attic/smartw/users/kraftko/Bagfiles/contam_eval/tuesday_02022016/maps/kyle_after_run_5"
    img_fp = "/nfs/attic/smartw/users/kraftko/Bagfiles/contam_eval/tuesday_02022016/trial_5/kyle_single_05_blacklight_imgs_after_door_camera.jpg"
    save_file_prepend = "kyle_single_05"
    save_dir = "/nfs/attic/smartw/users/kraftko/Bagfiles/contam_eval/tuesday_02022016/results/"

    img_topic = "/image_raw"    
    if "wall_camera" in img_fp:
        img_topic = "/wall_camera/image_raw"
    elif "door_camera" in img_fp:
        img_topic = "/door_camera/image_raw"

    PixelConversion.build_image_map(map_fp, img_fp, img_topic, save_file_prepend, save_dir)
    # PixelConversion.color_match_image()
          
    # rospy.spin()