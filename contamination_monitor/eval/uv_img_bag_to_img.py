#!/usr/bin/env python
import argparse
import imutils
import cv2
import cv2.cv as cv
import numpy as np
import itertools

import rosbag
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

bag = rosbag.Bag('test.bag', 'w')

def parse():
    parser = argparse.ArgumentParser(description='Process images in bag files.')
    parser.add_argument('bagfile', type=file, 
                       help='Bag file with images')
    parser.add_argument('--n', type=int, default=10,
                       help='Number of topics desired to make median image.')
    parser.add_argument('--img_topic', type=str, default="/image_raw",
                       help='Image topic.')
    parser.add_argument('--camera_info', type=str, default="/camera_info",
                       help='Image topic.')
    parser.add_argument('--o', type=str,                   
                       help='File destination for output image.')

    args = parser.parse_args()

    print "Bagfile: ", args.bagfile
    print "Images to Combine: ", args.n
    print "Output Image File: ", args.o
    print "Image topic: ", args.img_topic
    print "Camera info: ", args.camera_info

    return args.bagfile, args.img_topic, args.camera_info, args.n, args.o
  
def main():

    bagfile, img_topic, camera_info, num_topics, outfile = parse()

    image = get_avg_image(bagfile, img_topic, camera_info, num_topics)

    if outfile == None:
        outfile = bagfile + ".jpg"

    cv2.imwrite(outfile, image, [cv2.IMWRITE_JPEG_QUALITY, 90] )
    print "Image saved"
    

    # cv2.imshow("Mean", img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

# def mean_image(images):
#     print images.shape
#     mean = np.mean(images, axis=(0))
#     print "Mean shape ", mean.shape
#     print mean
#     return mean

def get_img_from_msg(img_msg):
    # Use cv_bridge() to convert the ROS image to OpenCV format
    try:
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError, e:
        print e
    
    # Convert the image to a Numpy array since most cv2 functions
    # require Numpy arrays.
    frame = np.array(frame, dtype=np.uint8)

    return frame

def get_avg_image(bagfile, img_topic, camera_info, num_topics):
    # imgs = np.zeros(shape=(num_topics, 480,640,3), dtype=np.uint8)
    img_index = 0

    avg_img = None

    for topic, msg, t in rosbag.Bag(bagfile).read_messages():
        if topic == img_topic:
            img = get_img_from_msg(msg)
            # imgs[img_index] = img[:]
            img_index += 1

            if avg_img is None:
                avg_img = img
            else:
                avg_img = cv2.addWeighted(avg_img,0.7,img,0.5, 0)

            # cv2.imshow("orig", img)
            # cv2.imshow("avg", avg_img)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()

            if img_index == num_topics:
                break

    return avg_img





if __name__ == "__main__":
  main()