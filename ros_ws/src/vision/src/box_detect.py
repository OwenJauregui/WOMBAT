#!/usr/bin/env python

#import ros
import rospy
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

#import message filters utility
import message_filters
import numpy as np
import cv2 as cv

#custom libraries
from vision_nodes.utils import Vision


def imageCallback(rgb_msg, depth_msg):

    global camera, bridge, box_pub, box_rviz
    global lower, upper

    img = bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
    depth = bridge.imgmsg_to_cv2(depth_msg)

    # Setup SimpleBlobDetector parameters.
    params = cv.SimpleBlobDetector_Params()
    
    # Change thresholds
    params.minThreshold = 10
    params.maxThreshold = 200

    # Change distance between blobs in pixels
    params.minDistBetweenBlobs = 20
    
    # Filter by Area.
    params.filterByArea = True
    params.minArea = 1500
    
    # Filter by Circularity
    params.filterByCircularity = False
    params.minCircularity = 0.1
    
    # Filter by Convexity
    params.filterByConvexity = False
    params.minConvexity = 0.87
    
    # Filter by Inertia
    params.filterByInertia = False
    params.minInertiaRatio = 0.01

    # Set up the detector with default parameters.
    detector = cv.SimpleBlobDetector(params)
    
    # Detect blobs.
    keypoints = detector.detect(img)

    mask = cv.inRange(img, lower, upper)

    cnts = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]

    for c in cnts:
        x,y,w,h = cv.boundingRect(c)
        cv.rectangle(img, (x, y), (x + w, y + h), (36,255,12), 2)
    if len(cnts) > 0:
        img_msg = bridge.cv2_to_imgmsg(img, "bgr8")
        box_pub.publish(img_msg)

    #cv.imshow('mask', mask)
    #cv.imshow('original', img)



            

def main():

    global camera, bridge, box_pub, box_rviz
    global lower, upper

    #threshholds for yellow detection
    lower = np.array([22, 93, 0], dtype="uint8")
    upper = np.array([45, 255, 255], dtype="uint8")
    #calibration parameters
    #camera rgb
    k = [616.8235473632812, 616.655517578125, 317.11993408203125, 243.79525756835938]
    scale = 1.0
    d = 8
    #Create Vision instance
    camera = Vision(k[0], k[1], k[2], k[3], scale, d)
    #subscribers headers for realsense
    input_rgb = "/camera/color/image_raw"
    input_depth = "/camera/aligned_depth_to_color/image_raw"

    #Initialize node
    rospy.init_node("box_scanner")
    bridge = CvBridge()

    #PUBLISHERS
    #qr publisher
    box_pub = rospy.Publisher("/WOMBAT/vision/box", Image, queue_size=1)  
    box_rviz = rospy.Publisher("/WOMBAT/vision/rviz_box", PointStamped, queue_size=1)

    #SUBSCRIBERS
    #camera subscriber
    rgb_sub = message_filters.Subscriber(input_rgb, Image)
    depth_sub = message_filters.Subscriber(input_depth, Image)

    #create time sync
    ts =  message_filters.TimeSynchronizer([rgb_sub, depth_sub], queue_size=1)

    #register callback
    ts.registerCallback(imageCallback) 

    #callback
    rospy.spin()

if __name__ == "__main__":
    main()
