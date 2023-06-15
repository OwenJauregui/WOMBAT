#!/usr/bin/env python

#import ros
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

#import message filters utility
import message_filters
import numpy as np
import cv2 as cv

#custom libraries
from vision_nodes.utils import Vision


def imageCallback(rgb_msg, depth_msg):

    global camera, bridge, image_pub, box_pub
    global lower, upper, detector

    img = bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
    depth = bridge.imgmsg_to_cv2(depth_msg)

    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, lower, upper)
    mask = cv.blur(mask,(7,7))
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, (7,7))
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, (7,7))
    #mask = cv.dilate(img,(5,5),iterations = 3)

    keypoints = detector.detect(mask)

    im_with_keypoints = cv.drawKeypoints(img, keypoints, np.array([]), (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    img = cv.bitwise_and(img,im_with_keypoints,mask = mask)

    #get point in 3d
    '''x =keypoints[0].pt[0]
    point = camera.map_point2d_3d((x,y), img, depth)
    
    #prepare point msg
    img_msg = Point()
    img_msg.x = point[0]
    img_msg.y = point[1]
    img_msg.z = point[2]
    box_pub.publish(img_msg)'''

    #publish image to debug
    #img_msg = bridge.cv2_to_imgmsg(mask, "mono8")
    img_msg = bridge.cv2_to_imgmsg(img, "bgr8")
    image_pub.publish(img_msg)


def main():

    global camera, bridge, image_pub, box_pub
    global lower, upper, detector

    #threshholds for yellow detection
    lower = np.array([20, 120, 120], dtype="uint8")
    upper = np.array([30, 250, 250], dtype="uint8")

    # Setup SimpleBlobDetector parameters.
    params = cv.SimpleBlobDetector_Params()

    #filter by color (only works on binary images)
    params.filterByColor = True
    params.blobColor = 255
    
    # Change thresholds
    params.minThreshold = 0
    params.maxThreshold = 150

    # Change distance between blobs in pixels
    params.minDistBetweenBlobs = 200
    
    # Filter by Area.
    params.filterByArea = False
    params.minArea = 5000
    #params.maxArea = 10000
    
    # Filter by Circularity
    params.filterByCircularity = False
    params.minCircularity = 0.8
    
    # Filter by Convexity
    params.filterByConvexity = False
    params.minConvexity = 0.87
    params.maxConvexity = 1.0
    
    # Filter by Inertia
    params.filterByInertia = False
    params.minInertiaRatio = 0.01

    # Create a detector with the parameters
    detector = cv.SimpleBlobDetector_create(params)

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
    image_pub = rospy.Publisher("/WOMBAT/vision/img", Image, queue_size=1)  
    #box_pub = rospy.Publisher("/WOMBAT/vision/box", Point, queue_size=1)

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
