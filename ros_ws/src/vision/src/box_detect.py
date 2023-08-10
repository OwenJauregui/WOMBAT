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

    global camera, bridge, box_pub, box_pos
    global lower, upper

    img = bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
    depth = bridge.imgmsg_to_cv2(depth_msg)

    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, lower, upper)
    #mask = cv.blur(mask,(7,7))
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, (7,7))
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, (7,7))
    #mask = cv.dilate(img,(5,5),iterations = 3)

    cnts = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]

    if len(cnts) > 0:
        
        #get color filter contours
        for c in cnts:
            x,y,w,h = cv.boundingRect(c)
            #cv.rectangle(img, (x, y), (x + w, y + h), (36,255,12), 2)
        
        # converting image into grayscale image
        #gray_mask = cv.cvtColor(mask, cv.COLOR_BGR2GRAY)

        # setting threshold of gray image
        _, threshold = cv.threshold(mask, 127, 255, cv.THRESH_BINARY)
        
        # using a findContours() function
        contours, _ = cv.findContours(threshold, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        
        i = 0
        
        # list for storing names of shapes
        for contour in contours:
        
            # here we are ignoring first counter because 
            # findcontour function detects whole image as shapes
            if i == 0:
                i = 1
                continue
        
            # cv.approxPloyDP() function to approximate the shape
            approx = cv.approxPolyDP(contour, 0.2 * cv.arcLength(contour, True), True)
            hull = cv.convexHull(contour)
            #print(hull.shape)
            
            # using drawContours() function
            cv.drawContours(img, [contour], 0, (0, 0, 255), 5)
        
            # finding center point of shape
            M = cv.moments(contour)
            if M['m00'] != 0.0:
                x = int(M['m10']/M['m00'])
                y = int(M['m01']/M['m00'])
            
            if len(approx) >= 3 and len(hull) < 7:

                cv.putText(img, 'Box', (x, y), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                #get point in 3d
                '''point = camera.map_point2d_3d((x,y), img, depth)
                
                #prepare point msg
                img_msg = Point()
                img_msg.x = point[0]
                img_msg.y = point[1]
                img_msg.z = point[2]
                box_pos.publish(img_msg)
                '''
        #publish image to debug
        img_msg = bridge.cv2_to_imgmsg(threshold, "mono8")
        #img_msg = bridge.cv2_to_imgmsg(img, "bgr8")
        box_pub.publish(img_msg)

    #cv.imshow('mask', mask)
    #cv.imshow('original', img)

            

def main():

    global camera, bridge, box_pub, box_pos
    global lower, upper

    #threshholds for yellow detection
    lower = np.array([25, 80, 120], dtype="uint8")
    upper = np.array([35, 255, 255], dtype="uint8")
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
    #box_pos = rospy.Publisher("/WOMBAT/vision/rviz_box", Point, queue_size=1)

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
