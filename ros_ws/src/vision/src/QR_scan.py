#!/usr/bin/env python

#import ros
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
from visualization_msgs.msg import Marker
#import message filters utility
import message_filters

#custom libraries
from vision_nodes.utils import Vision


def imageCallback(rgb_msg, depth_msg):

    global qr_pub, bridge, camera

    img = bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
    depth = bridge.imgmsg_to_cv2(depth_msg)

    id, pos = camera.decoder(img)

    if id != "X":
        #img_msg = bridge.cv2_to_imgmsg(image, "bgr8")
        point = camera.map_point2d_3d(pos, img, depth)

        if len(point) != 0:

            #prepare msg to esp32
            qrdata = String() 
            qrdata.data = id

            qr_pub.publish(qrdata)

            #prepare marker to map

            
def main():

    global qr_pub, bridge, camera

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
    rospy.init_node("QR_scan")
    bridge = CvBridge()

    #PUBLISHERS
    #qr publisher
    qr_pub = rospy.Publisher("/WOMBAT/vision/qr", String, queue_size=1)  
    qr_rviz = rospy.Publisher("/WOMBAT/vision/box", Marker, queue_size=1)

    #SUBSCRIBERS
    #camera subscriber
    #create subscribers by topic
    rgb_sub = message_filters.Subscriber(input_rgb, Image)
    depth_sub = message_filters.Subscriber(input_depth, Image)

    #create time sync
    ts =  message_filters.TimeSynchronizer([rgb_sub, depth_sub], queue_size=1)

    #print("msgs synch")
    #register callback
    ts.registerCallback(imageCallback) 

    #callback
    rospy.spin()

if __name__ == "__main__":
    main()
