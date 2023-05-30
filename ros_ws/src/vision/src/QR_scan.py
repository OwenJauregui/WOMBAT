#!/usr/bin/env python3

#import ros
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

#custom libraries
from vision_nodes.utils import Vision


def imageCallback(data):

    global image_pub, bridge, camera

    image = bridge.imgmsg_to_cv2(data, "bgr8")
    qrdata = camera.decoder(image)

    if qrdata != "X":
        #img_msg = bridge.cv2_to_imgmsg(image, "bgr8")

        image_pub.publish(qrdata)

def main():

    global image_pub, bridge, camera

    #Initialize node
    rospy.init_node("QR_scan")

    bridge = CvBridge()
    image_sub = rospy.Subscriber("/camera/color/image_raw", Image, imageCallback)
    image_pub = rospy.Publisher("/WOMBAT/vision/qr", String, queue_size=1)
    
    #calibration parameters
    #camera rgb
    k = [616.8235473632812, 616.655517578125, 317.11993408203125, 243.79525756835938]
    scale = 1.0
    d = 8

    #Create Vision instance
    camera = Vision(k[0], k[1], k[2], k[3], scale, d)

    #callback
    rospy.spin()

if __name__ == "__main__":
    main()
