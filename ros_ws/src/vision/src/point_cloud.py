#Importar rospy
import rospy

#Librerias basicas
import numpy as np
import cv2

#Librerias custom
from vision_nodes.utils import Vision

#Importar mensajes
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import Image

#import cvBridge
from cv_bridge import CvBridge

#import message filters utility
import message_filters

#create callback
def ImageDepthCallback(rgb_msg, depth_msg):

    global camera, cvi, p_pub

    #print("I get sinchronized RGB and Depth")

    #convert image topics on a np
    img = cvi.imgmsg_to_cv2(rgb_msg, "bgr8")
    depth = cvi.imgmsg_to_cv2(depth_msg)

    #show topics
    #cv2.imshow("img", img)
    #cv2.imshow("depth", depth)
    #cv2.waitKey(10)

    #calculate point cloud
    p_cloud = camera.point_cloud(img, depth)
    #print(len(p_cloud))

    #point cloud msg
    p_cloud_msg = Vision.plcloud2rviz(p_cloud, "world")

    #convert point cloud to RViz msg
    p_cloud_msg.header.stamp = rospy.Time.now()
    #print(p_cloud_msg.data)

    #pub msg
    p_pub.publish(p_cloud_msg)



def main():

    global camera, cvi, p_pub

    #Inicializar nodo
    rospy.init_node("pcloud_node")

    #CV bridge
    cvi = CvBridge()

    #subscribers headers for realsense
    input_rgb = "/camera/color/image_raw"
    input_depth = "/camera/depth/image_rect_raw"

    #calibration parameters
    #camera rgb
    k = [616.8235473632812, 616.655517578125, 317.11993408203125, 243.79525756835938]
    #depth camera
    #k = [386.8326416015625, 386.8326416015625, 322.6801452636719, 236.37200927734375]

    scale = 1.0
    d = 8

    #Crear instancia de vision
    camera = Vision(k[0], k[1], k[2], k[3], scale, d)

    #create publisher
    p_pub = rospy.Publisher("/WOMBAT/vision/pcloud", PointCloud2, queue_size=1)
    
    #print("publishing pcloud")

    #create subscribers by topic
    rgb_sub = message_filters.Subscriber(input_rgb, Image)
    depth_sub = message_filters.Subscriber(input_depth, Image)

    #create time sync
    ts =  message_filters.TimeSynchronizer([rgb_sub, depth_sub], queue_size=1)

    #print("msgs synch")
    #register callback
    ts.registerCallback(ImageDepthCallback)

    #callback
    rospy.spin()

if __name__ == '__main__':
    main()