#!/usr/bin/env python

#Importar rospy
import rospy

#Librerias basicas
import numpy as np

#custom libraries
from vision_nodes.utils import OccGrid

#import msgs
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan
from custom_msgs.msg import GridMap, Cell

#import sync msgs
import message_filters

#Subscriber
def LaserCallback(msg_odom, msg_laser):

    global grid, marker_pub, grid_pub

    #bot pose
    x = msg_odom.x
    y = msg_odom.y
    theta =msg_odom.theta

    #construct position state
    state_odom = [x, y, theta]
    #print("Oddometry" + str(state_odom))

    #extract laser info
    laser_ranges = msg_laser.ranges
    laser_angles = np.linspace(msg_laser.angle_min, msg_laser.angle_max, 11)

    #construct laser state
    state_laser = np.vstack([laser_angles, laser_ranges])
    #print("Laser: " + str(state_laser))

    #print("==============")

    #update occupacy map
    grid.update_laser(state_odom, state_laser)

    #get occupacy map
    occ_map = grid.get_map()

    #get RVIZ message
    occ_rviz = grid.get_rviz(occ_map, rospy.Time.now())

    #publish msg
    marker_pub.publish(occ_rviz)    

    #convert to occupacy grid
    grid_msg = grid.map_msg(occ_map, rospy.Time.now())

    #publish msg
    grid_pub.publish(grid_msg)

def main():

    global grid, marker_pub, grid_pub
    global display, rviz

    #Inicializar nodo
    rospy.init_node("occupacy_grid")

    #Read point cloyd
    input_odom = "/WOMBAT/navegation/pose"
    input_laser = rospy.get_param("/occupacy_grid/topics/laser", "/scan")

    #grid parameters
    x_grid = rospy.get_param("/occupacy_grid/grid/x", 10)
    y_grid = rospy.get_param("/occupacy_grid/grid/y", 10)
    d_grid = rospy.get_param("/occupacy_grid/grid/dim", 1.0)

    #algorythm parameters
    p_thresh = rospy.get_param("/occupacy_grid/params/p_thresh", 0.8)
    p_occ = rospy.get_param("/occupacy_grid/params/p_occ", 0.66)
    p_free = rospy.get_param("/occupacy_grid/params/p_free", 0.34)
    z_min = rospy.get_param("/occupacy_grid/params/z_min", 0.0)
    z_max = rospy.get_param("/occupacy_grid/params/z_max", 0.5)
    z_floor = rospy.get_param("/occupacy_grid/params/z_floor", -0.8)

    #interface config
    display = rospy.get_param("occupacy_grid/interface/display", True)
    rviz = rospy.get_param("occupacy_grid/interface/rviz", True)

    #Publisher
    marker_pub = rospy.Publisher("/occupacy_grid/rviz_grid", MarkerArray, queue_size=1)
    grid_pub = rospy.Publisher("/occupacy_grid/grid_map", GridMap, queue_size=1)

    #create instance 
    grid = OccGrid(x_g=x_grid, y_g=y_grid, d_g=d_grid)

    #Subscriber
    odom_sub = message_filters.Subscriber(input_odom, Pose2D)
    laser_sub = message_filters.Subscriber(input_laser, LaserScan)

    #sync messages
    ts = message_filters.TimeSynchronizer([odom_sub, laser_sub], queue_size=1)

    #callback
    ts.registerCallback(LaserCallback)


    #callback
    rospy.spin()

if __name__ == '__main__':
    main()