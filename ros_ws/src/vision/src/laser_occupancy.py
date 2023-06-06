#!/usr/bin/env python

#Importar rospy
import rospy

#Librerias basicas
import numpy as np

#custom libraries
from vision_nodes.utils import OccGrid
from vision_nodes.conversions import quaternion_to_theta

#import msgs
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

#import sync msgs
import message_filters

#Subscriber
def odomCallback(msg_odom):
    global state_odom

    #bot pose
    x = msg_odom.pose.position.x
    y = msg_odom.pose.position.y
    theta = quaternion_to_theta([msg_odom.pose.orientation.x,
                                 msg_odom.pose.orientation.y,
                                 msg_odom.pose.orientation.z,
                                 msg_odom.pose.orientation.w])

    #construct position state
    state_odom = [x, y, theta]


def LaserCallback(msg_laser):

    global grid, marker_pub, grid_pub, state_odom

    #extract laser info
    laser_ranges = msg_laser.ranges
    laser_angles = np.linspace(msg_laser.angle_min, msg_laser.angle_max, len(laser_ranges))

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
    grid_msg = grid.occGrid_msg(occ_map, rospy.Time.now())

    #publish msg
    grid_pub.publish(grid_msg)

def main():

    global grid, marker_pub, grid_pub, state_odom
    global display, rviz

    #Inicializar nodo
    rospy.init_node("occupacy_grid")

    #Read point cloyd
    input_odom = "/WOMBAT/navegation/pose"
    input_laser = rospy.get_param("/occupancy/topics/laser", "scan")

    #grid parameters
    x_grid = rospy.get_param("/occupancy/grid/x", 10)
    y_grid = rospy.get_param("/occupancy/grid/y", 10)
    d_grid = rospy.get_param("/occupancy/grid/dim", 1.0)

    #algorythm parameters
    p_thresh = rospy.get_param("/occupancy/params/p_thresh", 0.8)
    p_occ = rospy.get_param("/occupancy/params/p_occ", 0.66)
    p_free = rospy.get_param("/occupancy/params/p_free", 0.34)
    z_min = rospy.get_param("/occupancy/params/z_min", 0.0)
    z_max = rospy.get_param("/occupancy/params/z_max", 0.5)
    z_floor = rospy.get_param("/occupancy/params/z_floor", -0.8)

    #interface config
    display = rospy.get_param("occupancy/interface/display", True)
    rviz = rospy.get_param("occupancy/interface/rviz", True)
    
    state_odom = 0

    #Publisher
    marker_pub = rospy.Publisher("/occupancy/rviz_grid", MarkerArray, queue_size=1)
    grid_pub = rospy.Publisher("/occupancy/grid_map", OccupancyGrid, queue_size=1)

    #create instance 
    grid = OccGrid(x_g=x_grid, y_g=y_grid, d_g=d_grid)

    #Subscriber
    odom_sub = rospy.Subscriber(input_odom, PoseStamped, odomCallback, queue_size=5)
    laser_sub = rospy.Subscriber(input_laser, LaserScan, LaserCallback, queue_size=1)

    #callback
    rospy.spin()

if __name__ == '__main__':
    main()
