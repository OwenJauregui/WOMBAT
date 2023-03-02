#!/usr/bin/env python

#import ros
import rospy

from custom_msgs.msg import Path
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid

from vision_nodes.trajectory_algs import RRT
from vision_nodes.trajectory_algs import map2obs, array2rviz, check_trajectory

import numpy as np

#callback
def mapCallback(map_msg):
	global obs_pub, traj_pub, path_pub
	global traj_gen, recalc_trajectory, Et
	global p_start, p_end, sec
	
	#print("received map")
	obs = map2obs(map_msg, 80)
	
	if recalc_trajectory:
		#params path
		start = np.array([[p_start[0], p_start[1]]])
		target = np.array([[p_end[0], p_end[1]]])
		#calculate path
		Et = traj_gen.gen_traj(start, target, np.array(obs)[:, 0, 0:2])
		
		if Et.shape[0] > 0:
			Et = traj_gen.simplify_trajectory(Et, np.array(obs)[:, 0, 0:2])
		
			#print(Et.shape)
			recalc_trajectory = False
	else:
		#verify path without collisions
		coll = check_trajectory(Et, obs, sec)
		
		if coll:
			recalc_trajectory = True
	
	obs_rviz = array2rviz(obs, 7, [0.5, 0.5, 0.5], [0.0, 1.0, 0.0, 0.7], rospy.Time.now())
	path_rviz = array2rviz(Et, 4, [0.1, 0.0, 0.0], [1.0, 1.0, 1.0, 0.7], rospy.Time.now())
	
	#publish trajectory
	path_msg = Path()
	path_msg.header.stamp = rospy.Time.now()
	path_msg.path = path_rviz.points
	
	#publish msg
	obs_pub.publish(obs_rviz)
	
	#publish path on rviz
	traj_pub.publish(path_rviz)
	
	#publish path msg
	path_pub.publish(path_msg)


def main():

	global traj_pub, obs_pub, path_pub
	global traj_gen, recalc_trajectory, Et
	global p_start, p_end, sec
	
	#inicialize node
	rospy.init_node("trajectory_gen")
	
	#create instance publisher
	traj_pub = rospy.Publisher("/trajectory_gen/path", Marker, queue_size=1)
	obs_pub = rospy.Publisher("/trajectory_gen/obs", Marker, queue_size=1)
	path_pub = rospy.Publisher("/trajectory_gen/path_msg", Path, queue_size=1)
	
	#RRT instance
	map_topic = rospy.get_param("/trajectory_rrt/topics/input_map", "/occupacy_grid/grid_map")
	d = rospy.get_param("/trajectory_rrt/rrt/d", 0.01)
	x_range = rospy.get_param("/trajectory_rrt/rrt/x_range", [-0.5, 1.0])
	y_range = rospy.get_param("/trajectory_rrt/rrt/y_range", [-0.5, 0.5])
	sec = rospy.get_param("/trajectory_rrt/rrt/sec", 0.05)
	it = rospy.get_param("/trajectory_rrt/rrt/it", 100)
	
	#trajectory params
	p_start = rospy.get_param("/trajectory_rrt/path/start", [0.0,0.0])
	p_end = rospy.get_param("/trajectory_rrt/path/end", [0.2,0.3])
	
	#subscriber
	rospy.Subscriber(map_topic, OccupancyGrid, mapCallback)
	
	traj_gen = RRT(d , x_range, y_range, sec, it)
	
	recalc_trajectory = True
	
	#initalize timer
	rate = rospy.Rate(15)
	
	rospy.spin()
	
	
if __name__ == '__main__':
	main()
	
