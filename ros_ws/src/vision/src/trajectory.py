#!/usr/bin/env python

#import ros
import rospy

from custom_msgs.msg import Path
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose2D

from vision_nodes.trajectory_algs import RRT
from vision_nodes.trajectory_algs import map2obs, array2rviz, check_trajectory

import numpy as np

#callback
def goalCallback(goal):
	global p_start, p_end, obs, Et

	#update new goal
	x = goal.x
	y = goal.y

	p_end = [x,y]
	newGoal = True
	#print("new goal")
	#params path
	start = np.array([[p_start[0], p_start[1]]])
	target = np.array([[p_end[0], p_end[1]]])
	#calculate path
	Et = traj_gen.gen_traj(start, target, np.array(obs)[:, 0, 0:2])
	
	if Et.shape[0] > 0:
		Et = traj_gen.simplify_trajectory(Et, np.array(obs)[:, 0, 0:2])
		#print("trajectory found")
                #verify path without collisions
                coll = check_trajectory(Et, obs, sec)

                if not coll:
                    #publish point from path
                    path_rviz = array2rviz(Et, 4, [0.1, 0.0, 0.0], [0.0, 0.0, 1.0, 0.7], rospy.Time.now())

                    #publish trajectory
                    path_msg = Path()
                    path_msg.header.stamp = rospy.Time.now()
                    path_msg.header.frame_id = "map"
                    path_msg.path = path_rviz.points
                    #print(path_msg.path)
                                        
                    #publish path on rviz
                    traj_pub.publish(path_rviz)

                    #publish path msg
                    path_pub.publish(path_msg)
                    #print("published")


def poseCallback(odom):
	global p_start
	
	#update actual bot position
	x = odom.pose.position.x
	y = odom.pose.position.y

	p_start = [x,y]
	#print("position update")

def mapCallback(map_msg):
	global obs_pub, traj_pub, path_pub
	global traj_gen, recalc_trajectory, Et, obs
	global p_start, p_end, sec
	
	#print("received map")
	#view obstacles on rviz
	obs = map2obs(map_msg, 80)
	obs_rviz = array2rviz(obs, 7, [0.1, 0.1, 0.1], [1.0, 0.0, 0.0, 0.7], rospy.Time.now())
	#publish msg
	obs_pub.publish(obs_rviz)

	#verify for a path
	if len(Et) > 0:

		#verify path without collisions
		coll = check_trajectory(Et, obs, sec)
		if coll:
			recalc_trajectory = True
			#print("collision detected")
		if recalc_trajectory:
			
			#params path
			start = np.array([[p_start[0], p_start[1]]])
			target = np.array([[p_end[0], p_end[1]]])
			#calculate path
			Et = traj_gen.gen_traj(start, target, np.array(obs)[:, 0, 0:2])
			#print(Et)
			
			if Et.shape[0] > 0:
				Et = traj_gen.simplify_trajectory(Et, np.array(obs)[:, 0, 0:2])
			
				#print(Et.shape)
				recalc_trajectory = False
				#verify path without collisions
				coll = check_trajectory(Et, obs, sec)
				if not coll:
					#publish point from path
					path_rviz = array2rviz(Et, 4, [0.1, 0.0, 0.0], [0.0, 0.0, 1.0, 0.7], rospy.Time.now())
			
					#publish trajectory
					path_msg = Path()
					path_msg.header.stamp = rospy.Time.now()
					path_msg.header.frame_id = "map"
					path_msg.path = path_rviz.points
					#print(path_msg.path)

					#publish path on rviz
					traj_pub.publish(path_rviz)
					
					#publish path msg
					path_pub.publish(path_msg)
					#print("trajectory recalculated and published")

		
def main():

	global traj_pub, obs_pub, path_pub
	global traj_gen, recalc_trajectory, Et
	global p_start, p_end, sec
	
	#inicialize node
	rospy.init_node("trajectory_gen")

	#PARAMS
	#RRT instance
	map_topic = rospy.get_param("/trajectory_rrt/topics/input_map", "/map")
	d = rospy.get_param("/trajectory_rrt/rrt/d", 0.01)
	x_range = rospy.get_param("/trajectory_rrt/rrt/x_range", [-5, 5])
	y_range = rospy.get_param("/trajectory_rrt/rrt/y_range", [-5, 5])
	sec = rospy.get_param("/trajectory_rrt/rrt/sec", 0.05)
	it = rospy.get_param("/trajectory_rrt/rrt/it", 100)
	#print(map_topic)
	#trajectory params
	Et = []
	p_start =  []
	p_end =  []
	
	#create publisher
	#rviz 
	traj_pub = rospy.Publisher("/WOMBAT/trajectory_gen/path", Marker, queue_size=1)
	obs_pub = rospy.Publisher("/WOMBAT/trajectory_gen/obs", Marker, queue_size=1)
	#msg
	path_pub = rospy.Publisher("/WOMBAT/trajectory_gen/path_msg", Path, queue_size=1)

	#subscriber
	map_sub = rospy.Subscriber(map_topic, OccupancyGrid, mapCallback)
	pose_sub = rospy.Subscriber("/WOMBAT/navegation/pose", PoseStamped, poseCallback)
	goal_sub = rospy.Subscriber("/WOMBAT/navegation/newGoal", Pose2D, goalCallback)
	
	traj_gen = RRT(d , x_range, y_range, sec, it)
	
	recalc_trajectory = True
	
	rospy.spin()
	
	
if __name__ == '__main__':
	main()
	
