#!/usr/bin/env python

#import ros
import rospy

from custom_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose2D

import numpy as np

#callback
def pathCallback(path):

    global goal
    goal = np.array(path)


def poseCallback(odom):
    global goal, thresh
	
	#update actual bot position
    x = odom.pose.position.x
    y = odom.pose.position.y
    pose = [x,y]

    if len(goal) > 0:
        
        d = np.sqrt(np.power(pose[0] - goal.path[0].x, 2) + np.power(pose[1] - goal.path[0].y, 2))
        if d > thresh:
             
            #publish point from path
            point = Pose2D
            point.x = goal.path[0].x
            point.y = goal.path[0].y

            path_pub.publish(point)
        else:
             goal.pop(0)

		
def main():

    global path_pub, goal, thresh

    goal = []
    thresh = 0.5
    #inicialize node
    rospy.init_node("subGoal_pub")

    #create publisher
    path_pub = rospy.Publisher("/WOMBAT/navegation/goal", Pose2D, queue_size=1)

    #subscriber
    pose_sub = rospy.Subscriber("/WOMBAT/navegation/pose", PoseStamped, poseCallback)
    path_sub = rospy.Subscriber("/WOMBAT/trajectory_gen/path_msg", Path, pathCallback)
	
    rospy.spin()
	
	
if __name__ == '__main__':
	main()
	
