#!/usr/bin/env python

#ROS
import rospy
from geometry_msgs.msg import Pose2D, PoseStamped

#Puzzlebot
import numpy as np

def estimationCallback(pos):
    global t, pose, rviz_pub

    #prepare msg
    #header
    pose.header.frame_id = "world"
    pose.header.stamp = t
    #pose
    pose.pose.position.x = pos.x
    pose.pose.position.y = pos.y
    #orientation
    pose.pose.orientation.w = np.cos(pos.theta * 0.5)
    pose.pose.orientation.z = np.sin(pos.theta * 0.5)

    #publish msg
    rviz_pub.publish(pose)

def main():

    #Global variables
    global t, pose, rviz_pub
    t = 0
    
    rospy.init_node("Simulation")

    #publisher
    rviz_pub = rospy.Publisher("WOMBAT/navegation/rvizPose", PoseStamped, queue_size=10)

    #subscriber
    pose_sub  = rospy.Subscriber("/WOMBAT/navegation/pose", Pose2D, estimationCallback, queue_size=10)
    
    t = rospy.Time.now()

    #rviz msg
    pose = PoseStamped()

    #callback
    rospy.spin()


if __name__ == '__main__':
    main()
