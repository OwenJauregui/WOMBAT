#!/usr/bin/env python3

#ROS
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose2D

#Puzzlebot
import numpy as np

#Global variables
t = 0

#Matrices
wheel_speed = np.array([[0.0, 0.0]]).T
q           = np.array([[0.0, 0.0, 0.0]]).T

def rightVelCallback(vel):
    wheel_speed[0, 0] = vel

def leftVelCallback(vel):
    wheel_speed[1, 0] = vel

def kalmanPosCallback(pos):
    q[0, 0] = pos.position.x
    q[1, 0] = pos.position.y
    q[2, 0] = pos.position.theta

def main():
    
    rospy.init_node("Odometry")

    rate = rospy.Rate(100)

    pose_pub = rospy.Publisher("/WOMBAT/navegation/odometry", Pose2D, queue_size = 10)
    
    kf_sub   = rospy.Subscriber("/WOMBAT/navegation/pose", Pose2D, kalmanPosCallback, queue_size = 10)

    l_sub    = rospy.Subscriber("/WOMBAT/navegation/leftSpeed", Float64, leftVelCallback, queue_size = 10)

    r_sub    = rospy.Subscriber("/WOMBAT/navegation/rightSpeed", Float64, rightVelCallback, queue_size = 10)

    t = rospy.Time.now()

    pose = Pose2D()

    while not rospy.is_shutdown():
        
        temp = rospy.Time.now()
        dt = temp - t
        t = temp

        A = np.array([[r/2*np.cos(q[2,0]) - h*r/d*np.sin(q[2,0]), r/2*np.cos(q[2,0]) + h*r/d*np.sin(q[2,0])],
                      [r/2*np.sin(q[2,0]) + h*r/d*np.cos(q[2,0]), r/2*np.sin(q[2,0]) - h*r/d*np.cos(q[2,0])],
                      [r/d, -r/d]])


        q += A@wheel_speed*dt
        
        pose.position.x     = q[0, 0]
        pose.position.y     = q[1, 0]
        pose.position.theta = q[2, 0]

        puse_pub.publish()

        rate.sleep()

if __name__ == '__main__':
    main()
