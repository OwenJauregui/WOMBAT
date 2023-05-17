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
    global wheel_speed
    wheel_speed[0, 0] = vel.data

def leftVelCallback(vel):
    global wheel_speed
    wheel_speed[1, 0] = vel.data

def kalmanPosCallback(pos):
    global q
    q[0, 0] = pos.x
    q[1, 0] = pos.y
    q[2, 0] = pos.theta

def main():

    #parameters

    r = 0.05 #wheel radius
    d = 0.08 #distance between wheels
    h = 0.02 #distance between center and new point

    global q, wheel_speed, t
    
    rospy.init_node("Odometry")

    rate = rospy.Rate(1000)

    pose_pub = rospy.Publisher("/WOMBAT/navegation/odometry", Pose2D, queue_size = 10)
    
    kf_sub   = rospy.Subscriber("/WOMBAT/navegation/pose", Pose2D, kalmanPosCallback, queue_size = 10)

    l_sub    = rospy.Subscriber("/WOMBAT/navegation/leftSpeed", Float64, leftVelCallback, queue_size = 10)

    r_sub    = rospy.Subscriber("/WOMBAT/navegation/rightSpeed", Float64, rightVelCallback, queue_size = 10)

    t = rospy.Time.now()

    pose = Pose2D()

    while not rospy.is_shutdown():
        
        #update time
        temp = rospy.Time.now()
        dt = (temp.nsecs - t.nsecs)/1000000000
        
        if dt<0.0: 
            dt += 1     

        t = temp

        A = np.array([[r/2*np.cos(q[2,0]) - h*r/d*np.sin(q[2,0]), r/2*np.cos(q[2,0]) + h*r/d*np.sin(q[2,0])],
                      [r/2*np.sin(q[2,0]) + h*r/d*np.cos(q[2,0]), r/2*np.sin(q[2,0]) - h*r/d*np.cos(q[2,0])],
                      [r/d, -r/d]])


        q += (A@wheel_speed)*dt
        
        pose.x     = q[0, 0]
        pose.y     = q[1, 0]
        pose.theta = q[2, 0]

        pose_pub.publish(pose)

        rate.sleep()

if __name__ == '__main__':
    main()
