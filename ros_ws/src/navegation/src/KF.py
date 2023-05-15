#!/usr/bin/env python3

#ros
import rospy
from geometry_msgs.msg import Pose2D

#Puzzlebot
import numpy as np

#Subscriber callback
def RawPoseCallback(raw_pose):

    #states
    global q, x_hat
    #model
    global A
    #kalman
    global Q,R,Ri,H,Ht,Z,P,K
    #ROS variables
    global pose, KF_pub
    #time
    global t

    q[0,0] = raw_pose.position.x
    q[1,0] = raw_pose.position.y
    q[2,0] = raw_pose.position.theta

    #Update time
    temp = rospy.Time.now()
    dt = temp-t
    t = temp

    #kalman
    x_hat += (A@x_hat + K@(z - H@x_hat))*dt
    P += (Q - K@H@P)*dt 

    #update values
    z = H@q
    K = P@Ht@Ri

    #save message
    pose.x = q[0]
    pose.y = q[1]
    pose.theta = q[2]
    
    #publish message
    KF_pub.publish(pose)

#main
def main():

    #states
    global q, x_hat
    #model
    global A
    #kalman
    global Q,R,Ri,H,Ht,Z,P,K
    #ROS variables
    global pose, KF_pub
    #time
    global t

    #Initialize node
    rospy.init_node("KF")

    #parameters

    r = 0.04 #wheel radius
    d = 0.10 #distance between wheels
    h = 0.15 #distance between center and new point
    
    #             [X, Y, theta]
    q = np.array([[0.0,0.0,0.0]]).T

    #matrix
    D = np.array([[r/2*np.cos(q[2,0]) - h*r/d*np.sin(q[2,0]), r/2*np.cos(q[2,0]) + h*r/d*np.sin(q[2,0])], 
                  [r/2*np.sin(q[2,0]) + h*r/d*np.cos(q[2,0]), r/2*np.sin(q[2,0]) - h*r/d*np.cos(q[2,0])]])
    
    Phi = np.array([[r/d, -r/d]])

    A = np.concatenate(D,Phi)

    #kalman filter
    Q = np.array([[1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0]])

    R = np.array([[1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0]])
    Ri = np.linalg.inv(R)

    H = np.array([[0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0]])
    Ht = H.T

    Z = H@q

    P = np.array([[0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0]])

    K = P@Ht@Ri

    #initial time
    t = rospy.Time.now()

    x_hat = np.array([[0.0, 0.0, 0.0]]).T
   
    #message
    pose = Pose2D()
    
    #oddometry subscriber
    pose_sub = rospy.Subscriber("/WOMBAT/navegation/odometry", Pose2D, RawPoseCallback, queue_size=10)
    
    #estimation publisher
    KF_pub = rospy.Publisher("/WOMBAT/navegation/pose", Pose2D, queue_size = 10)

    #callback
    rospy.spin()

if __name__ == '__main__':
    main()
