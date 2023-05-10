#!/usr/bin/env python3

#ros
import rospy
from geometry_msgs.msg import Pose2D

#Puzzlebot
import numpy as np

def main():

    kart = rospy.Publisher("/Puzzlebot/pose", Pose2D, queue_size = 10)
    
    rospy.init_node("Control")
    rate = rospy.Rate(1000)

    #Parameters
    dt = 0.001 #time step

    r = 0.04 #wheel radius
    d = 0.10 #distance between wheels
    h = 0.15 #distance between center and new point

    k = np.array([[1, 0],
                  [0, 2]])

    #             [X, Y, theta]
    q = np.array([[0.0,0.0,0.0]]).T

    qd = np.array([[1000.0, -500.0]]).T

    qe = q[0:2] - qd

    #control U = [Wr, Wl]
    u = np.array([[0,0]]).T

    #matrix
    D = np.array([[r/2*np.cos(q[2,0]) - h*r/d*np.sin(q[2,0]), r/2*np.cos(q[2,0]) + h*r/d*np.sin(q[2,0])], 
                  [r/2*np.sin(q[2,0]) + h*r/d*np.cos(q[2,0]), r/2*np.sin(q[2,0]) - h*r/d*np.cos(q[2,0])]])
    
    Phi = np.array([[r/d, -r/d]])

    A = np.concatenate(D,Phi)

    
    pose = Pose2D()

    while not rospy.is_shutdown():

        qe = q[0:2] - qd
        #print(qe)
        u = np.linalg.inv(D)@-k@qe
        #print(u)
        
        #u = np.array([[1,5]]).T
        D = np.array([[r/2*np.cos(q[2,0]) - h*r/d*np.sin(q[2,0]), r/2*np.cos(q[2,0]) + h*r/d*np.sin(q[2,0])], 
                  [r/2*np.sin(q[2,0]) + h*r/d*np.cos(q[2,0]), r/2*np.sin(q[2,0]) - h*r/d*np.cos(q[2,0])]])
    
        Phi = np.array([[r/d, -r/d]])

        A = np.concatenate(D,Phi)

        #print(A@u*dt)
        q += A@u*dt

        pose.x = q[0]
        pose.y = q[1]
        pose.theta = q[2]

        kart.publish(pose)

        rate.sleep()


if __name__ == "__main__":
    main()