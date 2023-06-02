#!/usr/bin/env python

#ros
import rospy
from geometry_msgs.msg import PoseStamped, Pose2D
from std_msgs.msg import Float32

#Puzzlebot
import numpy as np

#Subscriber callback
def rawPoseCallback(raw_pose):

    #states
    global q, x_hat
    #model
    global A, u, r, d, h
    #kalman
    global Q,R,Ri,H,Ht,Z,P,K
    #ROS variables
    global pose, KF_pub
    #time
    global t

    q[0,0] = raw_pose.x
    q[1,0] = raw_pose.y
    q[2,0] = raw_pose.theta

    #update time
    temp = rospy.Time.now()
    dt = float(temp.nsecs - t.nsecs)/1000000000
    if dt < 0:
        dt += 1
    t = temp
    
    #kalman
    x_hat += (np.dot(A,u) + np.dot(K,(Z - np.dot(H,x_hat))))*dt
    P += (Q - np.dot(np.dot(K,H),P))*dt 

    #update values
    Z = np.dot(H,q)
    K = np.dot(np.dot(P,Ht),Ri)

    
    #save message
    #header
    pose.header.frame_id = "world"
    pose.header.stamp = t
    #pose
    pose.pose.position.x = x_hat[0, 0]
    pose.pose.position.y = x_hat[1, 0]
    pose.pose.position.z = 0.0
    #orientation
    pose.pose.orientation.w = np.cos(x_hat[2, 0] * 0.5)
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = np.sin(x_hat[2, 0] * 0.5)
    
    #publish message
    KF_pub.publish(pose)

def leftCallback(vel):
    global u
    u[1, 0] = vel.data
    
def rightCallback(vel):
    global u
    u[0, 0] = vel.data

#main
def main():

    #states
    global q, x_hat
    #model
    global A, u, r, d, h
    #kalman
    global Q,R,Ri,H,Ht,Z,P,K
    #ROS variables
    global pose, KF_pub
    #time
    global t

    #Initialize node
    rospy.init_node("KF")

    #parameters
    r = 0.05 #wheel radius
    d = 0.08 #distance between wheels
    h = 0.02 #distance between center and new point
    
    #             [X, Y, theta]
    q = np.array([[0.0,0.0,0.0]]).T

    #             [right, left]
    u = np.array([[0.0, 0.0]]).T

    #matrix
    D = np.array([[r/2*np.cos(q[2,0]) - h*r/d*np.sin(q[2,0]), r/2*np.cos(q[2,0]) + h*r/d*np.sin(q[2,0])], 
                  [r/2*np.sin(q[2,0]) + h*r/d*np.cos(q[2,0]), r/2*np.sin(q[2,0]) - h*r/d*np.cos(q[2,0])]])
    
    Phi = np.array([[r/d, -r/d]])

    A = np.concatenate((D,Phi))

    #kalman filter
    Q = np.array([[1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0]])

    R = np.array([[1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0]])
    Ri = np.linalg.inv(R)

    H = np.array([[1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0]])
    Ht = H.T

    Z = np.dot(H,q)

    P = np.array([[0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0]])

    K = np.dot(np.dot(P,Ht),Ri)

    #initial time
    t = rospy.Time.now()

    x_hat = np.array([[0.0, 0.0, 0.0]]).T
   
    #message
    pose = PoseStamped()
    
    l_speed = rospy.get_param("/navigation/topics/vel_l", "/WOMBAT/navegation/leftSpeed")
    r_speed = rospy.get_param("/navegation/topics/vel_r", "/WOMBAT/navegation/rightSpeed")

    #oddometry subscriber
    pose_sub  = rospy.Subscriber("/WOMBAT/navegation/odometry", Pose2D, rawPoseCallback, queue_size=10)
    left_sub  = rospy.Subscriber(l_speed, Float32, leftCallback, queue_size = 1)
    right_sub = rospy.Subscriber(r_speed, Float32, rightCallback, queue_size = 1)
    
    #estimation publisher
    KF_pub = rospy.Publisher("/WOMBAT/navegation/pose", PoseStamped, queue_size = 10)

    #callback
    rospy.spin()

if __name__ == '__main__':
    main()
