#!/usr/bin/env python

#ROS
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D, PoseStamped

#Puzzlebot
import numpy as np
import math

#Global variables
t = 0.0

#Matrices
wheel_speed = np.array([[0.0, 0.0]]).T
q           = np.array([[0.0, 0.0, 0.0]]).T

def rightVelCallback(vel):
    global wheel_speed
    #print(vel.data)
    wheel_speed[0, 0] = vel.data

def leftVelCallback(vel):
    global wheel_speed
    wheel_speed[1, 0] = vel.data

def kalmanPosCallback(pos):
    global q

    #convert from quaternion to euler
    
    # save orientation
    w = pos.pose.orientation.w 
    x = pos.pose.orientation.x
    y = pos.pose.orientation.y 
    z = pos.pose.orientation.z 

    #apply convertion
    t1 = +2.0 * (w * z + x * y)
    t2 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t1, t2)

    #save states
    q[0, 0] = pos.pose.position.x
    q[1, 0] = pos.pose.position.y 
    q[2, 0] = yaw

def main():

    #parameters

    r = 0.05 #wheel radius
    d = 0.08 #distance between wheels
    h = 0.02 #distance between center and new point

    global q, wheel_speed, t
    
    rospy.init_node("Odometry")

    rate = rospy.Rate(1000)
	
    l_speed = rospy.get_param("/navigation/topics/vel_l", "/WOMBAT/navegation/leftSpeed")
    r_speed = rospy.get_param("/navigation/topics/vel_r", "/WOMBAT/navegation/rightSpeed")
	
    pose_pub = rospy.Publisher("/WOMBAT/navegation/odometry", Pose2D, queue_size = 10)
    #rviz_pose = rospy.Publisher("WOMBAT/navegation/rvizPose", PoseStamped, queue_size=10)

    kf_sub   = rospy.Subscriber("/WOMBAT/navegation/pose", PoseStamped, kalmanPosCallback, queue_size = 10)

    l_sub    = rospy.Subscriber(l_speed, Float32, leftVelCallback, queue_size = 10)

    r_sub    = rospy.Subscriber(r_speed, Float32, rightVelCallback, queue_size = 10)

    t = rospy.Time.now()

    pose = Pose2D()

    while not rospy.is_shutdown():
        
        #update time
        temp = rospy.Time.now()
        dt = float(temp.nsecs - t.nsecs)/1000000000
        
        if dt<0.0: 
            dt += 1     

        t = temp

        A = np.array([[r/2*np.cos(q[2,0]) - h*r/d*np.sin(q[2,0]), r/2*np.cos(q[2,0]) + h*r/d*np.sin(q[2,0])],
                      [r/2*np.sin(q[2,0]) + h*r/d*np.cos(q[2,0]), r/2*np.sin(q[2,0]) - h*r/d*np.cos(q[2,0])],
                      [r/d, -r/d]])


        q += (np.dot(A,wheel_speed))*dt
       
        pose.x     = q[0, 0]
        pose.y     = q[1, 0]
        pose.theta = q[2, 0]

        pose_pub.publish(pose)

        rate.sleep()

if __name__ == '__main__':
    main()
