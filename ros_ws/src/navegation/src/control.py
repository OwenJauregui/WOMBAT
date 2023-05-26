#!/usr/bin/env python

#ROS dependencies
import rospy
from std_msgs.msg import Float32
from custom_msgs.msg import vel_cmd
from geometry_msgs.msg import Pose2D, PoseStamped

#Import numpy
import numpy as np
import math

q  = np.array([[0, 0, 0]]).T
qd = np.array([[0, 0, 0]]).T

#parameters
r = 0.05 #wheel radius
d = 0.08 #distance between wheels
h = 0.02 #distance between center and new point

k = np.array([[1, 0],
              [0, 2]])
cmd = vel_cmd()

def controlCalc():
    global q, qd, r, d, h, k, cmd, control, left_pub, right_pub

    qe = q[0:2] - qd[0:2]

        #u = np.array([[1,5]]).T
    D = np.array([[r/2*np.cos(q[2,0]) - h*r/d*np.sin(q[2,0]), r/2*np.cos(q[2,0]) + h*r/d*np.sin(q[2,0])],
                  [r/2*np.sin(q[2,0]) + h*r/d*np.cos(q[2,0]), r/2*np.sin(q[2,0]) - h*r/d*np.cos(q[2,0])]])

    u = np.dot(np.linalg.inv(D),(np.dot(-k,qe)))

    cmd.wr.data = u[0, 0]
    cmd.wl.data = u[1, 0]

    control.publish(cmd)

    left_pub.publish(cmd.wl)
    right_pub.publish(cmd.wr)    

def newEstimationCallback(pose_estimation):
    global q

    #convert from quaternion to euler
    # save orientation
    w = pose_estimation.pose.orientation.w 
    x = pose_estimation.pose.orientation.x
    y = pose_estimation.pose.orientation.y 
    z = pose_estimation.pose.orientation.z 

    #apply convertion
    t1 = +2.0 * (w * z + x * y)
    t2 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t1, t2)

    #save states
    q[0, 0] = pose_estimation.pose.position.x
    q[1, 0] = pose_estimation.pose.position.y 
    q[2, 0] = yaw
    controlCalc()

def newGoalCallback(goal_pose):
    global qd
    qd[0,0] = goal_pose.x
    qd[1,0] = goal_pose.y
    qd[2,0] = goal_pose.theta

def main():

    global control, left_pub, right_pub

    rospy.init_node("Control")
	
    l_speed = rospy.get_param("/navigation/topics/cmd_vel_l", "/WOMBAT/navegation/cmd_l")
    r_speed = rospy.get_param("/navigation/topics/cmd_vel_r", "/WOMBAT/navegation/cmd_r")

    control  = rospy.Publisher("/WOMBAT/navegation/control", vel_cmd, queue_size=1)
    left_pub = rospy.Publisher(l_speed, Float32, queue_size=1)
    right_pub = rospy.Publisher(r_speed, Float32, queue_size=1)

    pose_sub = rospy.Subscriber("/WOMBAT/navegation/pose", PoseStamped, newEstimationCallback, queue_size = 10)

    goal_sub = rospy.Subscriber("/WOMBAT/navegation/goal", Pose2D, newGoalCallback, queue_size = 1)
    rospy.spin()

if __name__ == "__main__":
    main()
