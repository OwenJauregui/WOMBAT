#!/usr/bin/env python

#ROS dependencies
import rospy
from custom_msgs.msg import vel_cmd
from geometry_msgs.msg import Pose2D

#Import numpy
import numpy as np

q  = np.array([[0, 0, 0]]).T
qd = np.array([[0, 0, 0]]).T

#Parameters
r = 0.04 #wheel radius
d = 0.10 #distance between wheels
h = 0.15 #distance between center and new point

k = np.array([[1, 0],
              [0, 2]])
cmd = vel_cmd()

control = rospy.Publisher("/wombat/navegation/control", vel_cmd, queue_size = 10)

def controlCalc():
    global q, qd, r, d, h, k, cmd, control

    qe = q[0:2] - qd

        #u = np.array([[1,5]]).T
    D = np.array([[r/2*np.cos(q[2,0]) - h*r/d*np.sin(q[2,0]), r/2*np.cos(q[2,0]) + h*r/d*np.sin(q[2,0])],
                  [r/2*np.sin(q[2,0]) + h*r/d*np.cos(q[2,0]), r/2*np.sin(q[2,0]) - h*r/d*np.cos(q[2,0])]])

    u = np.dot(np.linalg.inv(D),np.dot(-k,qe))

    cmd.wr.data = u[0, 0]
    cmd.wl.data = u[1, 0]

    control.publish(cmd)

def newEstimation(pose_estimation):
    global q
    q[0,0] = pose_estimation.x
    q[1,0] = pose_estimation.y
    q[2,0] = pose_estimation.theta
    controlCalc()

def newGoal(goal_pose):
    global qd
    qd[0,0] = goal_pose.x
    qd[1,0] = goal_pose.y
    qd[2,0] = goal_pose.theta

def main():
    
    pose_sub = rospy.Subscriber("/wombat/navegation/estimation", Pose2D, newEstimation, queue_size = 10)

    goal_sub = rospy.Subscriber("/wombat/navegation/goal", Pose2D, newGoal, queue_size = 1)
    rospy.init_node("Control")
    rate = rospy.Rate(1000)
    rospy.spin()

if __name__ == "__main__":
    main()
