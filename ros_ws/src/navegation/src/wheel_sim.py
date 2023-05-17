#!/usr/bin/env python3

#ROS dependencies
import rospy
from custom_msgs.msg import vel_cmd
from std_msgs.msg import Float64
import random
import numpy as np

wheel_vel = [0.0, 0.0]
noise_multiplier = 0.2

def controlCallback(cmd):
    global wheel_vel

    r = 0.05
    max_vel = 10
    
    max_rads = max_vel/r

    wheel_vel[0] = cmd.wr.data
    wheel_vel[1] = cmd.wl.data

    if not (-max_rads < wheel_vel[0] < max_rads):
        wheel_vel[0] = max_rads*(wheel_vel[0]/abs(wheel_vel[0]))
    
    if not (-max_rads < wheel_vel[1] < max_rads):
        wheel_vel[1] = max_rads*(wheel_vel[1]/abs(wheel_vel[1]))

def main():
    global wheel_vel, noise_multiplier

    rospy.init_node("Noise")
    rate = rospy.Rate(100)

    random.seed(rospy.Time.now())

    control_sub = rospy.Subscriber("/WOMBAT/navegation/control", vel_cmd, controlCallback, queue_size = 10)

    left_pub    = rospy.Publisher("/WOMBAT/navegation/leftSpeed", Float64, queue_size = 1)

    right_pub    = rospy.Publisher("/WOMBAT/navegation/rightSpeed", Float64, queue_size = 1)

    speed = Float64()
    
    while not rospy.is_shutdown():
        
        speed = wheel_vel[0] + random.random()*noise_multiplier*2 - noise_multiplier

        right_pub.publish(speed)

        speed = wheel_vel[1] + random.random()*noise_multiplier*2 - noise_multiplier

        left_pub.publish(speed)

        rate.sleep()

if __name__ == '__main__':
    main()
