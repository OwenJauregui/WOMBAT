#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Float64
from sensor_msgs.msg import JointState

def cmd_callback(data, pub):
    # Convert Float32 to Float64
    float64_data = Float64()
    float64_data.data = float(data.data)

    # Publish the converted data
    pub.publish(float64_data)

def vel_callback(data, (joint_name, pub)):

    # Extract the joint velocity from the received JointState message
    print("Callback")
    if joint_name in data.name:
        index = data.name.index(joint_name)
        if index < len(data.velocity):
            velocity = data.velocity[index]
    
            # Convert Float32 to Float64
            float32_data = Float32()
            float32_data.data = velocity

            # Publish the converted data
            pub.publish(float32_data)

if __name__ == '__main__':
    rospy.init_node('float_conversion_node')

    l_cmd = rospy.get_param("/navigation/topics/cmd_vel_l", "/WOMBAT/navegation/cmd_l")
    r_cmd = rospy.get_param("/navigation/topics/cmd_vel_r", "/WOMBAT/navegation/cmd_r")
    l_speed = rospy.get_param("/navigation/topics/vel_l", "/WOMBAT/navegation/leftSpeed")
    r_speed = rospy.get_param("/navigation/topics/vel_r", "/WOMBAT/navegation/rightSpeed")

    gazebo_l_cmd = '/left_wheel_controller/command'
    gazebo_r_cmd = '/right_wheel_controller/command'
    gazebo_l_speed = 'base_to_left_w'
    gazebo_r_speed = 'base_to_right_w'

    # Create publishers for the Float64 topics
    pub1 = rospy.Publisher(gazebo_l_cmd, Float64, queue_size=10)
    pub2 = rospy.Publisher(gazebo_r_cmd, Float64, queue_size=10)
    pub3 = rospy.Publisher(l_speed, Float32, queue_size=10)
    pub4 = rospy.Publisher(r_speed, Float32, queue_size=10)

    # Create subscribers for the Float32 topics
    rospy.Subscriber(l_cmd, Float32, cmd_callback, callback_args=pub1)
    rospy.Subscriber(r_cmd, Float32, cmd_callback, callback_args=pub2)
    rospy.Subscriber('/joint_states', JointState, vel_callback, callback_args=(gazebo_l_speed, pub3))
    rospy.Subscriber('/joint_states', JointState, vel_callback, callback_args=(gazebo_r_speed, pub4))
 

    # Spin the node
    rospy.spin()

