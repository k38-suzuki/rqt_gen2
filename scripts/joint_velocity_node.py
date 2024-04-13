#!/usr/bin/env python

import rospy
import kinova_msgs.msg

def talker():
    pub = rospy.Publisher('/j2s7s300_driver/in/joint_velocity', kinova_msgs.msg.JointVelocity, queue_size=2)
    rospy.init_node('gen2_ros', anonymous=True)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        joint_velocity_msg = kinova_msgs.msg.JointVelocity()
        joint_velocity_msg.joint1 = 0.0
        joint_velocity_msg.joint2 = 0.0
        joint_velocity_msg.joint3 = 0.0
        joint_velocity_msg.joint4 = 0.0
        joint_velocity_msg.joint5 = 0.0
        joint_velocity_msg.joint6 = 0.0
        joint_velocity_msg.joint7 = 0.0
        pub.publish(joint_velocity_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
