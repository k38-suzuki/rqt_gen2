#!/usr/bin/env python

import rospy
import kinova_msgs.msg

def talker():
    pub = rospy.Publisher('/j2s7s300_driver/in/cartesian_velocity', kinova_msgs.msg.PoseVelocity, queue_size=2)
    rospy.init_node('gen2_ros', anonymous=True)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        pose_velocity_msg = kinova_msgs.msg.PoseVelocity()
        pose_velocity_msg.twist_linear_x = 0.0
        pose_velocity_msg.twist_linear_y = 0.0
        pose_velocity_msg.twist_linear_z = 0.0
        pose_velocity_msg.twist_angular_x = 0.0
        pose_velocity_msg.twist_angular_y = 0.0
        pose_velocity_msg.twist_angular_z = 0.0
        pub.publish(pose_velocity_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
