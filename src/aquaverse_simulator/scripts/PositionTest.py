#! /usr/bin/env python
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
import math



if __name__ == '__main__':
    rospy.init_node('position_test_node')
    position_pub = rospy.Publisher('/BlueRov2/0/position_control', Odometry, queue_size=100)

    position_x = 20
    position_y = 20
    position_yaw_degree = 0

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        position_msg = Odometry()
        position_msg.header.stamp = rospy.Time.now()
        position_msg.pose.pose.position.x = position_x
        position_msg.pose.pose.position.y = position_y
        q = quaternion_from_euler(0, 0, position_yaw_degree*3.14/180)
        position_msg.pose.pose.orientation.x = q[0]
        position_msg.pose.pose.orientation.y = q[1]
        position_msg.pose.pose.orientation.z = q[2]
        position_msg.pose.pose.orientation.w = q[3]

        position_pub.publish(position_msg)

        rate.sleep()
    
    rospy.spin()
