#! /usr/bin/env python
import rospy
from geometry_msgs.msg import TwistStamped



if __name__ == '__main__':
    rospy.init_node('cmd_vel_test_node')
    cmd_vel_pub = rospy.Publisher('/BlueRov2/0/cmd_vel', TwistStamped, queue_size=100)
    rate = rospy.Rate(50) # 50hz


    while not rospy.is_shutdown():
        twist_msg = TwistStamped()
        twist_msg.header.stamp = rospy.Time.now()
        twist_msg.twist.linear.x = 0      # 0.4
        twist_msg.twist.linear.y = 0      # 0.2
        twist_msg.twist.angular.z = 0.4   # 0.2
        cmd_vel_pub.publish(twist_msg)

        rate.sleep()

