#! /usr/bin/env python
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from sensor_msgs.msg import Imu, Image
from geometry_msgs.msg import TwistStamped
from AquaUDPReceiver import AquaReceiverStart
from AquaUDPTransporter import *
from PacketParse import *
from Keyboard import *
from AquaThread.ImageDisplayThread import *
from AquaThread.DataLoggerThread import *

def handle_imu(msg):
    # Parse IMU packet
    imu_data = parse_imu_packet(msg)
    imu_msg = Imu()
    msg_time = msg['timestamp']
    imu_msg.header.stamp = rospy.Time.from_sec(msg_time)
    imu_msg.header.frame_id = "imu_link"
    imu_msg.linear_acceleration.x = imu_data['Acceleration']['x']
    imu_msg.linear_acceleration.y = -imu_data['Acceleration']['y']
    imu_msg.linear_acceleration.z = imu_data['Acceleration']['z']
    imu_msg.angular_velocity.x = imu_data['AngularVelocity']['x']
    imu_msg.angular_velocity.y = -imu_data['AngularVelocity']['y']
    imu_msg.angular_velocity.z = -imu_data['AngularVelocity']['z']
    q = imu_data['Orientation']
    quat = [q['x'], q['y'], q['z'], q['w']]
    roll, pitch, yaw = euler_from_quaternion(quat)
    yaw = -yaw
    new_quat = quaternion_from_euler(roll, pitch, yaw)
    imu_msg.orientation.x = new_quat[0]
    imu_msg.orientation.y = new_quat[1]
    imu_msg.orientation.z = new_quat[2]
    imu_msg.orientation.w = new_quat[3]
    imu_pub.publish(imu_msg)

    # print(f"IMU header: {imu_msg.header}")
    # print(f"IMU linear_acceleration: {imu_msg.linear_acceleration}")
    # print(f"IMU angular_velocity: {imu_msg.angular_velocity}")
    # print(f"IMU orientation: {imu_msg.orientation}")
    # print("------------")





def handle_dvl(msg):
    # Parse DVL packet
    dvl_data = parse_dvl_packet(msg)
    dvl_msg = TwistStamped()
    msg_time = msg['timestamp']
    dvl_msg.header.stamp = rospy.Time.from_sec(msg_time)
    dvl_msg.header.frame_id = "dvl_link"
    dvl_msg.twist.linear.x = dvl_data['Velocity']['x']
    dvl_msg.twist.linear.y = -dvl_data['Velocity']['y']
    dvl_msg.twist.linear.z = dvl_data['Velocity']['z']
    dvl_pub.publish(dvl_msg)

def handle_position(msg):
    # Parse Position packet
    position_data = parse_position_packet(msg)
    position_msg = Odometry()
    msg_time = msg['timestamp']
    position_msg.header.stamp = rospy.Time.from_sec(msg_time)
    position_msg.header.frame_id = "absolute_odom"
    position_msg.pose.pose.position.x = position_data['Position']['x']
    position_msg.pose.pose.position.y = -position_data['Position']['y']
    position_msg.pose.pose.position.z = position_data['Position']['z']
    q = position_data['OrientationQuat']
    quat = [q['x'], q['y'], q['z'], q['w']]
    roll, pitch, yaw = euler_from_quaternion(quat)
    yaw = -yaw
    new_quat = quaternion_from_euler(roll, pitch, yaw)
    position_msg.pose.pose.orientation.x = new_quat[0]
    position_msg.pose.pose.orientation.y = new_quat[1]
    position_msg.pose.pose.orientation.z = new_quat[2]
    position_msg.pose.pose.orientation.w = new_quat[3]

    position_pub.publish(position_msg)

def handle_multibeamimagingsonar(msg):
    np_arr = np.frombuffer(msg['data'], np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)

    sonar_img = Image()
    msg_time = msg['timestamp']
    sonar_img.header.stamp = rospy.Time.from_sec(msg_time)
    sonar_img.header.frame_id = "sonar_link"
    sonar_img.height, sonar_img.width = img.shape
    sonar_img.encoding = "mono8"
    sonar_img.is_bigendian = 0
    sonar_img.step = sonar_img.width
    sonar_img.data = img.tobytes()
    sonar_pub.publish(sonar_img)


if __name__ == '__main__':
    rospy.init_node('aqua_ros_get_message_node', anonymous=True)

    rospy.loginfo("\033[1;32m----> AquaROSGetMessage Node Started.\033[0m")


    use_keyboard = rospy.get_param('~use_keyboard_control', False)


    imu_pub = rospy.Publisher('/BlueRov2/0/imu',Imu, queue_size=100)
    sonar_pub = rospy.Publisher('/BlueRov2/0/sonar', Image, queue_size=100)
    dvl_pub = rospy.Publisher(f'/BlueRov2/0/dvl', TwistStamped, queue_size=100)
    position_pub = rospy.Publisher(f'/BlueRov2/0/position', Odometry, queue_size=100)


    on_topic_callbacks = {
        '/BlueRov2/0/Dvl': handle_dvl,
        '/BlueRov2/0/Imu': handle_imu,
        '/BlueRov2/0/Position': handle_position,
        '/BlueRov2/0/MultibeamImagingSonar': handle_multibeamimagingsonar,
    }

    AquaReceiverStart(on_topic_callbacks)


    if use_keyboard:
        terminal = TerminalSettings()
        terminal.disable_echo()  

        thrust_thread = Thread(target=send_control_loop)
        thrust_thread.start()

        # Start keyboard monitoring
        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()

        thrust_thread.join()

    rospy.spin()

