from AquaThread.BaseCallbackThread import *
from PacketParse import parse_imu_packet, parse_dvl_packet, parse_position_packet
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

class DataLoggerThread(BaseCallbackWorker):
    def __init__(self, tag="IMU", robot_type="BlueRov2", robot_id=0):
        super().__init__(tag=tag, robot_type=robot_type, robot_id=robot_id)
        self.tag = tag
        self.robot_type = robot_type
        self.robot_id = robot_id

        if self.tag == "IMU":
            self.imu_pub = rospy.Publisher(f'/{self.robot_type}/{self.robot_id}/imu', Imu, queue_size=100)
            self.imu_rate = rospy.Rate(100)  # 控制频率为 100Hz
        
        if self.tag == "DVL":
            self.dvl_pub = rospy.Publisher(f'/{self.robot_type}/{self.robot_id}/dvl', TwistStamped, queue_size=100)
            self.dvl_rate = rospy.Rate(50)  # 控制频率为 50Hz

        if self.tag == "POSITION":
            self.position_pub = rospy.Publisher(f'/{self.robot_type}/{self.robot_id}/position', Odometry, queue_size=100)
            self.position_rate = rospy.Rate(100)  # 控制频率为 100Hz


    def run(self):
        # 记录上一帧的时间戳
        last_timestamp = None

        while self._running:
            with self._lock:
                data = self._data
            if data is not None:
                if self.tag == "IMU":
                    msg_time = data['timestamp']
                    print(f"Imu Time: {msg_time}")
                    if msg_time == last_timestamp:
                        continue  # 跳过重复帧
                    last_timestamp = msg_time

                    # Parse IMU packet
                    imu_data = parse_imu_packet(data)
                    # 创建IMU消息
                    imu_msg = Imu()
                    imu_msg.header.stamp = rospy.Time.from_sec(msg_time)
                    imu_msg.header.frame_id = "imu_link"
                    imu_msg.linear_acceleration.x = imu_data['Acceleration']['x']
                    imu_msg.linear_acceleration.y = imu_data['Acceleration']['y']
                    imu_msg.linear_acceleration.z = imu_data['Acceleration']['z']
                    imu_msg.angular_velocity.x = imu_data['AngularVelocity']['x']
                    imu_msg.angular_velocity.y = imu_data['AngularVelocity']['y']
                    imu_msg.angular_velocity.z = imu_data['AngularVelocity']['z']
                    imu_msg.orientation.x = imu_data['Orientation']['x']
                    imu_msg.orientation.y = imu_data['Orientation']['y']
                    imu_msg.orientation.z = imu_data['Orientation']['z']
                    imu_msg.orientation.w = imu_data['Orientation']['w']
                    # 发布IMU消息
                    self.imu_pub.publish(imu_msg)
                    self.imu_rate.sleep()

                    # print(f"[{self.tag}] Accel: {imu_data['Acceleration']}, Gyro: {imu_data['AngularVelocity']}, Orientation: {imu_data['Orientation']}")

                elif self.tag == "DVL":
                    # Parse DVL packet
                    dvl_data = parse_dvl_packet(data)
                    # print(f"[{self.tag}] Velocity: {dvl_data['Velocity']}")
                    # print(f"DVl Velocity: x_vel:{dvl_data['Velocity']['x']}, y_vel:{dvl_data['Velocity']['y']}")

                elif self.tag == "POSITION":
                    # Parse Position packet
                    position_data = parse_position_packet(data)
                    # print(f"[{self.tag}] Position: {position_data['Position']}, Euler: {position_data['OrientationEuler']}")

            time.sleep(0.01)
