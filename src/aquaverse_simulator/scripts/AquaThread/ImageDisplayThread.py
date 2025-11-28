from AquaThread.BaseCallbackThread import *
import rospy
from sensor_msgs.msg import Imu, Image
import std_msgs.msg

class ImageDisplayThread(BaseCallbackWorker):
    def __init__(self, tag="Image", robot_type="BlueRov2", robot_id=0):
        super().__init__(tag=tag, robot_type=robot_type, robot_id=robot_id)
        self.tag = tag
        self.robot_type = robot_type
        self.robot_id = robot_id

        if self.tag == "SONAR":
            self.sonar_pub = rospy.Publisher(f'/{self.robot_type}/{self.robot_id}/sonar', Image, queue_size=10)
            self.sonar_rate = rospy.Rate(10)  # 控制频率为 10Hz
        
    def run(self):
        # 记录上一帧的时间戳
        last_timestamp = None

        while self._running:
            with self._lock:
                msg = self._data.copy() if self._data is not None else None
            if msg is None:
                continue
            msg_time = msg['timestamp']
            if msg_time == last_timestamp:
                continue  # 跳过重复帧
            last_timestamp = msg_time  # 更新上一帧时间戳
            np_arr = np.frombuffer(msg['data'], np.uint8)

            # 发送成像声呐数据到ROS
            if self.tag == "SONAR":
                img = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
                # 转换成灰度图像
                sonar_img = Image()
                sonar_img.header.stamp = rospy.Time.from_sec(msg_time)
                sonar_img.header.frame_id = "sonar_link"
                sonar_img.height, sonar_img.width = img.shape
                sonar_img.encoding = "mono8"
                sonar_img.is_bigendian = 0
                sonar_img.step = sonar_img.width
                sonar_img.data = img.tobytes()
                self.sonar_pub.publish(sonar_img)

                self.sonar_rate.sleep()



