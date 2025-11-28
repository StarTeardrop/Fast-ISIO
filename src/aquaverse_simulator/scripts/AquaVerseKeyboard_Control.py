from Keyboard import *
from AquaThread.ImageDisplayThread import *
from AquaThread.DataLoggerThread import *
import subprocess


# Topic callback function
def handle_color(msg):
    np_arr = np.frombuffer(msg['data'], np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
    if img is not None:
        color_display.update_data(img)

def handle_depth(msg):
    np_arr = np.frombuffer(msg['data'], np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
    if img is not None:
        depth_display.update_data(img)

def handle_underwater(msg):
    np_arr = np.frombuffer(msg['data'], np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
    if img is not None:
        underwater_display.update_data(img)

def handle_multibeamImagingSonar(msg):
    np_arr = np.frombuffer(msg['data'], np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
    if img is not None:
        sonar_display.update_data(img)

def handle_Imu(msg):
    imu_logger.update_data(msg)

def handle_dvl(msg):
    dvl_logger.update_data(msg)

def handle_position(msg):
    position_logger.update_data(msg)


if __name__ == '__main__':
    # exe_path = r"E:\\Test\\Windows\\AquaVerse.exe"
    # profile_path = r'E:\\Profiles'
    # Start simulator
    # subprocess.Popen([exe_path, f"-ProfilePath={profile_path}"])

    on_topic_callbacks = {
        '/BlueRov2/0/0/Color': handle_color,
        '/BlueRov2/0/0/Depth': handle_depth,
        '/BlueRov2/0/0/Underwater': handle_underwater,
        '/BlueRov2/0/MultibeamImagingSonar': handle_multibeamImagingSonar,
        '/BlueRov2/0/Imu': handle_Imu,
        '/BlueRov2/0/Dvl': handle_dvl,
        '/BlueRov2/0/Position': handle_position
    }

    AquaReceiverStart(on_topic_callbacks)

    print("==>Use keyboard control Robot：")
    print("==>I/K/J/L: Front/back/left/right")
    print("==>U/O: Left/right rotation")
    print("==>Z/X: Down/Up")
    print("==>ESC: Quit")

    # Initialize data receiving thread
    color_display = ImageDisplayThread(window_name="COLOR")
    # depth_display = ImageDisplayThread(window_name="DEPTH")
    # underwater_display = ImageDisplayThread(window_name="UNDERWATER")
    # sonar_display = ImageDisplayThread(window_name="SONAR")
    imu_logger = DataLoggerThread(tag="IMU")
    dvl_logger = DataLoggerThread(tag="DVL")
    position_logger = DataLoggerThread(tag="POSITION")
    # Start data receiving thread
    color_display.start()
    # depth_display.start()
    # underwater_display.start()
    # sonar_display.start()
    imu_logger.start()
    dvl_logger.start()
    position_logger.start()

    # Start thrust sending thread
    thrust_thread = Thread(target=send_control_loop)
    thrust_thread.start()

    # Start keyboard monitoring
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

    color_display.join()
    # depth_display.join()
    # underwater_display.join()
    # sonar_display.join()
    imu_logger.join()
    dvl_logger.join()
    position_logger.join()
    thrust_thread.join()
    print("Exiting code")
