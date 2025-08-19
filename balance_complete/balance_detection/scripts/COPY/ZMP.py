import rospy
from sensor_msgs.msg import Imu, JointState
from IMU_wavelet_filter import get_denoised_gyro
import numpy as np
import csv
import logging
import time

# Setup logger
logger = logging.getLogger('roslog')
logger.setLevel(logging.INFO)

# Setup CSV file handler for logging
csv_file = open('ros_logs.csv', 'wb')  # 'wb' untuk Python 2.7 agar tidak ada extra newline
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['timestamp', 'log_level', 'message'])

# Define custom logging handler to write to CSV
class CSVHandler(logging.Handler):
    def emit(self, record):
        msg = self.format(record)
        csv_writer.writerow([time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime(record.created)), record.levelname, msg])

# Add CSVHandler to logger
csv_handler = CSVHandler()
logger.addHandler(csv_handler)

# Variabel global untuk menyimpan data sensor
accel = None
orientation = None
gyro_raw = None
joint_positions = None
previous_com = None  # Untuk menyimpan posisi CoM sebelumnya
previous_time = None  # Untuk menghitung delta waktu

# Fungsi callback untuk data IMU
def imu_callback(msg):
    global accel, orientation, gyro_raw
    accel = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
    orientation = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
    gyro_raw = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)

# Fungsi callback untuk data posisi joint
def joint_state_callback(msg):
    global joint_positions
    joint_positions = msg.position

# Inisialisasi Node ROS dan langganan topik
rospy.init_node('dynamic_com_calculator', anonymous=True)
rospy.Subscriber("/robotis/open_cr/imu", Imu, imu_callback)
rospy.Subscriber("/robotis/present_joint_states", JointState, joint_state_callback)

# Fungsi untuk mendapatkan data gyroscope yang sudah difilter
def get_filtered_gyro(gyro_raw):
    denoised_gyro = get_denoised_gyro(gyro_raw)
    return denoised_gyro

# Data massa dan offset dari masing-masing servo
servo_data = [
    {"id": 20, "mass": 0.09, "CoM_offset": (0, 2.85, 45.1)},
    {"id": 3, "mass": 0.079, "CoM_offset": (-10.8, 1.3, 37.5)},
    {"id": 4, "mass": 0.079, "CoM_offset": (10.8, 1.3, 37.5)},
    {"id": 5, "mass": 0.031, "CoM_offset": (-23.4, 0.9, 37.5)},
    {"id": 6, "mass": 0.031, "CoM_offset": (23.4, 0.9, 37.5)},
    {"id": 1, "mass": 0.086, "CoM_offset": (-4.5, 1.8, 35.2)},
    {"id": 2, "mass": 0.086, "CoM_offset": (4.5, 1.8, 35.2)},
    {"id": 19, "mass": 1.179, "CoM_offset": (0, 1.8, 35.2)},
    {"id": 9, "mass": 0.082, "CoM_offset": (-3.2, -2.7, 23.5)},
    {"id": 11, "mass": 0.164, "CoM_offset": (-3.2, 1, 23.5)},
    {"id": 10, "mass": 0.082, "CoM_offset": (3.2, -2.7, 23.5)},
    {"id": 12, "mass": 0.246, "CoM_offset": (3.2, 1, 23.5)},
    {"id": 13, "mass": 0.139, "CoM_offset": (-3.2, 0.7, 16.4)},
    {"id": 14, "mass": 0.139, "CoM_offset": (3.2, 0.7, 16.4)},
    {"id": 17, "mass": 0.082, "CoM_offset": (-3.2, -2.7, 3.7)},
    {"id": 15, "mass": 0.2, "CoM_offset": (-3.2, 0.4, 3.7)},
    {"id": 16, "mass": 0.282, "CoM_offset": (3.2, 0.4, 3.7)}
]

def quaternion_to_rotation_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)]
    ])

def calculate_dynamic_com():
    global previous_com, previous_time
    if accel is None or orientation is None or joint_positions is None or gyro_raw is None:
        logger.info("Waiting for sensor data...")
        return None

    total_mass = 0
    weighted_sum = np.array([0.0, 0.0, 0.0])

    # Dapatkan data gyro yang sudah difilter
    gyro = get_filtered_gyro(gyro_raw)

    # Hitung matriks rotasi dari quaternion secara manual
    rotation_matrix = quaternion_to_rotation_matrix(orientation)

    # Perhitungan posisi CoM dinamis
    for servo in servo_data:
        mass = servo["mass"]
        offset = np.array(servo["CoM_offset"])

        joint_id = servo["id"] - 1
        joint_pos = joint_positions[joint_id] if joint_id < len(joint_positions) else 0

        local_position = offset + np.array([0, 0, joint_pos])
        transformed_position = rotation_matrix.dot(local_position)

        weighted_sum += transformed_position * mass
        total_mass += mass

    CoM_x, CoM_y, CoM_z = weighted_sum / total_mass
    logger.info("Calculated Dynamic CoM: x={}, y={}, z={}".format(CoM_x, CoM_y, CoM_z))

    current_time = rospy.get_time()
    if previous_com is not None and previous_time is not None:
        delta_time = current_time - previous_time
        velocity_com = (np.array([CoM_x, CoM_y, CoM_z]) - previous_com) / delta_time
    else:
        velocity_com = np.array([0.0, 0.0, 0.0])

    previous_com = np.array([CoM_x, CoM_y, CoM_z])
    previous_time = current_time

    projected_com = np.array([CoM_x, CoM_y, 0])
    logger.info("Projected CoM on the ground: x={}, y={}".format(projected_com[0], projected_com[1]))

    return CoM_x, CoM_y, CoM_z

def calculate_dynamic_zmp(CoM_x, CoM_y, CoM_z):
    global accel

    if accel is None:
        logger.info("Waiting for accelerometer data...")
        return None

    GRAVITY = 9.81
    F_z = accel[2] - GRAVITY

    if abs(F_z) < 1e-5:
        logger.info("F_z terlalu kecil, tidak bisa menghitung ZMP dengan valid")
        return None

    ddot_CoM_z = accel[2] - GRAVITY
    ZMP_x = CoM_x - (CoM_z / ddot_CoM_z) * accel[0]
    ZMP_y = CoM_y - (CoM_z / ddot_CoM_z) * accel[1]

    logger.info("Calculated Dynamic ZMP: x={}, y={}".format(ZMP_x, ZMP_y))

    return ZMP_x, ZMP_y

# Panggil fungsi dalam loop untuk estimasi CoM secara dinamis dan menghitung ZMP
if __name__ == "__main__":
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        CoM = calculate_dynamic_com()
        if CoM is not None and accel is not None:
            ZMP = calculate_dynamic_zmp(CoM[0], CoM[1], CoM[2])
            if ZMP is not None:
                # Simpan ke CSV file atau lakukan tindakan lain jika dibutuhkan
                pass
        rate.sleep()

csv_file.close()
