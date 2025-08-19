import rospy
from sensor_msgs.msg import Imu, JointState
from IMU_wavelet_filter import get_denoised_gyro
import numpy as np

# Variabel global untuk menyimpan data sensor
accel = None
orientation = None
gyro_raw = None
joint_positions = None

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
    # Mendapatkan data gyroscope yang sudah difilter dengan memberikan input gyro_raw
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
    if accel is None or orientation is None or joint_positions is None or gyro_raw is None:
        rospy.loginfo("Waiting for sensor data...")
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

        # Penyesuaian posisi CoM berdasarkan data joint positions
        joint_id = servo["id"] - 1  # Karena indeks Python mulai dari 0
        joint_pos = joint_positions[joint_id] if joint_id < len(joint_positions) else 0

        # Menghitung posisi servo berdasarkan offset dan rotasi
        local_position = offset + np.array([0, 0, joint_pos])  # Pertimbangkan joint_pos di sumbu Z
        transformed_position = rotation_matrix.dot(local_position)

        weighted_sum += transformed_position * mass
        total_mass += mass

    # Hitung posisi CoM
    CoM_x, CoM_y, CoM_z = weighted_sum / total_mass
    rospy.loginfo("Calculated Dynamic CoM: x={}, y={}, z={}".format(CoM_x, CoM_y, CoM_z))

    return CoM_x, CoM_y, CoM_z

# Panggil fungsi dalam loop untuk estimasi CoM secara dinamis
if __name__ == "__main__":
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        calculate_dynamic_com()
        rate.sleep()
