import rospy
from sensor_msgs.msg import Imu, JointState
from IMU_wavelet_filter import get_denoised_gyro
import numpy as np
import csv

# Variabel global untuk menyimpan data sensor
accel = None
orientation = None
gyro_raw = None
joint_positions = None
previous_com = None  # Untuk menyimpan posisi CoM sebelumnya
previous_time = None  # Untuk menghitung delta waktu

# File CSV untuk menyimpan data
log_file_path = "robot_data_log.csv"

# Inisialisasi file CSV
with open(log_file_path, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Time", "Accel_X", "Accel_Y", "Accel_Z",
                     "Orientation_X", "Orientation_Y", "Orientation_Z", "Orientation_W",
                     "Gyro_X", "Gyro_Y", "Gyro_Z",
                     "CoM_X", "CoM_Y", "CoM_Z",
                     "Velocity_X", "Velocity_Y", "Velocity_Z",
                     "ZMP_X", "ZMP_Y"])

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
        rospy.loginfo("Waiting for sensor data...")
        return None

    total_mass = 0
    weighted_sum = np.array([0.0, 0.0, 0.0])

    # Dapatkan data gyro yang sudah difilter
    gyro = get_filtered_gyro(gyro_raw)

    # Hitung matriks rotasi dari quaternion
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

    # Hitung kecepatan CoM
    current_time = rospy.get_time()
    if previous_com is not None and previous_time is not None:
        delta_time = current_time - previous_time
        velocity_com = (np.array([CoM_x, CoM_y, CoM_z]) - previous_com) / delta_time
    else:
        velocity_com = np.array([0.0, 0.0, 0.0])  # Jika ini pertama kali, kecepatan dianggap nol

    previous_com = np.array([CoM_x, CoM_y, CoM_z])
    previous_time = current_time

    # Proyeksi CoM pada permukaan bawah
    projected_com = np.array([CoM_x, CoM_y, 0])  # Misalkan permukaan bawah adalah z=0
    rospy.loginfo("Projected CoM on the ground: x={}, y={}, z=0".format(projected_com[0], projected_com[1]))

    return CoM_x, CoM_y, CoM_z, velocity_com

def calculate_dynamic_zmp(CoM_x, CoM_y, CoM_z):
    global accel

    if accel is None:
        rospy.loginfo("Waiting for accelerometer data...")
        return None

    GRAVITY = 9.81
    F_z = accel[2] - GRAVITY

    total_mass = sum(servo["mass"] for servo in servo_data)
    F_normal = total_mass * GRAVITY

    # Tentukan koefisien gesek
    mu = 0.5  # Misalnya 0.5 untuk permukaan sintetis
    F_friction = mu * F_normal

    # Memastikan F_z tidak mendekati nol
    if abs(F_z) < 1e-6:
        F_z = 1e-6

    # Hitung ZMP
    ZMP_x = CoM_x - (F_friction / F_z) * (CoM_y)
    ZMP_y = CoM_y + (F_friction / F_z) * (CoM_x)

    return ZMP_x, ZMP_y

# Hitung data dan simpan ke CSV
def main_loop():
    count = 0
    while not rospy.is_shutdown():
        com_values = calculate_dynamic_com()
        if com_values is not None:
            CoM_x, CoM_y, CoM_z, velocity_com = com_values
            zmp_values = calculate_dynamic_zmp(CoM_x, CoM_y, CoM_z)
            if zmp_values is not None:
                ZMP_x, ZMP_y = zmp_values

                # Simpan data ke file CSV
                with open(log_file_path, mode='a') as file:
                    writer = csv.writer(file)
                    writer.writerow([rospy.get_time()] + list(accel) + list(orientation) + list(gyro_raw) +
                                    [CoM_x, CoM_y, CoM_z] +
                                    list(velocity_com) + [ZMP_x, ZMP_y])

                count += 1
                rospy.loginfo("Data logged. Total entries: {}".format(count))

                if count >= 15000:
                    rospy.loginfo("Reached 15,000 entries, stopping...")
                    break

        rospy.sleep(0.1)  # Tidur sejenak untuk mengurangi beban CPU

if __name__ == "__main__":
    main_loop()