# #!/usr/bin/env python
# -*- coding: utf-8 -*-
# 
# import numpy as np
# import pandas as pd
# import ast
# import matplotlib.pyplot as plt
# from matplotlib.patches import Polygon

# # Fungsi menghitung luas menggunakan Shoelace Theorem
# def calculate_area(coords):
#     x = coords[:, 0]
#     y = coords[:, 1]
#     return 0.5 * np.abs(np.dot(x[:-1], y[1:]) - np.dot(y[:-1], x[1:]))

# # Fungsi menghitung keliling
# def calculate_perimeter(coords):
#     return np.sum(np.sqrt(np.sum(np.diff(coords, axis=0)**2, axis=1)))

# # Membaca data joint states dan IMU
# joint_data = pd.read_csv(r'C:\Users\syari\Downloads\balance_detection\balance_detection\scripts\DONE\data_lagi\joint_states.csv')
# imu_data = pd.read_csv(r'C:\Users\syari\Downloads\balance_detection\balance_detection\scripts\DONE\data_lagi\initial_imu_data.csv')

# # Fungsi untuk mendeteksi fase single/double support dan update polygon
# def update_polygon(joint_state_str, imu_state):
#     # Koordinat awal (contoh) kaki-kaki robot
#     points_double_support = np.array([
#         [0, 0],        # Kaki kiri depan
#         [0, -12.3],    # Kaki kiri belakang
#         [1.1, -13],    # Kaki kanan belakang
#         [16.6, -13],   # Kaki kanan belakang jauh
#         [17.7, -12.3], # Kaki kanan depan
#         [17.7, 0]      # Kembali ke kaki kanan depan
#     ])

#     # Parsing joint state dari string ke tuple (tuple dalam string format)
#     joint_state = np.array(ast.literal_eval(joint_state_str))

#     # Contoh ambang batas deteksi kaki terangkat (bisa disesuaikan dengan data sebenarnya)
#     left_leg_lifted = joint_state[0] > 0.5  # Misal, jika joint 0 menunjukkan kaki kiri terangkat
#     right_leg_lifted = joint_state[3] > 0.5  # Misal, jika joint 3 menunjukkan kaki kanan terangkat

#     if left_leg_lifted and not right_leg_lifted:
#         # Single support pada kaki kanan
#         points_single_support = points_double_support[2:]  # Hanya kaki kanan
#         points = points_single_support
#     elif right_leg_lifted and not left_leg_lifted:
#         # Single support pada kaki kiri
#         points_single_support = points_double_support[:3]  # Hanya kaki kiri
#         points = points_single_support
#     else:
#         # Double support (keduanya di tanah)
#         points = points_double_support

#     # Update koordinat berdasarkan IMU data
#     points[:, 0] += joint_state[:len(points)] * 0.1  # Sesuaikan perubahan x berdasarkan posisi joint
#     points[:, 1] += imu_state['Orientation_y'] * 0.1  # Sesuaikan perubahan y berdasarkan orientasi (pitch dari IMU)

#     return points

# # Menentukan jumlah baris minimal antara joint_data dan imu_data
# num_rows = min(len(joint_data), len(imu_data))

# # List untuk menyimpan hasil perhitungan
# results = []

# # Loop melalui setiap baris data (hanya sampai jumlah baris minimum)
# for i in range(num_rows):
#     joint_state_str = joint_data.iloc[i]['Joint States']
#     imu_state = imu_data.iloc[i]

#     # Update posisi polygon berdasarkan joint dan imu
#     points = update_polygon(joint_state_str, imu_state)

#     # Menambahkan titik pertama di akhir untuk perhitungan shoelace dan keliling
#     points = np.vstack([points, points[0]])

#     # Menghitung luas dan keliling
#     area = calculate_area(points)
#     perimeter = calculate_perimeter(points)

#     # Simpan hasil dalam list
#     results.append({
#         'Time': joint_data.iloc[i]['Time'],
#         'Area': area,
#         'Perimeter': perimeter
#     })

# # Membuat DataFrame dari hasil perhitungan
# results_df = pd.DataFrame(results)

# # Simpan hasil ke file CSV baru
# results_df.to_csv(r'C:\Users\syari\Downloads\support_polygon_results.csv', index=False)

# # Plot hanya untuk baris terakhir
# fig, ax = plt.subplots()
# polygon = Polygon(points[:-1], closed=True, edgecolor='b', fill=True, alpha=0.3)

# # Menambahkan polygon ke plot
# ax.add_patch(polygon)
# ax.plot(points[:, 0], points[:, 1], 'ro-')  # Plot titik-titik koordinat

# # Atur tampilan plot
# ax.set_xlim(-2, 20)
# ax.set_ylim(-15, 5)
# ax.set_aspect('equal', 'box')
# ax.set_title(f"Support Polygon Area = {area:.2f} units²\nPerimeter = {perimeter:.2f} units")
# ax.set_xlabel("X-axis")
# ax.set_ylabel("Y-axis")

# # Tampilkan plot
# plt.grid(True)
# plt.show()

# print("Perhitungan selesai. Hasil disimpan di 'support_polygon_results.csv'.")

import rospy
from sensor_msgs.msg import Imu, JointState
from IMU_wavelet_filter import get_denoised_gyro
import numpy as np
import ast

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

# Mapping ID ke index 'name'
id_to_index_map = {
    1: 9,   # l_sho_pitch
    2: 18,  # r_sho_pitch
    3: 10,  # l_sho_roll
    4: 19,  # r_sho_roll
    5: 4,   # l_el
    6: 13,  # r_el
    7: 6,   # l_hip_roll
    8: 15,  # r_hip_roll
    9: 5,   # l_hip_pitch
    10: 14, # r_hip_pitch
    11: 8,  # l_knee
    12: 17, # r_knee
    13: 2,  # l_ank_pitch
    14: 11, # r_ank_pitch
    15: 3,  # l_ank_roll
    16: 12, # r_ank_roll
    19: 0,  # head_pan
    20: 1,  # head_tilt
}

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

        servo_id = servo["id"]
        joint_index = id_to_index_map.get(servo_id)

        if joint_index is not None and joint_index < len(joint_positions):
            joint_pos = joint_positions[joint_index]
        else:
            joint_pos = 0

        local_position = offset + np.array([0, 0, joint_pos])  
        transformed_position = rotation_matrix.dot(local_position)

        weighted_sum += transformed_position * mass
        total_mass += mass

    CoM_x, CoM_y, CoM_z = weighted_sum / total_mass
    rospy.loginfo("Calculated Dynamic CoM: x={}, y={}, z={}".format(CoM_x, CoM_y, CoM_z))

    current_time = rospy.get_time()
    if previous_com is not None and previous_time is not None:
        delta_time = current_time - previous_time
        velocity_com = (np.array([CoM_x, CoM_y, CoM_z]) - previous_com) / delta_time
    else:
        velocity_com = np.array([0.0, 0.0, 0.0])

    previous_com = np.array([CoM_x, CoM_y, CoM_z])
    previous_time = current_time

    projected_com = np.array([CoM_x, CoM_y, 0])  
    rospy.loginfo("Projected CoM on the ground: x={}, y={}, z=0".format(projected_com[0], projected_com[1]))

    return CoM_x, CoM_y, CoM_z

def calculate_dynamic_zmp(CoM_x, CoM_y, CoM_z):
    global accel

    if accel is None:
        rospy.loginfo("Waiting for accelerometer data...")
        return None

    GRAVITY = 9.81
    F_z = accel[2] - GRAVITY

    if abs(F_z) < 1e-5:
        rospy.loginfo("F_z terlalu kecil, tidak bisa menghitung ZMP dengan valid")
        return None

    ddot_CoM_z = accel[2] - GRAVITY

    ZMP_x = CoM_x - (CoM_z / ddot_CoM_z) * accel[0]
    ZMP_y = CoM_y - (CoM_z / ddot_CoM_z) * accel[1]

    rospy.loginfo("Calculated Dynamic ZMP: x={}, y={}".format(ZMP_x, ZMP_y))

    return ZMP_x, ZMP_y

# Fungsi menghitung luas menggunakan Shoelace Theorem 
def calculate_area(coords):
    x = coords[:, 0]
    y = coords[:, 1]
    return 0.5 * np.abs(np.dot(x[:-1], y[1:]) - np.dot(y[:-1], x[1:]))

# Fungsi menghitung keliling
def calculate_perimeter(coords):
    return np.sum(np.sqrt(np.sum(np.diff(coords, axis=0)**2, axis=1)))

# Fungsi untuk mendeteksi fase dukungan
def detect_support_phase(joint_states, imu_accel_y):
    global last_phase  # Gunakan variabel global untuk melacak fase terakhir
    l_ank_pitch = np.array([state[id_to_index_map[13]] for state in joint_states])  # l_ank_pitch
    r_ank_pitch = np.array([state[id_to_index_map[14]] for state in joint_states])  # r_ank_pitch

    phases = []
    for l_ank, r_ank, accel in zip(l_ank_pitch, r_ank_pitch, imu_accel_y):
        if l_ank > 0 and r_ank > 0:  # Kedua sendi positif
            current_phase = 'Double Support'
        elif l_ank <= 0 and r_ank <= 0:  # Kedua sendi negatif
            current_phase = 'Single Support'
        else:
            if accel < -0.5:  # Threshold untuk mendeteksi ketika kaki mengangkat
                if l_ank <= 0:
                    current_phase = 'Right Single Support'
                else:
                    current_phase = 'Left Single Support'
            else:
                current_phase = 'Double Support'

        if current_phase.startswith('Single Support'):
            if last_phase == 'Right Single Support':
                current_phase = 'Left Single Support'
            elif last_phase == 'Left Single Support':
                current_phase = 'Right Single Support'

        phases.append(current_phase)
        last_phase = current_phase  # Update last_phase untuk iterasi berikutnya

    return phases

# Fungsi utama
def main():
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        com_data = calculate_dynamic_com()
        if com_data:
            CoM_x, CoM_y, CoM_z = com_data
            calculate_dynamic_zmp(CoM_x, CoM_y, CoM_z)

        # Deteksi fase dukungan dan hitung luas + keliling support polygon
        if joint_positions is not None and accel is not None:
            imu_accel_y = accel[1]  # Akselerasi sumbu Y dari IMU
            joint_states = [joint_positions]  # Buat list joint state untuk input
            support_phase = detect_support_phase(joint_states, [imu_accel_y])

            # Update koordinat titik berdasarkan fase dukungan
            points_right_single_support = np.array([
                [9.5, 0],
                [9.5, -12.3],
                [10.6, -13],
                [16.6, -13],
                [17.7, -12.3],
                [17.7, 0]
            ])
            points_left_single_support = np.array([
                [0, 0],
                [0, -12.3],
                [1.1, -13],
                [7.1, -13],
                [8.7, -12.3],
                [8.7, 0]
            ])

            current_phase = support_phase[0]  # Ambil fase saat ini
            if current_phase == 'Left Single Support':
                points = points_left_single_support
            elif current_phase == 'Right Single Support':
                points = points_right_single_support
            else:
                # Double support
                points = np.array([
                    [0, 0],
                    [0, -12.3],
                    [1.1, -13],
                    [16.6, -13],
                    [17.7, -12.3],
                    [17.7, 0]
                ])

            # Perhitungan area dan perimeter dari polygon
            area = calculate_area(points)
            perimeter = calculate_perimeter(points)

            rospy.loginfo("Support Phase: {}, Area: {}, Perimeter: {}".format(current_phase, area, perimeter))

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
