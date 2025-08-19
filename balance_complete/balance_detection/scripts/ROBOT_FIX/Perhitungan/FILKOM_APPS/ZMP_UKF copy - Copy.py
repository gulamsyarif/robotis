import rospy
from sensor_msgs.msg import Imu, JointState
from IMU_UKF import ukf_denoise  # Mengimpor fungsi UKF dari IMU_UKF.py
import numpy as np
import time

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
    gyro_raw = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

# Fungsi callback untuk data posisi joint
def joint_state_callback(msg):
    global joint_positions
    joint_positions = msg.position

# Inisialisasi Node ROS dan langganan topik
rospy.init_node('dynamic_com_calculator', anonymous=True)
rospy.Subscriber("/robotis/open_cr/imu", Imu, imu_callback)
rospy.Subscriber("/robotis/present_joint_states", JointState, joint_state_callback)

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
    {"id": 18, "mass": 0.082, "CoM_offset": (3.2, -2.7, 3.7)},
    {"id": 15, "mass": 0.282, "CoM_offset": (-3.2, 0.4, 3.7)},
    {"id": 16, "mass": 0.282, "CoM_offset": (3.2, 0.4, 3.7)},
    {"id": 7, "mass": 0.086, "CoM_offset": (-4.5, 1.8, 28.4)},
    {"id": 8, "mass": 0.086, "CoM_offset": (4.5, 1.8, 28.4)}
]

def quaternion_to_rotation_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)]
    ])

# Fungsi untuk mendapatkan data gyroscope yang sudah difilter menggunakan UKF
def get_filtered_gyro(gyro_raw):
    process_noise = np.eye(3) * 0.1
    measurement_noise = np.eye(3) * 0.1
    dt = 0.01

    # Panggil fungsi UKF untuk mendenoise data gyro
    denoised_gyro = ukf_denoise(gyro_raw, process_noise, measurement_noise, dt)
    return denoised_gyro[-1]  # Mengambil hasil denoised terbaru

# Pemetaan ID ke index 'name' saat ini genap kiri, ganjil kanan
id_to_index_map = {
    1: 18,  # r_sho_pitch   9,   # l_sho_pitch
    2: 9,   # l_sho_pitch   18,  # r_sho_pitch 
    3: 19,  # r_sho_roll    10,  # l_sho_roll  
    4: 10,  # l_sho_roll    19,  # r_sho_roll
    5: 13,  # r_el          4,   # l_el  
    6: 4,   # l_el          13,  # r_el
    7: 16, # r_hip_yaw      6,   # l_hip_roll
    8: 7, # l_hip_yaw       15,  # r_hip_roll
    9: 15,  # r_hip_roll    5,   # l_hip_pitch
    10: 6,   # l_hip_roll   14, # r_hip_pitch 
    11: 14, # r_hip_pitch   8,  # l_knee
    12: 5,   # l_hip_pitch  17, # r_knee 
    13: 17, # r_knee        2,  # l_ank_pitch7
    14: 8,  # l_knee        11, # r_ank_pitch
    15: 11,  # r_ank_pitch  3,  # l_ank_roll
    16: 2,  # l_ank_pitch   12, # r_ank_roll 
    17: 12, # r_ank_roll    7, # l_hip_yaw
    18: 3,  # l_ank_roll    16, # r_hip_yaw 
    19: 1,  # head_tilt
    20: 0,  # head_pan
}

def calculate_dynamic_com():
    global previous_com, previous_time
    if accel is None or orientation is None or joint_positions is None or gyro_raw is None:
        rospy.loginfo("Waiting for sensor data...")
        return None
    
    start_time = time.time()

    total_mass = 0
    weighted_sum = np.array([0.0, 0.0, 0.0])

    # Dapatkan data gyro yang sudah difilter
    gyro = get_filtered_gyro(gyro_raw)

    # Hitung matriks rotasi dari quaternion
    rotation_matrix = quaternion_to_rotation_matrix(orientation)

    # Hitung delta_time dengan kondisi awal
    current_time = time.time()
    delta_time = current_time - previous_time if previous_time is not None else 0
    previous_time = current_time

    # Buat matriks rotasi tambahan berdasarkan data gyroscope
    if delta_time > 0:
        rotation_adjustment = np.identity(3) + np.cross(np.identity(3), gyro_raw) * delta_time
    else:
        rotation_adjustment = np.identity(3)  # Gunakan matriks identitas jika `delta_time` masih nol

    # Gabungkan matriks rotasi dengan koreksi gyroscope
    adjusted_rotation_matrix = np.dot(rotation_matrix, rotation_adjustment)

    for servo in servo_data:
        mass = servo["mass"]
        offset = np.array(servo["CoM_offset"])
        servo_id = servo["id"]
        joint_index = id_to_index_map.get(servo_id)
        joint_pos = joint_positions[joint_index] if joint_index is not None and joint_index < len(joint_positions) else 0

        local_position = offset + np.array([0, 0, joint_pos])
        transformed_position = adjusted_rotation_matrix.dot(local_position)
        weighted_sum += transformed_position * mass
        total_mass += mass
        
    computation_time = time.time() - start_time  # Hitung durasi kalkulasi
    rospy.loginfo("CoM Computation Time: {:.6f} seconds".format(computation_time))

    # Hitung posisi CoM
    CoM_x, CoM_y, CoM_z = weighted_sum / total_mass
    rospy.loginfo("Calculated Dynamic CoM: x={}, y={}, z={}".format(CoM_x, CoM_y, CoM_z))
    rospy.loginfo("Projected CoM on the ground: x={}, y={}, z=0".format(CoM_x, CoM_y))
    
    return CoM_x, CoM_y, CoM_z

# Parameter tambahan untuk perhitungan gaya
FOOT_BASE_WIDTH = 0.08  # Lebar basis kaki (m)
FOOT_BASE_LENGTH = 0.123  # Panjang basis kaki (m)

def calculate_dynamic_zmp(CoM_x, CoM_y, CoM_z):
    global accel

    if accel is None:
        rospy.loginfo("Waiting for accelerometer data...")
        return None
    
    start_time = time.time()

    GRAVITY = 9.81

    # Gaya total berdasarkan percepatan dan massa robot
    F_x = total_mass * accel[0]  # Gaya horizontal pada sumbu X
    F_y = total_mass * accel[1]  # Gaya horizontal pada sumbu Y
    F_z = total_mass * (accel[2] - GRAVITY)  # Gaya vertikal

    # Memastikan F_z tidak mendekati nol
    if abs(F_z) < 1e-5:
        rospy.loginfo("F_z terlalu kecil, tidak bisa menghitung ZMP dengan valid")
        return None

    # Jarak antara CoM dan ZMP (d_x, d_y)
    d_x = CoM_x
    d_y = CoM_y

    # Menghitung momen
    moment_x = F_z * d_y - F_y * CoM_z  # M_x
    moment_y = F_x * CoM_z - F_z * d_x  # M_y

    # Log hasil perhitungan
    rospy.loginfo("Calculated Moments: Mx={}, My={}".format(moment_x, moment_y))

    # Menghitung ZMP dengan momen total
    ZMP_x = CoM_x - (CoM_z / F_z) * (accel[0] + moment_y / F_z)
    ZMP_y = CoM_y - (CoM_z / F_z) * (accel[1] - moment_x / F_z)

    # Log hasil ZMP
    rospy.loginfo("Calculated Dynamic ZMP: x={}, y={}".format(ZMP_x, ZMP_y))

    computation_time = time.time() - start_time  # Akhiri pengukuran waktu
    rospy.loginfo("ZMP Calculation Time: {:.6f} seconds".format(computation_time))
    return ZMP_x, ZMP_y

def calculate_support_polygon(joint_states, imu_accel_z):
    # Mapping id joint
    start_time = time.time()
    l_ank_pitch = np.array([state[id_to_index_map[13]] for state in joint_states])
    r_ank_pitch = np.array([state[id_to_index_map[14]] for state in joint_states])

    # Deteksi fase dukungan
    if l_ank_pitch[0] > 0 and r_ank_pitch[0] > 0:
        current_phase = 'Double Support'
    elif l_ank_pitch[0] <= 0 and r_ank_pitch[0] <= 0:
        current_phase = 'Single Support'
    else:
        if imu_accel_z < -0.35:
            current_phase = 'Single Support'
        else:
            current_phase = 'Double Support'

    # Tentukan koordinat berdasarkan fase
    if current_phase == 'Single Support':
        points = np.array([
            [0, 0],
            [0, -12.3],
            [1.1, -13],
            [7.1, -13],
            [8.7, -12.3],
            [8.7, 0]
        ])
    else:  # Double Support
        points = np.array([
            [0, 0],
            [0, -12.3],
            [1.1, -13],
            [16.6, -13],
            [17.7, -12.3],
            [17.7, 0]
        ])

    # Hitung luas dan keliling
    x = points[:, 0]
    y = points[:, 1]
    area = 0.5 * np.abs(np.dot(x[:-1], y[1:]) - np.dot(y[:-1], x[1:]))
    perimeter = np.sum(np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1)))

    computation_time = time.time() - start_time  # Akhiri pengukuran waktu
    rospy.loginfo("Support Polygon Calculation Time: {:.6f} seconds".format(computation_time))

    return {
        "phase": current_phase,
        "area": area,
        "perimeter": perimeter
    }

def main():
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        com_data = calculate_dynamic_com()
        if com_data:
            CoM_x, CoM_y, CoM_z = com_data
            calculate_dynamic_zmp(CoM_x, CoM_y, CoM_z)

        # Deteksi fase dukungan dan hitung polygon
        if joint_positions is not None and accel is not None:
            imu_accel_z = accel[1]  # Akselerasi sumbu Y dari IMU
            joint_states = [joint_positions]  # Buat list joint state untuk input
            support_polygon_data = calculate_support_polygon(joint_states, imu_accel_z)

            # Log hasil perhitungan
            rospy.loginfo(
                "Support Phase: {}, Area: {}, Perimeter: {}".format(
                    support_polygon_data["phase"],
                    support_polygon_data["area"],
                    support_polygon_data["perimeter"]
                )
            )

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass