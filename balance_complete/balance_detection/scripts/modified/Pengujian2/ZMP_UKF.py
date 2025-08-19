import rospy
from sensor_msgs.msg import Imu, JointState
from IMU_UKF import ukf_denoise  # Mengimpor fungsi UKF dari IMU_UKF.py
import numpy as np

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
    # (data servo lainnya)
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
    process_noise = np.eye(3) * 0.01
    measurement_noise = np.eye(3) * 0.1
    dt = 0.01

    # Panggil fungsi UKF untuk mendenoise data gyro
    denoised_gyro = ukf_denoise(gyro_raw, process_noise, measurement_noise, dt)
    return denoised_gyro[-1]  # Mengambil hasil denoised terbaru

# Pemetaan ID ke index 'name' saat ini
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

        # Ambil posisi joint berdasarkan ID servo dan peta ID ke indeks joint positions
        servo_id = servo["id"]
        joint_index = id_to_index_map.get(servo_id)

        # Cek apakah indeks valid
        if joint_index is not None and joint_index < len(joint_positions):
            joint_pos = joint_positions[joint_index]
        else:
            joint_pos = 0

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

    return CoM_x, CoM_y, CoM_z

# Parameter tambahan untuk perhitungan gaya
FOOT_BASE_WIDTH = 0.08  # Lebar basis kaki (m)
FOOT_BASE_LENGTH = 0.123  # Panjang basis kaki (m)

def calculate_dynamic_zmp(CoM_x, CoM_y, CoM_z):
    global accel

    if accel is None:
        rospy.loginfo("Waiting for accelerometer data...")
        return None

    GRAVITY = 9.81
    F_z = accel[2] - GRAVITY

    total_mass = sum(servo["mass"] for servo in servo_data)
    F_normal = total_mass * GRAVITY

    # Tentukan koefisien gesek, sesuaikan sesuai dengan permukaan yang digunakan
    mu = 0.5  # Misalnya 0.5 untuk permukaan sintetis
    F_friction = mu * F_normal

    # Memastikan F_z tidak mendekati nol
    if abs(F_z) < 1e-5:
        rospy.loginfo("F_z terlalu kecil, tidak bisa menghitung ZMP dengan valid")
        return None

    # # Perbaiki perhitungan ZMP
    # ZMP_x = CoM_x - (CoM_z / (F_normal / total_mass)) * accel[0]
    # ZMP_y = CoM_y - (CoM_z / (F_normal / total_mass)) * accel[1]

    # Menghitung momen berdasarkan gaya gesek
    moment_x = F_friction * (FOOT_BASE_WIDTH / 2)
    moment_y = F_friction * (FOOT_BASE_LENGTH / 2)

    ZMP_x = CoM_x - (CoM_z / F_normal) * (accel[0] + moment_y / F_normal)
    ZMP_y = CoM_y - (CoM_z / F_normal) * (accel[1] - moment_x / F_normal)

    rospy.loginfo("Calculated Moments: Mx={}, My={}".format(moment_x, moment_y))
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

# Fungsi untuk mendeteksi fase dukungan (Single dan Double Support saja)
def detect_support_phase(joint_states, imu_accel_z):
    global last_phase  # Gunakan variabel global untuk melacak fase terakhir
    l_ank_pitch = np.array([state[id_to_index_map[13]] for state in joint_states])  # l_ank_pitch
    r_ank_pitch = np.array([state[id_to_index_map[14]] for state in joint_states])  # r_ank_pitch

    phases = []
    for l_ank, r_ank, accel in zip(l_ank_pitch, r_ank_pitch, imu_accel_z):
        if l_ank > 0 and r_ank > 0:  # Kedua sendi positif
            current_phase = 'Double Support'
        elif l_ank <= 0 and r_ank <= 0:  # Kedua sendi negatif
            current_phase = 'Single Support'
        else:
            if accel < -0.4:  # Threshold untuk mendeteksi ketika kaki mengangkat
                current_phase = 'Single Support'
            else:
                current_phase = 'Double Support'

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
            imu_accel_z = accel[1]  # Akselerasi sumbu Y dari IMU
            joint_states = [joint_positions]  # Buat list joint state untuk input
            support_phase = detect_support_phase(joint_states, [imu_accel_z])

            # Update koordinat titik berdasarkan fase dukungan
            points_single_support = np.array([
                [0, 0],
                [0, -12.3],
                [1.1, -13],
                [7.1, -13],
                [8.7, -12.3],
                [8.7, 0]
            ])

            points_double_support = np.array([
                [0, 0],
                [0, -12.3],
                [1.1, -13],
                [16.6, -13],
                [17.7, -12.3],
                [17.7, 0]
            ])

            current_phase = support_phase[0]  # Ambil fase saat ini
            if current_phase == 'Single Support':
                points = points_single_support
            else:
                # Double support
                points = points_double_support

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
