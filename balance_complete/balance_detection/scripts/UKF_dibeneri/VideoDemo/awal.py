import rospy
from sensor_msgs.msg import Imu, JointState
from IMU_UKF import ukf_denoise  # Mengimpor fungsi UKF dari IMU_UKF.py
import numpy as np
import time
import matplotlib.pyplot as plt


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

    # Menghitung momen berdasarkan gaya gesek
    moment_x = F_friction * (FOOT_BASE_WIDTH / 2)
    moment_y = F_friction * (FOOT_BASE_LENGTH / 2)

    ZMP_x = CoM_x - (CoM_z / F_normal) * (accel[0] + moment_y / F_normal)
    ZMP_y = CoM_y - (CoM_z / F_normal) * (accel[1] - moment_x / F_normal)

    rospy.loginfo("Calculated Moments: Mx={}, My={}".format(moment_x, moment_y))
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
    
# Fungsi untuk menghitung data CoM, ZMP, dan Support Phase
def calculate_data():
    # Menghitung CoM secara dinamis
    com_data = calculate_dynamic_com()
    if com_data:
        CoM_x, CoM_y, CoM_z = com_data

        # Menghitung ZMP
        zmp_data = calculate_dynamic_zmp(CoM_x, CoM_y, CoM_z)

        # Mendapatkan fase dukungan dan menghitung support polygon
        if joint_positions is not None and accel is not None:
            imu_accel_z = accel[1]  # Akselerasi sumbu Y dari IMU
            joint_states = [joint_positions]  # Buat list joint state untuk input
            support_polygon_data = calculate_support_polygon(joint_states, imu_accel_z)

            # Mengembalikan data dalam format yang diminta
            return [{
                'com_x': CoM_x,
                'com_y': CoM_y,
                'zmp_x': zmp_data[0] if zmp_data else None,
                'zmp_y': zmp_data[1] if zmp_data else None,
                'support_phase': support_polygon_data["phase"]
            }]
    return []

def evaluate_balance(CoM_x, CoM_y, ZMP_x, ZMP_y):
    # Definisikan batas Support Polygon berdasarkan fase dukungan
    if support_phase == 'Double Support':
        x_min, x_max = -8.6, 8.6
        y_min, y_max = -6.5, 6.5
    elif support_phase == 'Single Support':
        x_min, x_max = -4.3, 4.3
        y_min, y_max = -6.5, 6.5
    else:
        rospy.logwarn("Unknown support phase, assuming Double Support.")
        x_min, x_max = -8.6, 8.6
        y_min, y_max = -6.5, 6.5

    # Periksa posisi ZMP dan CoM terhadap Support Polygon
    zmp_inside = x_min <= ZMP_x <= x_max and y_min <= ZMP_y <= y_max
    com_inside = x_min <= CoM_x <= x_max and y_min <= CoM_y <= y_max
    
    # Hitung jarak ZMP dan CoM dari batas Support Polygon
    zmp_distance = max(abs(ZMP_x - x_min), abs(ZMP_x - x_max), abs(ZMP_y - y_min), abs(ZMP_y - y_max))
    com_distance = max(abs(CoM_x - x_min), abs(CoM_x - x_max), abs(CoM_y - y_min), abs(CoM_y - y_max))

    if zmp_inside and com_inside:
        status = "Robot Seimbang"
    elif zmp_inside and not com_inside:
        status = "Robot Tidak Seimbang (CoM di luar, ZMP di dalam)"
    elif not zmp_inside and com_inside:
        status = "Robot Tidak Seimbang (ZMP di luar, CoM di dalam)"
    elif not zmp_inside and not com_inside:
        if zmp_distance < 5 and com_distance < 5:
            status = "Robot Tidak Seimbang (ZMP dan CoM di luar, <5m)"
        else:
            status = "Robot Tidak Seimbang (ZMP dan CoM di luar, >5m)"
    else:
        status = "Robot Tidak Seimbang (Kondisi Tidak Terdefinisi)"

    rospy.loginfo("Balance Status: {}".format(status))
    return status

# Fungsi untuk melakukan plotting secara real-time
def real_time_plot():
    # Set up matplotlib figure dan axis
    fig, axs = plt.subplots(1, 2, figsize=(12, 6))

    # Inisialisasi variabel untuk menyimpan data real-time
    com_x_data = []
    com_y_data = []
    zmp_x_data = []
    zmp_y_data = []
    support_phase_data = []

    # Tentukan batasan untuk support polygon
    x_min = -4.3
    x_max = 4.3
    y_min = -6.5
    y_max = 6.5

    # Fungsi untuk update plot real-time
    def update_plot():
        # Ambil data terbaru dari calculate_data
        data = calculate_data()

        if data:
            # Ekstrak data CoM, ZMP, dan support phase
            com_x = data[0]['com_x']
            com_y = data[0]['com_y']
            zmp_x = data[0]['zmp_x']
            zmp_y = data[0]['zmp_y']
            support_phase = data[0]['support_phase']

            # Update data real-time
            com_x_data.append(com_x)
            com_y_data.append(com_y)
            zmp_x_data.append(zmp_x)
            zmp_y_data.append(zmp_y)
            support_phase_data.append(support_phase)

            # Tentukan x_min dan x_max berdasarkan support phase
            if support_phase == 'Double Support':
                x_min = -8.6
                x_max = 8.6
            else:
                x_min = -4.3
                x_max = 4.3

            # Plot data CoM dan ZMP pada sumbu X
            axs[0].cla()  # Clear axis
            axs[0].fill_between(range(len(com_x_data)), x_min, x_max, color='lightblue', label='Support Polygon Area')
            axs[0].plot(com_x_data, color='green', linestyle='--', linewidth=2, label='CoM')
            axs[0].plot(zmp_x_data, color='red', linewidth=2, label='ZMP')
            axs[0].set_title('CoM & ZMP X-axis', fontsize=14)
            axs[0].set_xlabel('Data [index]')
            axs[0].set_ylabel('Nilai (cm)')
            axs[0].legend(loc='upper right')

            # Plot data CoM dan ZMP pada sumbu Y
            axs[1].cla()  # Clear axis
            axs[1].fill_between(range(len(com_y_data)), y_min, y_max, color='lightblue', label='Support Polygon Area')
            axs[1].plot(com_y_data, color='green', linestyle='--', linewidth=2, label='CoM')
            axs[1].plot(zmp_y_data, color='red', linewidth=2, label='ZMP')
            axs[1].set_title('CoM & ZMP Y-axis', fontsize=14)
            axs[1].set_xlabel('Data [index]')
            axs[1].set_ylabel('Nilai (cm)')
            axs[1].legend(loc='upper right')

            plt.draw()

        # Tambahkan delay untuk real-time plotting
        rospy.sleep(0.1)

    # Loop untuk update plot secara real-time
    while not rospy.is_shutdown():
        update_plot()

def real_time_plot():
    # Set up matplotlib figure dan axis
    fig, axs = plt.subplots(1, 2, figsize=(12, 6))

    # Inisialisasi variabel untuk menyimpan data real-time
    com_x_data = []
    com_y_data = []
    zmp_x_data = []
    zmp_y_data = []
    support_phase_data = []

    # Tentukan batasan untuk support polygon
    x_min = -4.3
    x_max = 4.3
    y_min = -6.5
    y_max = 6.5

    # Fungsi untuk update plot real-time
    def update_plot():
        # Ambil data terbaru dari calculate_data
        data = calculate_data()

        if data:
            # Ekstrak data CoM, ZMP, dan support phase
            com_x = data[0]['com_x']
            com_y = data[0]['com_y']
            zmp_x = data[0]['zmp_x']
            zmp_y = data[0]['zmp_y']
            support_phase = data[0]['support_phase']

            # Update data real-time
            com_x_data.append(com_x)
            com_y_data.append(com_y)
            zmp_x_data.append(zmp_x)
            zmp_y_data.append(zmp_y)
            support_phase_data.append(support_phase)

            # Tentukan x_min dan x_max berdasarkan support phase
            if support_phase == 'Double Support':
                x_min = -8.6
                x_max = 8.6
                y_min = -6.5
                y_max = 6.5
            else:
                x_min = -4.3
                x_max = 4.3
                y_min = -6.5
                y_max = 6.5

            # Plot data CoM dan ZMP pada sumbu X
            axs[0].cla()  # Clear axis
            axs[0].fill_between(range(len(com_x_data)), x_min, x_max, color='lightblue', label='Support Polygon Area')
            axs[0].plot(com_x_data, color='green', linestyle='--', linewidth=2, label='CoM')
            axs[0].plot(zmp_x_data, color='red', linewidth=2, label='ZMP')
            axs[0].set_xlim([0, len(com_x_data)])
            axs[0].set_xlabel('Time (s)')
            axs[0].set_ylabel('Position (m)')
            axs[0].legend(loc='best')

            # Plot data CoM dan ZMP pada sumbu Y (optional)
            axs[1].cla()  # Clear axis
            axs[1].fill_between(range(len(com_y_data)), y_min, y_max, color='lightblue', label='Support Polygon Area')
            axs[1].plot(com_y_data, color='green', linestyle='--', linewidth=2, label='CoM')
            axs[1].plot(zmp_y_data, color='red', linewidth=2, label='ZMP')
            axs[1].set_xlim([0, len(com_y_data)])
            axs[1].set_xlabel('Time (s)')
            axs[1].set_ylabel('Position (m)')
            axs[1].legend(loc='best')

            # Refresh the plot
            plt.draw()
            plt.pause(0.01)  # Allow for the plot to refresh

    # Loop to update the plot continuously
    while not rospy.is_shutdown():
        update_plot()

    plt.show()  # Show the plot window when ROS node is done

# Fungsi utama untuk inisialisasi ROS dan memulai plotting
def main():
    # Initialize the ROS node once
    rospy.init_node('dynamic_com_calculator', anonymous=True)  # Initialize with a single name
    
    rate = rospy.Rate(10)  # 10 Hz (or whatever frequency you want)

    while not rospy.is_shutdown():
        # Get CoM data and calculate ZMP
        com_data = calculate_dynamic_com()
        if com_data:
            CoM_x, CoM_y, CoM_z = com_data
            calculate_dynamic_zmp(CoM_x, CoM_y, CoM_z)

        # Detect support phase and calculate polygon
        if joint_positions is not None and accel is not None:
            imu_accel_z = accel[1]  # Accelerometer's Z-axis data from IMU
            joint_states = [joint_positions]  # Create joint state list for input
            support_polygon_data = calculate_support_polygon(joint_states, imu_accel_z)

            # Log the calculated support phase and polygon data
            rospy.loginfo(
                "Support Phase: {}, Area: {}, Perimeter: {}".format(
                    support_polygon_data["phase"],
                    support_polygon_data["area"],
                    support_polygon_data["perimeter"]
                )
            )

        # Fetch and log calculated data
        data = calculate_data()
        if data:
            rospy.loginfo("Calculated Data: {}".format(data))

        # Call the function for real-time plotting
        real_time_plot()

        # Sleep to maintain the desired rate (10 Hz)
        rate.sleep()

if __name__ == '__main__':
    main()
