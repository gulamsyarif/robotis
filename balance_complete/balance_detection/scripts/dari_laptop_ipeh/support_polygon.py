# #!/usr/bin/env python
# -*- coding: utf-8 -*-

# import rospy
# from sensor_msgs.msg import Imu, JointState
# from geometry_msgs.msg import Point
# import math

# class SupportPolygonCalculator:
#     def __init__(self):
#         # Subscribe to the IMU and JointState topics
#         rospy.Subscriber("/robotis/open_cr/imu", Imu, self.imu_callback)
#         rospy.Subscriber("/robotis/present_joint_states", JointState, self.joint_state_callback)

#         self.orientation = None  # Store IMU orientation data (quaternion)
#         self.joint_states = None  # Store joint states
        
#         self.support_polygon_points = []
#         self.foot_length = 0.13  # 13 cm
#         self.foot_width = 0.08   # 8 cm
        
#         rospy.spin()

#     def imu_callback(self, msg):
#         # Extract quaternion orientation from IMU
#         self.orientation = msg.orientation
#         self.roll, self.pitch, self.yaw = self.quaternion_to_euler(self.orientation)

#     def joint_state_callback(self, msg):
#         self.joint_states = msg
#         self.calculate_foot_positions()

#     def calculate_foot_positions(self):
#         if self.joint_states is None or self.orientation is None:
#             return
        
#         left_foot_joint_names = ['l_hip_yaw', 'l_hip_roll', 'l_hip_pitch', 'l_knee', 'l_ank_pitch', 'l_ank_roll']
#         right_foot_joint_names = ['r_hip_yaw', 'r_hip_roll', 'r_hip_pitch', 'r_knee', 'r_ank_pitch', 'r_ank_roll']
        
#         left_foot_position = self.forward_kinematics(left_foot_joint_names)
#         right_foot_position = self.forward_kinematics(right_foot_joint_names)
        
#         # Normalize z-coordinates to ground level
#         left_foot_position.z = 0
#         right_foot_position.z = 0

#         # Get foot corners (using length and width)
#         left_foot_corners = self.get_foot_corners(left_foot_position)
#         right_foot_corners = self.get_foot_corners(right_foot_position)

#         # Log the corners for debugging
#         rospy.loginfo("Left Foot Corners: {}".format(left_foot_corners))
#         rospy.loginfo("Right Foot Corners: {}".format(right_foot_corners))

#         # Combine corners for support polygon
#         self.support_polygon_points = left_foot_corners + right_foot_corners

#         # Calculate area with updated points
#         area = self.calculate_polygon_area(self.support_polygon_points)
#         rospy.loginfo("Support Polygon Area: {:.4f} m^2".format(area))

#         # Log support polygon dimensions
#         self.log_support_polygon_dimensions(left_foot_corners, right_foot_corners)

#     def forward_kinematics(self, joint_names):
#         joint_positions = []
#         for joint_name in joint_names:
#             idx = self.joint_states.name.index(joint_name)
#             joint_positions.append(self.joint_states.position[idx])
        
#         # Link lengths (adjust based on your robot model)
#         l_hip = 9.0  # Length from hip to knee
#         l_knee = 20.8  # Length from knee to ankle

#         # Calculate foot position based on joint angles
#         hip_yaw = joint_positions[0]
#         hip_roll = joint_positions[1]
#         hip_pitch = joint_positions[2]
#         knee_angle = joint_positions[3]
        
#         # Calculate knee position
#         knee_x = l_hip * math.cos(hip_yaw)
#         knee_y = l_hip * math.sin(hip_yaw)
#         knee_z = l_hip * math.sin(hip_roll)

#         # Calculate ankle position
#         ankle_x = knee_x + l_knee * math.cos(hip_yaw) * math.cos(hip_pitch)
#         ankle_y = knee_y + l_knee * math.sin(hip_yaw) * math.cos(hip_pitch)
#         ankle_z = knee_z - l_knee * math.sin(hip_pitch)

#         return Point(ankle_x, ankle_y, ankle_z)

#     def get_foot_corners(self, foot_position):
#         # Calculate the four corners of the foot based on its dimensions
#         half_length = self.foot_length / 2
#         half_width = self.foot_width / 2
        
#         # Define the four corners (front-left, front-right, back-left, back-right)
#         corners = [
#             Point(foot_position.x - half_length, foot_position.y - half_width, 0),  # Back-left
#             Point(foot_position.x - half_length, foot_position.y + half_width, 0),  # Back-right
#             Point(foot_position.x + half_length, foot_position.y - half_width, 0),  # Front-left
#             Point(foot_position.x + half_length, foot_position.y + half_width, 0),  # Front-right
#         ]
        
#         return corners

#     def calculate_polygon_area(self, points):
#         if len(points) < 3:
#             return 0.0  # Not a valid polygon

#         x = [p.x for p in points]
#         y = [p.y for p in points]
#         area = 0.0
#         n = len(points)
#         for i in range(n):
#             j = (i + 1) % n
#             area += x[i] * y[j]
#             area -= y[i] * x[j]
#         return abs(area) / 2.0

#     def log_support_polygon_dimensions(self, left_foot_corners, right_foot_corners):
#         # Calculate distances between key points in the support polygon
#         def distance(p1, p2):
#             return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)
        
#         rospy.loginfo("Distance between Left Foot and Right Foot: {:.4f} m".format(
#             distance(left_foot_corners[0], right_foot_corners[0])))
#         rospy.loginfo("Distance between Left Toe and Right Toe: {:.4f} m".format(
#             distance(left_foot_corners[2], right_foot_corners[2])))

#     def quaternion_to_euler(self, orientation):
#         x = orientation.x
#         y = orientation.y
#         z = orientation.z
#         w = orientation.w

#         sinr_cosp = 2 * (w * x + y * z)
#         cosr_cosp = 1 - 2 * (x * x + y * y)
#         roll = math.atan2(sinr_cosp, cosr_cosp)

#         sinp = 2 * (w * y - z * x)
#         if abs(sinp) >= 1:
#             pitch = math.pi / 2 * math.copysign(1, sinp)
#         else:
#             pitch = math.asin(sinp)

#         siny_cosp = 2 * (w * z + x * y)
#         cosy_cosp = 1 - 2 * (y * y + z * z)
#         yaw = math.atan2(siny_cosp, cosy_cosp)

#         return roll, pitch, yaw

# if __name__ == '__main__':
#     rospy.init_node('support_polygon_calculator', anonymous=True)
#     spc = SupportPolygonCalculator()

# ======================================

# import rospy
# import numpy as np
# from sensor_msgs.msg import Imu, JointState
# from tf.transformations import euler_from_quaternion  # Import this to convert quaternion to Euler angles

# # Initialize global variables
# imu_orientation = None
# joint_states = None

# # Foot vertices relative to the center of the foot
# # Define the octagon shape with given side lengths (in cm)
# foot_vertices = np.array([
#     [8, 0], 
#     [8 + 1.8, 1.8],
#     [8 + 1.8 + 13, 0],
#     [8 + 1.8 + 13 + 1.8, -1.8],
#     [8, -13], 
#     [6.2, -13 - 1.8], 
#     [0, -13],
#     [-1.8, -11.2]
# ])  # Adjust these based on the orientation of each foot

# # Callback to get IMU data
# def imu_callback(data):
#     global imu_orientation
#     imu_orientation = data.orientation  # Quaternion (x, y, z, w)

# # Callback to get joint states data
# def joint_state_callback(data):
#     global joint_states
#     joint_states = data

# # Function to apply rotation matrix based on IMU orientation
# def apply_orientation(vertices, orientation):
#     # Convert quaternion to rotation matrix (2D projection)
#     roll, pitch, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
#     rotation_matrix = np.array([
#         [np.cos(yaw), -np.sin(yaw)],
#         [np.sin(yaw), np.cos(yaw)]
#     ])
#     return np.dot(vertices, rotation_matrix.T)

# # Function to calculate support polygon (including length, width, area)
# def calculate_support_polygon():
#     if imu_orientation is None or joint_states is None:
#         return

#     # Get positions of both feet using joint angles (simplified)
#     # Assuming both feet are 2.2 cm apart, and the adjacent sides are the 13 cm ones
#     left_foot_position = np.array([-1.1, 0])  # Initial left foot position
#     right_foot_position = np.array([1.1, 0])  # Initial right foot position

#     # Rotate foot vertices based on IMU orientation
#     left_foot_polygon = apply_orientation(foot_vertices, imu_orientation) + left_foot_position
#     right_foot_polygon = apply_orientation(foot_vertices, imu_orientation) + right_foot_position

#     # Combine the left and right foot polygons
#     support_polygon = np.vstack((left_foot_polygon, right_foot_polygon))

#     # Calculate support polygon area
#     area = calculate_polygon_area(support_polygon)

#     # Calculate bounding box (length and width)
#     length = np.max(support_polygon[:, 0]) - np.min(support_polygon[:, 0])  # Max x - Min x
#     width = np.max(support_polygon[:, 1]) - np.min(support_polygon[:, 1])    # Max y - Min y

#     # Print or log the results
#     print("Support Polygon Length: {} cm".format(length))
#     print("Support Polygon Width: {} cm".format(width))
#     print("Support Polygon Area: {} cm^2".format(area))

# # Function to calculate the area of a polygon
# def calculate_polygon_area(polygon):
#     x = polygon[:, 0]
#     y = polygon[:, 1]
#     return 0.5 * np.abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))

# # ROS initialization and subscriptions
# def listener():
#     rospy.init_node('support_polygon_calculator', anonymous=True)
#     rospy.Subscriber('/robotis/open_cr/imu', Imu, imu_callback)
#     rospy.Subscriber('/robotis/present_joint_states', JointState, joint_state_callback)
    
#     rate = rospy.Rate(10)  # 10 Hz
#     while not rospy.is_shutdown():
#         calculate_support_polygon()
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         listener()
#     except rospy.ROSInterruptException:
#         pass

# ======================================

import numpy as np 
import pandas as pd
import ast
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

# Fungsi menghitung luas menggunakan Shoelace Theorem
def calculate_area(coords):
    x = coords[:, 0]
    y = coords[:, 1]
    return 0.5 * np.abs(np.dot(x[:-1], y[1:]) - np.dot(y[:-1], x[1:]))

# Fungsi menghitung keliling
def calculate_perimeter(coords):
    return np.sum(np.sqrt(np.sum(np.diff(coords, axis=0)**2, axis=1)))

# Membaca data joint states dan IMU
joint_data = pd.read_csv(r'C:\Users\syari\Downloads\balance_detection\balance_detection\scripts\DONE\data_lagi\joint_states.csv')
imu_data = pd.read_csv(r'C:\Users\syari\Downloads\balance_detection\balance_detection\scripts\DONE\data_lagi\initial_imu_data.csv')

# Ekstrak data yang relevan dari IMU
imu_time = imu_data['Time'].tolist()
imu_accel_y = imu_data['Accel_y'].tolist()  # Mengambil akselerasi pada sumbu Y

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

# Koordinat untuk single support kaki kanan dan kiri
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

# Variabel untuk menyimpan fase dukungan terakhir
last_phase = None

# Buat fungsi untuk menentukan fase dukungan
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
            # Tambahkan logika untuk mendeteksi dengan menggunakan akselerasi
            if accel < -0.5:  # Threshold untuk mendeteksi ketika kaki mengangkat
                if l_ank <= 0:  # Jika l_ank_pitch negatif atau nol, berarti kaki kiri tidak di tanah
                    current_phase = 'Right Single Support'
                else:
                    current_phase = 'Left Single Support'
            else:
                current_phase = 'Double Support'

        # Mengatur fase dukungan berdasarkan fase terakhir
        if current_phase.startswith('Single Support'):
            if last_phase == 'Right Single Support':
                current_phase = 'Left Single Support'
            elif last_phase == 'Left Single Support':
                current_phase = 'Right Single Support'

        phases.append(current_phase)
        last_phase = current_phase  # Update last_phase untuk iterasi berikutnya

    return phases

# Inisialisasi last_phase untuk deteksi awal
last_phase = None  # Atau bisa 'None' sesuai kebutuhan

# Menentukan jumlah baris minimal antara joint_data dan imu_data
num_rows = min(len(joint_data), len(imu_data))

# List untuk menyimpan hasil perhitungan
results = []

# Loop melalui setiap baris data (hanya sampai jumlah baris minimum)
for i in range(num_rows):
    joint_state_str = joint_data.iloc[i]['Joint States']
    imu_state = imu_data.iloc[i]

    # Parsing joint state dari string ke tuple (tuple dalam string format)
    joint_state = np.array(ast.literal_eval(joint_state_str))

    # Deteksi fase dukungan
    joint_states = [joint_state]  # Membuat list dengan satu joint state untuk mendeteksi fase
    support_phases = detect_support_phase(joint_states, [imu_accel_y[i]])  # Dapatkan fase dukungan untuk data ini
    current_phase = support_phases[0]  # Ambil fase dukungan saat ini

    # Koordinat untuk support
    points_left_single_support = np.array([
        [0, 0],
        [0, -12.3],
        [1.1, -13],
        [7.1, -13],
        [8.2, -12.3],
        [7.7, 0]
    ])

    points_right_single_support = np.array([
        [9.5, 0],
        [9.5, -12.3],
        [10.6, -13],
        [16.6, -13],
        [17.7, -12.3],
        [17.7, 0]
    ])

    # Update titik berdasarkan fase
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

    # Update koordinat berdasarkan IMU data
    points[:, 0] += joint_state[:len(points)] * 0.1  # Sesuaikan perubahan x berdasarkan posisi joint
    points[:, 1] += imu_state['Orientation_y'] * 0.1  # Sesuaikan perubahan y berdasarkan orientasi (pitch dari IMU)

    # Menambahkan titik pertama di akhir untuk perhitungan shoelace dan keliling
    points = np.vstack([points, points[0]])

    # Menghitung luas dan keliling
    area = calculate_area(points)
    perimeter = calculate_perimeter(points)

    # Simpan hasil dalam list
    results.append({
        'Time': joint_data.iloc[i]['Time'],
        'Area': area,
        'Perimeter': perimeter,
        'Support Phase': current_phase
    })

# Membuat DataFrame dari hasil perhitungan
results_df = pd.DataFrame(results)

# Simpan hasil ke file CSV baru
results_df.to_csv(r'C:\Users\syari\Downloads\support_polygon_results.csv', index=False)

# Plot hanya untuk baris terakhir
fig, ax = plt.subplots()
polygon = Polygon(points[:-1], closed=True, edgecolor='b', fill=True, alpha=0.3)

# Menambahkan polygon ke plot
ax.add_patch(polygon)
ax.plot(points[:, 0], points[:, 1], 'ro-')  # Plot titik-titik koordinat

# Atur tampilan plot
ax.set_xlim(-2, 20)
ax.set_ylim(-15, 5)
ax.set_aspect('equal', 'box')
ax.set_title(f"Support Polygon Area = {area:.2f} units²\nPerimeter = {perimeter:.2f} units\nPhase = {current_phase}")
ax.set_xlabel("X-axis")
ax.set_ylabel("Y-axis")

# Tampilkan plot
plt.grid(True)
plt.show()

print("Perhitungan selesai. Hasil disimpan di 'support_polygon_results.csv'.")

# ======================================

#!/usr/bin/env python
import rospy
import numpy as np
import ast
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

# Fungsi menghitung luas menggunakan Shoelace Theorem
def calculate_area(coords):
    x = coords[:, 0]
    y = coords[:, 1]
    return 0.5 * np.abs(np.dot(x[:-1], y[1:]) - np.dot(y[:-1], x[1:]))

# Fungsi menghitung keliling
def calculate_perimeter(coords):
    return np.sum(np.sqrt(np.sum(np.diff(coords, axis=0)**2, axis=1)))

# Variabel global untuk menyimpan data dari callback
joint_data = None
imu_data = None

# Pemetaan ID ke index 'name' saat ini
id_to_index_map = {
    'l_ank_pitch': 13,
    'r_ank_pitch': 14,
}

# Callback untuk topik /present_joint_states
def joint_states_callback(msg):
    global joint_data
    joint_data = msg

# Callback untuk topik /open_cr/imu
def imu_callback(msg):
    global imu_data
    imu_data = msg

# Fungsi untuk mendeteksi fase dukungan
def detect_support_phase(joint_states, imu_accel_y):
    l_ank_pitch = joint_states.position[id_to_index_map['l_ank_pitch']]
    r_ank_pitch = joint_states.position[id_to_index_map['r_ank_pitch']]

    if l_ank_pitch > 0 and r_ank_pitch > 0:  # Kedua sendi positif, double support
        current_phase = 'Double Support'
    else:  # Salah satu sendi negatif, single support
        current_phase = 'Single Support'

    return current_phase

def main():
    rospy.init_node('support_phase_detection_node', anonymous=True)

    # Berlangganan topik joint_states dan imu
    rospy.Subscriber('/robotis/present_joint_states', JointState, joint_states_callback)
    rospy.Subscriber('/robotis/open_cr/imu', Imu, imu_callback)

    rate = rospy.Rate(10)  # Loop pada 10Hz

    while not rospy.is_shutdown():
        if joint_data and imu_data:
            # Akses akselerasi dari IMU
            imu_accel_y = imu_data.linear_acceleration.y
            # Deteksi fase dukungan
            current_phase = detect_support_phase(joint_data, imu_accel_y)

            # Koordinat dasar untuk polygon (support area)
            points = np.array([
                [0, 0],        
                [0, -12.3],    
                [1.1, -13],    
                [16.6, -13],   
                [17.7, -12.3], 
                [17.7, 0]      
            ])

            # Update koordinat berdasarkan joint states dan imu data
            points[:, 0] += np.array(joint_data.position[:len(points)]) * 0.1
            points[:, 1] += imu_data.orientation.y * 0.1

            # Tambahkan titik pertama di akhir untuk perhitungan shoelace dan keliling
            points = np.vstack([points, points[0]])

            # Menghitung luas dan keliling
            area = calculate_area(points)
            perimeter = calculate_perimeter(points)

            # Tampilkan hasil
            print("Phase: {}, Area: {:.2f}, Perimeter: {:.2f}".format(current_phase, area, perimeter))

            # Plot untuk iterasi terakhir
            fig, ax = plt.subplots()
            polygon = Polygon(points[:-1], closed=True, edgecolor='b', fill=True, alpha=0.3)
            ax.add_patch(polygon)
            ax.plot(points[:, 0], points[:, 1], 'ro-')  # Plot titik-titik koordinat

            # Atur tampilan plot
            ax.set_xlim(-2, 20)
            ax.set_ylim(-15, 5)
            ax.set_aspect('equal', 'box')
            ax.set_title("Support Polygon Area = {:.2f} units^2\nPerimeter = {:.2f} units\nPhase = {}".format(area, perimeter, current_phase))
            ax.set_xlabel("X-axis")
            ax.set_ylabel("Y-axis")
            plt.grid(True)
            plt.show()

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
