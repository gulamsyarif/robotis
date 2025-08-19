import rospy
from sensor_msgs.msg import Imu, JointState
from IMU_wavelet_filter import get_denoised_gyro
import numpy as np
import csv
import os

# Global variables to store sensor data
accel = None
orientation = None
gyro_raw = None
joint_positions = None
previous_com = None  # To store previous CoM position
previous_time = None  # To calculate delta time
data_count = 0  # To count number of data points

# File paths for CSV
data_dir = "sensor_data"
if not os.path.exists(data_dir):
    os.makedirs(data_dir)

joint_state_file = os.path.join(data_dir, "joint_states.csv")
com_file = os.path.join(data_dir, "com.csv")
velocity_file = os.path.join(data_dir, "velocity.csv")
zmp_file = os.path.join(data_dir, "zmp.csv")

# Initialize CSV files
with open(joint_state_file, 'wb') as f:
    writer = csv.writer(f)
    writer.writerow(["Time", "Joint States"])

with open(com_file, 'wb') as f:
    writer = csv.writer(f)
    writer.writerow(["Time", "CoM_x", "CoM_y", "CoM_z"])

with open(velocity_file, 'wb') as f:
    writer = csv.writer(f)
    writer.writerow(["Time", "Velocity_x", "Velocity_y", "Velocity_z"])

with open(zmp_file, 'wb') as f:
    writer = csv.writer(f)
    writer.writerow(["Time", "ZMP_x", "ZMP_y"])

# Callback function for IMU data
def imu_callback(msg):
    global accel, orientation, gyro_raw
    accel = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
    orientation = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
    gyro_raw = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)

# Callback function for joint state data
def joint_state_callback(msg):
    global joint_positions
    joint_positions = msg.position

# Initialize ROS Node and subscribe to topics
rospy.init_node('dynamic_com_calculator', anonymous=True)
rospy.Subscriber("/robotis/open_cr/imu", Imu, imu_callback)
rospy.Subscriber("/robotis/present_joint_states", JointState, joint_state_callback)

# Function to get filtered gyroscope data
def get_filtered_gyro(gyro_raw):
    # Get filtered gyroscope data by providing gyro_raw as input
    denoised_gyro = get_denoised_gyro(gyro_raw)
    return denoised_gyro

# Data for servos
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
    global previous_com, previous_time, data_count
    if accel is None or orientation is None or joint_positions is None or gyro_raw is None:
        rospy.loginfo("Waiting for sensor data...")
        return None

    total_mass = 0
    weighted_sum = np.array([0.0, 0.0, 0.0])

    # Get filtered gyro data
    gyro = get_filtered_gyro(gyro_raw)

    # Calculate rotation matrix from quaternion
    rotation_matrix = quaternion_to_rotation_matrix(orientation)

    # Calculate dynamic CoM
    for servo in servo_data:
        mass = servo["mass"]
        offset = np.array(servo["CoM_offset"])

        # Adjust CoM position based on joint positions
        joint_id = servo["id"] - 1  # Python index starts at 0
        joint_pos = joint_positions[joint_id] if joint_id < len(joint_positions) else 0

        # Calculate servo position based on offset and rotation
        local_position = offset + np.array([0, 0, joint_pos])  # Consider joint_pos on Z-axis
        transformed_position = rotation_matrix.dot(local_position)

        weighted_sum += transformed_position * mass
        total_mass += mass

    # Calculate CoM
    CoM_x, CoM_y, CoM_z = weighted_sum / total_mass
    rospy.loginfo("Calculated Dynamic CoM: x={}, y={}, z={}".format(CoM_x, CoM_y, CoM_z))

    # Calculate CoM velocity
    current_time = rospy.get_time()
    if previous_com is not None and previous_time is not None:
        delta_time = current_time - previous_time
        velocity_com = (np.array([CoM_x, CoM_y, CoM_z]) - previous_com) / delta_time
    else:
        velocity_com = np.array([0.0, 0.0, 0.0])  # If this is the first time, velocity considered zero

    previous_com = np.array([CoM_x, CoM_y, CoM_z])
    previous_time = current_time

    # Project CoM on the ground
    projected_com = np.array([CoM_x, CoM_y, 0])  # Assume ground is z=0
    rospy.loginfo("Projected CoM on the ground: x={}, y={}, z=0".format(projected_com[0], projected_com[1]))

    # Log the data
    log_data(current_time, CoM_x, CoM_y, CoM_z, velocity_com, projected_com)

    return CoM_x, CoM_y, CoM_z, velocity_com, projected_com

def log_data(current_time, CoM_x, CoM_y, CoM_z, velocity_com, projected_com):
    global data_count
    if data_count < 3000:
        # Log joint states
        with open(joint_state_file, 'ab') as f:
            writer = csv.writer(f)
            writer.writerow([current_time] + list(joint_positions))

        # Log CoM
        with open(com_file, 'ab') as f:
            writer = csv.writer(f)
            writer.writerow([current_time, CoM_x, CoM_y, CoM_z])

        # Log velocity
        with open(velocity_file, 'ab') as f:
            writer = csv.writer(f)
            writer.writerow([current_time] + list(velocity_com))

        # Log projected CoM
        with open(zmp_file, 'ab') as f:
            writer = csv.writer(f)
            writer.writerow([current_time] + list(projected_com))

        data_count += 1
    else:
        rospy.loginfo("Data collection limit reached.")

# Main loop
rate = rospy.Rate(10)  # 10 Hz
while not rospy.is_shutdown():
    calculate_dynamic_com()
    rate.sleep()
