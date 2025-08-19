#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu, JointState
import numpy as np
import csv

# Import your denoising function
from IMU_wavelet_filter import get_denoised_gyro

# Global variables for storing sensor data
accel = None
orientation = None
gyro_raw = None
joint_positions = None
previous_com = None  # For storing the previous CoM position
previous_time = None  # For calculating delta time

# CSV file for logging data
log_file_path = "robot_data_log.csv"

# Initialize CSV file
with open(log_file_path, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Time", "Accel_X", "Accel_Y", "Accel_Z",
                     "Orientation_X", "Orientation_Y", "Orientation_Z", "Orientation_W",
                     "Gyro_X", "Gyro_Y", "Gyro_Z",
                     "CoM_X", "CoM_Y", "CoM_Z",
                     "Velocity_X", "Velocity_Y", "Velocity_Z",
                     "ZMP_X", "ZMP_Y"])

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

# Initialize the ROS node and subscribe to topics
rospy.init_node('dynamic_com_calculator', anonymous=True)
rospy.Subscriber("/robotis/open_cr/imu", Imu, imu_callback)
rospy.Subscriber("/robotis/present_joint_states", JointState, joint_state_callback)

# Servo data for mass and CoM offset calculations
servo_data = [
    {"id": 20, "mass": 0.09, "CoM_offset": (0, 2.85, 45.1)},
    # (Add the rest of the servo data similarly)
    # ...
]

# Function to convert quaternion to rotation matrix
def quaternion_to_rotation_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)]
    ])

# Function to calculate dynamic CoM
def calculate_dynamic_com():
    global previous_com, previous_time
    if accel is None or orientation is None or joint_positions is None or gyro_raw is None:
        rospy.loginfo("Waiting for sensor data...")
        return None

    total_mass = 0
    weighted_sum = np.array([0.0, 0.0, 0.0])

    # Get filtered gyro data
    gyro = get_filtered_gyro(gyro_raw)

    # Calculate the rotation matrix from the orientation quaternion
    rotation_matrix = quaternion_to_rotation_matrix(orientation)

    # Calculate the dynamic CoM
    for servo in servo_data:
        mass = servo["mass"]
        offset = np.array(servo["CoM_offset"])

        # Adjust CoM position based on joint positions
        joint_id = servo["id"] - 1
        joint_pos = joint_positions[joint_id] if joint_id < len(joint_positions) else 0

        # Calculate servo position based on offset and rotation
        local_position = offset + np.array([0, 0, joint_pos])
        transformed_position = rotation_matrix.dot(local_position)

        weighted_sum += transformed_position * mass
        total_mass += mass

    # Calculate CoM position
    CoM_x, CoM_y, CoM_z = weighted_sum / total_mass
    rospy.loginfo("Calculated Dynamic CoM: x={}, y={}, z={}".format(CoM_x, CoM_y, CoM_z))

    # Calculate CoM velocity
    current_time = rospy.get_time()
    if previous_com is not None and previous_time is not None:
        delta_time = current_time - previous_time
        velocity_com = (np.array([CoM_x, CoM_y, CoM_z]) - previous_com) / delta_time
    else:
        velocity_com = np.array([0.0, 0.0, 0.0])

    previous_com = np.array([CoM_x, CoM_y, CoM_z])
    previous_time = current_time

    # Project CoM onto the ground (z=0)
    projected_com = np.array([CoM_x, CoM_y, 0])
    rospy.loginfo("Projected CoM on the ground: x={}, y={}, z=0".format(projected_com[0], projected_com[1]))

    return CoM_x, CoM_y, CoM_z, velocity_com

# Function to calculate dynamic ZMP
def calculate_dynamic_zmp(CoM_x, CoM_y, CoM_z):
    global accel

    if accel is None:
        rospy.loginfo("Waiting for accelerometer data...")
        return None

    GRAVITY = 9.81
    F_z = accel[2] - GRAVITY

    total_mass = sum(servo["mass"] for servo in servo_data)
    F_normal = total_mass * GRAVITY

    # Friction coefficient (example: 0.5 for synthetic surface)
    mu = 0.5
    F_friction = mu * F_normal

    if abs(F_z) < 1e-6:
        F_z = 1e-6

    ZMP_x = CoM_x - (F_friction / F_z) * CoM_y
    ZMP_y = CoM_y + (F_friction / F_z) * CoM_x

    return ZMP_x, ZMP_y

# Main loop for calculating and logging data
def main_loop():
    count = 0
    while not rospy.is_shutdown():
        com_values = calculate_dynamic_com()
        if com_values is not None:
            CoM_x, CoM_y, CoM_z, velocity_com = com_values
            zmp_values = calculate_dynamic_zmp(CoM_x, CoM_y, CoM_z)
            if zmp_values is not None:
                ZMP_x, ZMP_y = zmp_values

                # Log data into CSV file
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

        rospy.sleep(0.1)

if __name__ == "__main__":
    main_loop()
