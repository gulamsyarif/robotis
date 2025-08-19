#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu, JointState
import numpy as np
import csv
import os

# Global variables to store sensor data
accel = None
orientation = None
gyro_raw = None
joint_positions = None
previous_com = None
previous_time = None
data_count = 0

# Placeholder for servo data (mass and CoM offsets)
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


# File paths for CSV
data_dir = "3sensor_data_berjalan"
if not os.path.exists(data_dir):
    os.makedirs(data_dir)

joint_state_file = os.path.join(data_dir, "joint_states.csv")
com_file = os.path.join(data_dir, "com.csv")
velocity_file = os.path.join(data_dir, "velocity.csv")
zmp_file = os.path.join(data_dir, "zmp.csv")
initial_imu_file = os.path.join(data_dir, "initial_imu_data.csv")
denoised_gyro_file = os.path.join(data_dir, "denoised_gyro_data.csv")

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

with open(initial_imu_file, 'wb') as f:
    writer = csv.writer(f)
    writer.writerow(["Time", "Accel_x", "Accel_y", "Accel_z", "Orientation_x", "Orientation_y", "Orientation_z", "Orientation_w", "Gyro_x", "Gyro_y", "Gyro_z"])

with open(denoised_gyro_file, 'wb') as f:
    writer = csv.writer(f)
    writer.writerow(["Time", "Denoised_Gyro_x", "Denoised_Gyro_y", "Denoised_Gyro_z"])

# Denoising parameters
buffer_size = 10
gyro_x_buffer, gyro_y_buffer, gyro_z_buffer = [], [], []
initial_imu_data = []  # List to store initial IMU data
denoised_data = []  # List to store denoised gyro data

# Haar Wavelet Transform Functions
def haar_wavelet_transform(data, level=2):
    output = np.copy(data)
    results = []
    length = len(data)
    for _ in range(level):
        detail_coeffs = np.zeros(length // 2)
        approx_coeffs = np.zeros(length // 2)
        for i in range(length // 2):
            approx_coeffs[i] = (output[2 * i] + output[2 * i + 1]) / 2
            detail_coeffs[i] = (output[2 * i] - output[2 * i + 1]) / 2
        results.append((approx_coeffs, detail_coeffs))
        output = approx_coeffs
        length //= 2
    return results

def haar_wavelet_reconstruction(coeffs):
    length = len(coeffs[-1][0]) * 2
    for i in range(len(coeffs)-1, -1, -1):
        approx_coeffs, detail_coeffs = coeffs[i]
        new_length = len(approx_coeffs) * 2
        reconstructed = np.zeros(new_length)
        for j in range(len(approx_coeffs)):
            reconstructed[2 * j] = approx_coeffs[j] + detail_coeffs[j]
            reconstructed[2 * j + 1] = approx_coeffs[j] - detail_coeffs[j]
        coeffs[i] = (reconstructed, detail_coeffs)
    return reconstructed

def soft_thresholding(coeffs, threshold):
    thresholded_coeffs = []
    for (approx, detail) in coeffs:
        thresholded_detail = np.sign(detail) * np.maximum(np.abs(detail) - threshold, 0)
        thresholded_coeffs.append((approx, thresholded_detail))
    return thresholded_coeffs

def dwt_denoise_custom(signal, level=2, threshold=0.5):
    coeffs = haar_wavelet_transform(signal, level=level)
    thresholded_coeffs = soft_thresholding(coeffs, threshold)
    denoised_signal = haar_wavelet_reconstruction(thresholded_coeffs)
    return denoised_signal

# Callback functions
def imu_callback(data):
    global accel, orientation, gyro_raw
    time_now = rospy.Time.now().to_sec()
    
    # Store IMU data
    accel = data.linear_acceleration
    orientation = data.orientation
    gyro_raw = data.angular_velocity

    # Log initial IMU data
    initial_imu_data.append([time_now, accel.x, accel.y, accel.z, 
                             orientation.x, orientation.y, orientation.z, orientation.w, 
                             gyro_raw.x, gyro_raw.y, gyro_raw.z])
    
    # Save to CSV every 100 data points
    if len(initial_imu_data) >= 100:
        with open(initial_imu_file, 'ab') as f:
            writer = csv.writer(f)
            writer.writerows(initial_imu_data)
        initial_imu_data[:] = []  # Clear data after saving
    
    # Process gyro data for denoising
    gyro_x_buffer.append(gyro_raw.x)
    gyro_y_buffer.append(gyro_raw.y)
    gyro_z_buffer.append(gyro_raw.z)
    
    # Denoise gyro data after sufficient samples
    if len(gyro_x_buffer) >= buffer_size:
        denoised_gyro_x = dwt_denoise_custom(np.array(gyro_x_buffer))[-1]
        denoised_gyro_y = dwt_denoise_custom(np.array(gyro_y_buffer))[-1]
        denoised_gyro_z = dwt_denoise_custom(np.array(gyro_z_buffer))[-1]

        denoised_data.append([time_now, denoised_gyro_x, denoised_gyro_y, denoised_gyro_z])

        # Save denoised data to CSV every 100 data points
        if len(denoised_data) >= 100:
            with open(denoised_gyro_file, 'ab') as f:
                writer = csv.writer(f)
                writer.writerows(denoised_data)
            denoised_data[:] = []  # Clear data after saving

        # Clear buffers
        gyro_x_buffer.pop(0)
        gyro_y_buffer.pop(0)
        gyro_z_buffer.pop(0)

def joint_state_callback(data):
    global joint_positions
    time_now = rospy.Time.now().to_sec()
    
    # Store joint states
    joint_positions = data.position
    
    # Log joint states
    with open(joint_state_file, 'ab') as f:
        writer = csv.writer(f)
        writer.writerow([time_now, joint_positions])

    # Calculate CoM, velocity, and ZMP
    calculate_com_and_zmp()

def calculate_com_and_zmp():
    global joint_positions, previous_com, previous_time, data_count

    # Calculate the CoM (Center of Mass)
    if joint_positions is not None:
        total_mass = sum(servo["mass"] for servo in servo_data)
        com_x, com_y, com_z = 0.0, 0.0, 0.0

        for i, servo in enumerate(servo_data):
            angle = joint_positions[i]  # Assuming joint_positions contains the joint angles
            offset = np.array(servo["CoM_offset"])
            # Use some function to calculate the position based on the angle and offset
            position = np.array([offset[0], offset[1], offset[2]])  # Replace this with actual position calculation
            com_x += position[0] * servo["mass"]
            com_y += position[1] * servo["mass"]
            com_z += position[2] * servo["mass"]

        com_x /= total_mass
        com_y /= total_mass
        com_z /= total_mass

        current_time = rospy.get_time()
        dt = current_time - (previous_time if previous_time is not None else current_time)
        previous_time = current_time

        # Calculate CoM velocity
        if previous_com is not None:
            velocity_x = (com_x - previous_com[0]) / dt
            velocity_y = (com_y - previous_com[1]) / dt
            velocity_z = (com_z - previous_com[2]) / dt
        else:
            velocity_x = velocity_y = velocity_z = 0.0

        previous_com = [com_x, com_y, com_z]

        # Calculate ZMP (Zero Moment Point)
        zmp_x = com_x  # Placeholder for actual ZMP calculation
        zmp_y = com_y  # Placeholder for actual ZMP calculation

        # Save CoM and ZMP data
        with open(com_file, 'ab') as f:
            writer = csv.writer(f)
            writer.writerow([current_time, com_x, com_y, com_z])

        with open(velocity_file, 'ab') as f:
            writer = csv.writer(f)
            writer.writerow([current_time, velocity_x, velocity_y, velocity_z])

        with open(zmp_file, 'ab') as f:
            writer = csv.writer(f)
            writer.writerow([current_time, zmp_x, zmp_y])

        data_count += 1  # Increment data count

def main():
    rospy.init_node('sensor_data_logger', anonymous=True)

    rospy.Subscriber("/robotis/open_cr/imu", Imu, imu_callback)
    rospy.Subscriber("/robotis/present_joint_states", JointState, joint_state_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
