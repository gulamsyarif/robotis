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
with open(joint_state_file, 'w') as f:
    writer = csv.writer(f)
    writer.writerow(["Time", "Joint States"])

with open(com_file, 'w') as f:
    writer = csv.writer(f)
    writer.writerow(["Time", "CoM_x", "CoM_y", "CoM_z"])

with open(velocity_file, 'w') as f:
    writer = csv.writer(f)
    writer.writerow(["Time", "Velocity_x", "Velocity_y", "Velocity_z"])

with open(zmp_file, 'w') as f:
    writer = csv.writer(f)
    writer.writerow(["Time", "ZMP_x", "ZMP_y"])

with open(initial_imu_file, 'w') as f:
    writer = csv.writer(f)
    writer.writerow(["Time", "Accel_x", "Accel_y", "Accel_z", "Orientation_x", "Orientation_y", "Orientation_z", "Orientation_w", "Gyro_x", "Gyro_y", "Gyro_z"])

with open(denoised_gyro_file, 'w') as f:
    writer = csv.writer(f)
    writer.writerow(["Time", "Denoised_Gyro_x", "Denoised_Gyro_y", "Denoised_Gyro_z"])

# UKF parameters
alpha = 1e-3
beta = 2
kappa = 0
n = 3  # State dimension: gyro x, y, z
lambda_ = alpha**2 * (n + kappa) - n
gamma = np.sqrt(n + lambda_)

# UKF Functions
def calculate_sigma_points(x, P):
    sigma_points = np.zeros((2 * n + 1, n))
    sigma_points[0] = x
    sqrt_P = np.linalg.cholesky((n + lambda_) * P)
    for i in range(n):
        sigma_points[i + 1] = x + sqrt_P[i]
        sigma_points[i + 1 + n] = x - sqrt_P[i]
    return sigma_points

def predict_sigma_points(sigma_points, dt):
    predicted_sigma_points = np.zeros_like(sigma_points)
    for i, sp in enumerate(sigma_points):
        predicted_sigma_points[i] = sp  # Assuming gyro remains linear
    return predicted_sigma_points

def predict_mean_and_covariance(predicted_sigma_points, process_noise_cov):
    weights_mean = np.full(2 * n + 1, 0.5 / (n + lambda_))
    weights_mean[0] = lambda_ / (n + lambda_)
    weights_cov = np.copy(weights_mean)
    weights_cov[0] += 1 - alpha**2 + beta

    x_pred = np.sum(weights_mean[:, None] * predicted_sigma_points, axis=0)
    P_pred = process_noise_cov
    for i in range(2 * n + 1):
        diff = predicted_sigma_points[i] - x_pred
        P_pred += weights_cov[i] * np.outer(diff, diff)
    return x_pred, P_pred

def update(x_pred, P_pred, z, R):
    sigma_points = calculate_sigma_points(x_pred, P_pred)
    z_sigma_points = sigma_points

    z_pred = np.mean(z_sigma_points, axis=0)
    P_zz = R + np.sum([(sp - z_pred)[:, None] * (sp - z_pred)[None, :] for sp in z_sigma_points], axis=0)
    P_xz = np.sum([(sigma_points[i] - x_pred)[:, None] * (z_sigma_points[i] - z_pred)[None, :] for i in range(2 * n + 1)], axis=0)
    
    K = np.dot(P_xz, np.linalg.inv(P_zz))
    x_updated = x_pred + np.dot(K, (z - z_pred))
    P_updated = P_pred - np.dot(K, P_zz).dot(K.T)
    
    return x_updated, P_updated

def ukf_denoise(gyro_data, process_noise, measurement_noise, dt):
    x = np.zeros(3)  # Initial state
    P = np.eye(3)  # Initial covariance

    denoised_signal = []
    for z in gyro_data:
        sigma_points = calculate_sigma_points(x, P)
        predicted_sigma_points = predict_sigma_points(sigma_points, dt)
        x_pred, P_pred = predict_mean_and_covariance(predicted_sigma_points, process_noise)
        
        x, P = update(x_pred, P_pred, z, measurement_noise)
        denoised_signal.append(x)
    
    return np.array(denoised_signal)

buffer_size = 10
gyro_x_buffer, gyro_y_buffer, gyro_z_buffer = [], [], []
joint_state_buffer = []

def calculate_com():
    """Calculate the center of mass (CoM) based on the servo data."""
    total_mass = sum(servo['mass'] for servo in servo_data)
    com_x = sum(servo['mass'] * servo['CoM_offset'][0] for servo in servo_data) / total_mass
    com_y = sum(servo['mass'] * servo['CoM_offset'][1] for servo in servo_data) / total_mass
    com_z = sum(servo['mass'] * servo['CoM_offset'][2] for servo in servo_data) / total_mass
    return com_x, com_y, com_z

def calculate_velocity(dt):
    global previous_com, previous_time
    com = calculate_com()
    if previous_com is None or previous_time is None:
        previous_com = com
        previous_time = rospy.Time.now().to_sec()
        return 0, 0, 0  # No velocity available at the start
    current_time = rospy.Time.now().to_sec()
    velocity_x = (com[0] - previous_com[0]) / (current_time - previous_time)
    velocity_y = (com[1] - previous_com[1]) / (current_time - previous_time)
    velocity_z = (com[2] - previous_com[2]) / (current_time - previous_time)
    previous_com = com
    previous_time = current_time
    return velocity_x, velocity_y, velocity_z

def calculate_zmp():
    com_x, com_y, com_z = calculate_com()
    # Assume ZMP is a projection of CoM for simplicity
    zmp_x = com_x
    zmp_y = com_y
    return zmp_x, zmp_y

def imu_callback(msg):
    global gyro_x_buffer, gyro_y_buffer, gyro_z_buffer, joint_state_buffer, data_count

    # Extract IMU data
    accel = msg.linear_acceleration
    orientation = msg.orientation
    gyro_raw = msg.angular_velocity

    # Log initial IMU data
    time_now = rospy.Time.now().to_sec()
    initial_imu_data = [time_now, accel.x, accel.y, accel.z, orientation.x, orientation.y, orientation.z, orientation.w, gyro_raw.x, gyro_raw.y, gyro_raw.z]
    
    with open(initial_imu_file, 'a') as f:
        writer = csv.writer(f)
        writer.writerow(initial_imu_data)

    # Buffer the gyro data
    gyro_x_buffer.append(gyro_raw.x)
    gyro_y_buffer.append(gyro_raw.y)
    gyro_z_buffer.append(gyro_raw.z)

    # Log joint states (assuming data is available in the correct format)
    joint_state_data = [time_now, gyro_raw.x, gyro_raw.y, gyro_raw.z]  # Replace with actual joint states
    global joint_state_buffer  # Declare joint_state_buffer as global
    joint_state_buffer.append(joint_state_data)
    
    # Check if enough data has been collected
    if len(gyro_x_buffer) >= buffer_size:
        gyro_data = np.array([gyro_x_buffer, gyro_y_buffer, gyro_z_buffer]).T
        process_noise = np.eye(3) * 0.01
        measurement_noise = np.eye(3) * 0.1
        dt = 0.01

        denoised_gyro = ukf_denoise(gyro_data, process_noise, measurement_noise, dt)
        
        if len(denoised_gyro) > 0:
            time_now = rospy.Time.now().to_sec()
            denoised_gyro_data = [time_now, denoised_gyro[-1, 0], denoised_gyro[-1, 1], denoised_gyro[-1, 2]]
            rospy.loginfo("Denoised Gyro: [%f, %f, %f]" % (denoised_gyro[-1, 0], denoised_gyro[-1, 1], denoised_gyro[-1, 2]))

            # Save denoised gyro data to CSV
            with open(denoised_gyro_file, 'a') as f:
                writer = csv.writer(f)
                writer.writerow(denoised_gyro_data)

            # Calculate and save CoM, velocity, and ZMP
            com = calculate_com()
            velocity = calculate_velocity(dt)
            zmp = calculate_zmp()

            # Log joint states to CSV
            with open(joint_state_file, 'a') as f:
                writer = csv.writer(f)
                writer.writerows(joint_state_buffer)

            # Log CoM data to CSV
            with open(com_file, 'a') as f:
                writer = csv.writer(f)
                com_data = [time_now, com[0], com[1], com[2]]
                writer.writerow(com_data)

            # Log velocity data to CSV
            with open(velocity_file, 'a') as f:
                writer = csv.writer(f)
                velocity_data = [time_now, velocity[0], velocity[1], velocity[2]]
                writer.writerow(velocity_data)

            # Log ZMP data to CSV
            with open(zmp_file, 'a') as f:
                writer = csv.writer(f)
                zmp_data = [time_now, zmp[0], zmp[1]]
                writer.writerow(zmp_data)

        # Clear buffers
        gyro_x_buffer, gyro_y_buffer, gyro_z_buffer = [], [], []
        joint_state_buffer = []

def main():
    rospy.init_node('imu_ukf_denoise_node', anonymous=True)
    rospy.Subscriber("robotis/open_cr/imu", Imu, imu_callback)
    # Assuming you have a way to subscribe to joint states as well
    # rospy.Subscriber("your_joint_states_topic", JointState, joint_state_callback) 
    rospy.spin()

if __name__ == "__main__":
    main()
