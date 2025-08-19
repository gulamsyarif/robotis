#!/usr/bin/env python3

import data_imu.record_imu_data as record_imu_data
import wavelet_filter
import ukf_estimation
import zmp_com_support
import balance_evaluation

# 1. Get IMU data
acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = record_imu_data.read_imu_data()

# 2. Apply Wavelet transform for noise reduction
denoised_accel_signal = wavelet_filter.wavelet_denoise([acc_x, acc_y, acc_z])

# 3. Use UKF for state estimation
filtered_state = ukf_estimation.ukf_estimation(denoised_accel_signal)

# 4. Calculate CoM, ZMP, and Support Polygon
zmp_x, zmp_y = zmp_com_support.calculate_zmp(acc_x, acc_y, acc_z, height_com=0.5)
support_polygon_area = zmp_com_support.calculate_support_polygon(foot_positions=[[0,0], [0.1,0]])

# 5. Check if the robot is balanced
balanced = balance_evaluation.is_robot_balanced(zmp_x, zmp_y, support_polygon_area)
print("Is the robot balanced? ", balanced)
