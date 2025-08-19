# import pandas as pd
# import matplotlib.pyplot as plt

# # Load data from CSV files
# raw_imu = pd.read_csv(r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\modified\Filter\jongkok\raw_imu.csv').iloc[1000:1101]
# ukf_filtered = pd.read_csv(r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\modified\Filter\jongkok\UKF_filtered.csv').iloc[1000:1101]
# wt_filtered = pd.read_csv(r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\modified\Filter\jongkok\WT_filtered.csv').iloc[1000:1101]

# # Set up the figure and subplots for each axis
# fig, axs = plt.subplots(3, 1, figsize=(12, 8))
# time = raw_imu['Time'] - raw_imu['Time'].iloc[0]

# # Main title for all subplots
# fig.suptitle("Comparison of IMU Data Across Filtering Methods while Robot in Static Squat Position", fontsize=16)

# # Plotting Gyro X data comparison
# axs[0].plot(time, wt_filtered['Denoised_Gyro_x'], color='blue', label='Discrete Wavelet Transformed')
# axs[0].plot(time, ukf_filtered['Denoised_Gyro_x'], color='orange', label='Unscented Kalman Filtered')
# axs[0].plot(time, raw_imu['Gyro_x'], color='green', label='Raw IMU Data')
# axs[0].set_title("Comparison of Gyro X")
# axs[0].set_ylabel("Gyro X")
# axs[0].legend()

# # Plotting Gyro Y data comparison
# axs[1].plot(time, wt_filtered['Denoised_Gyro_y'], color='blue', label='Discrete Wavelet Transformed')
# axs[1].plot(time, ukf_filtered['Denoised_Gyro_y'], color='orange', label='Unscented Kalman Filtered')
# axs[1].plot(time, raw_imu['Gyro_y'], color='green', label='Raw IMU Data')
# axs[1].set_title("Comparison of Gyro Y")
# axs[1].set_ylabel("Gyro Y")
# axs[1].legend()

# # Plotting Gyro Z data comparison
# axs[2].plot(time, wt_filtered['Denoised_Gyro_z'], color='blue', label='Discrete Wavelet Transformed')
# axs[2].plot(time, ukf_filtered['Denoised_Gyro_z'], color='orange', label='Unscented Kalman Filtered')
# axs[2].plot(time, raw_imu['Gyro_z'], color='green', label='Raw IMU Data')
# axs[2].set_title("Comparison of Gyro Z")
# axs[2].set_ylabel("Gyro Z")
# axs[2].set_xlabel("Time")
# axs[2].legend()

# # Adjust layout to prevent overlap
# plt.tight_layout(rect=[0, 0.03, 1, 0.95])  # Leave space for suptitle
# plt.show()

import pandas as pd
import matplotlib.pyplot as plt

# Load data from CSV files
raw_imu = pd.read_csv(r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\modified\Filter\nendang\raw_imu.csv')
ukf_filtered = pd.read_csv(r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\modified\Filter\nendang\UKF_filtered.csv')
wt_filtered = pd.read_csv(r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\modified\Filter\nendang\WT_filtered.csv')

# Tentukan waktu awal dari indeks ke-1000
start_time = raw_imu.loc[1000, 'Time']
end_time = start_time + 2 # 5 detik setelah waktu awal

# Dapatkan subset `raw_imu` yang berada dalam rentang waktu
raw_imu_filtered = raw_imu[(raw_imu['Time'] >= start_time) & (raw_imu['Time'] <= end_time)]

# Dapatkan indeks yang sesuai dengan rentang waktu di `raw_imu`
indices = raw_imu_filtered.index

# Gunakan indeks yang sama untuk memfilter `ukf_filtered` dan `wt_filtered`
ukf_filtered_filtered = ukf_filtered.loc[indices]
wt_filtered_filtered = wt_filtered.loc[indices]

# Set x-axis time to start from 0
time = raw_imu_filtered['Time'] - start_time  # Normalisasi waktu agar mulai dari 0

# Plot data yang sudah difilter
fig, axs = plt.subplots(3, 1, figsize=(12, 8))

# Main title for all subplots
fig.suptitle("Comparison of IMU Data Across Filtering Methods while Robot in Kicking Condition", fontsize=16)

# Plotting Gyro X data comparison
axs[0].plot(time, wt_filtered_filtered['Denoised_Gyro_x'], color='blue', label='Discrete Wavelet Transformed')
axs[0].plot(time, ukf_filtered_filtered['Denoised_Gyro_x'], color='orange', label='Unscented Kalman Filtered')
axs[0].plot(time, raw_imu_filtered['Gyro_x'], color='green', label='Raw IMU Data')
axs[0].set_title("Comparison of Gyro X")
axs[0].set_ylabel("Gyro X (°/s)")
axs[0].set_xlabel("Time (s)")
axs[0].legend()

# Plotting Gyro Y data comparison
axs[1].plot(time, wt_filtered_filtered['Denoised_Gyro_y'], color='blue', label='Discrete Wavelet Transformed')
axs[1].plot(time, ukf_filtered_filtered['Denoised_Gyro_y'], color='orange', label='Unscented Kalman Filtered')
axs[1].plot(time, raw_imu_filtered['Gyro_y'], color='green', label='Raw IMU Data')
axs[1].set_title("Comparison of Gyro Y")
axs[1].set_ylabel("Gyro Y (°/s)")
axs[1].set_xlabel("Time (s)")
axs[1].legend()

# Plotting Gyro Z data comparison
axs[2].plot(time, wt_filtered_filtered['Denoised_Gyro_z'], color='blue', label='Discrete Wavelet Transformed')
axs[2].plot(time, ukf_filtered_filtered['Denoised_Gyro_z'], color='orange', label='Unscented Kalman Filtered')
axs[2].plot(time, raw_imu_filtered['Gyro_z'], color='green', label='Raw IMU Data')
axs[2].set_title("Comparison of Gyro Z")
axs[2].set_ylabel("Gyro Z (°/s)")
axs[2].set_xlabel("Time (s)")
axs[2].legend()

# Adjust layout to prevent overlap
plt.tight_layout(rect=[0, 0.03, 1, 0.95])  # Leave space for suptitle
plt.show()
