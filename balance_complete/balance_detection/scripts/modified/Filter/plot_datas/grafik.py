import pandas as pd
import matplotlib.pyplot as plt

# Load data from CSV files
raw_imu = pd.read_csv(r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\modified\Filter\jongkok\raw_imu.csv').iloc[1000:1101]
ukf_filtered = pd.read_csv(r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\modified\Filter\jongkok\UKF_filtered.csv').iloc[1000:1101]
wt_filtered = pd.read_csv(r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\modified\Filter\jongkok\WT_filtered.csv').iloc[1000:1101]

# Set up the figure and subplots for each axis
fig, axs = plt.subplots(3, 1, figsize=(12, 8))
time = raw_imu['Time']  # Time data from raw_imu dataset

# Main title for all subplots
fig.suptitle("Comparison of IMU Data Across Filtering Methods while Robot in Static Squat Position", fontsize=16)

# Plotting Gyro X data comparison
axs[0].plot(time, wt_filtered['Denoised_Gyro_x'], color='blue', label='Discrete Wavelet Transformed')
axs[0].plot(time, ukf_filtered['Denoised_Gyro_x'], color='orange', label='Unscented Kalman Filtered')
axs[0].plot(time, raw_imu['Gyro_x'], color='green', label='Raw IMU Data')
axs[0].set_title("Comparison of Gyro X")
axs[0].set_ylabel("Gyro X")
axs[0].legend()

# Plotting Gyro Y data comparison
axs[1].plot(time, wt_filtered['Denoised_Gyro_y'], color='blue', label='Discrete Wavelet Transformed')
axs[1].plot(time, ukf_filtered['Denoised_Gyro_y'], color='orange', label='Unscented Kalman Filtered')
axs[1].plot(time, raw_imu['Gyro_y'], color='green', label='Raw IMU Data')
axs[1].set_title("Comparison of Gyro Y")
axs[1].set_ylabel("Gyro Y")
axs[1].legend()

# Plotting Gyro Z data comparison
axs[2].plot(time, wt_filtered['Denoised_Gyro_z'], color='blue', label='Discrete Wavelet Transformed')
axs[2].plot(time, ukf_filtered['Denoised_Gyro_z'], color='orange', label='Unscented Kalman Filtered')
axs[2].plot(time, raw_imu['Gyro_z'], color='green', label='Raw IMU Data')
axs[2].set_title("Comparison of Gyro Z")
axs[2].set_ylabel("Gyro Z")
axs[2].set_xlabel("Time")
axs[2].legend()

# Adjust layout to prevent overlap
plt.tight_layout(rect=[0, 0.03, 1, 0.95])  # Leave space for suptitle
plt.show()
