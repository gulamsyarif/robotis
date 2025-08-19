# import pandas as pd 
# import matplotlib.pyplot as plt

# # Load data from CSV
# data = pd.read_csv(r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\UKF_dibeneri\Data\jongkok\raw_imu.csv')

# # Extract data for each plot
# samples = data['Time']
# accel_x = data['Accel_x']
# accel_y = data['Accel_y']
# accel_z = data['Accel_z']
# gyro_x = data['Gyro_x']
# gyro_y = data['Gyro_y']
# gyro_z = data['Gyro_z']
# orient_x = data['Orientation_x']
# orient_y = data['Orientation_y']
# orient_z = data['Orientation_z']
# orient_w = data['Orientation_w']

# # Plot settings
# fig, axs = plt.subplots(3, 1, figsize=(10, 8), facecolor='lightgrey')  # Reduced height to make plots more compact

# # Adjust spacing around the figure and between subplots
# fig.subplots_adjust(top=0.95, bottom=0.05, left=0.1, right=0.9, hspace=0.3)

# # Smaller font size settings for compact layout
# title_fontsize = 12
# label_fontsize = 10
# legend_fontsize = 8
# tick_labelsize = 7

# # Accelerometer Plot
# axs[0].plot(samples, accel_x, color='orange', linestyle='-', label='Accel X')
# axs[0].plot(samples, accel_y, color='green', linestyle='-', label='Accel Y')
# axs[0].plot(samples, accel_z, color='brown', linestyle='-', label='Accel Z')
# axs[0].set_title('Acceleration Data', fontsize=title_fontsize)
# axs[0].set_xlabel('Data [index]', fontsize=label_fontsize)
# axs[0].set_ylabel('Acceleration value', fontsize=label_fontsize)
# axs[0].tick_params(axis='both', labelsize=tick_labelsize)
# axs[0].legend(fontsize=legend_fontsize, loc='upper right')
# axs[0].grid(True, linestyle='--')  # Add dashed grid lines

# # Gyroscope Plot
# axs[1].plot(samples, gyro_x, color='purple', linestyle='-', label='Gyro X')
# axs[1].plot(samples, gyro_y, color='red', linestyle='-', label='Gyro Y')
# axs[1].plot(samples, gyro_z, color='blue', linestyle='-', label='Gyro Z')
# axs[1].set_title('Gyroscope Data', fontsize=title_fontsize)
# axs[1].set_xlabel('Data [index]', fontsize=label_fontsize)
# axs[1].set_ylabel('Gyroscope value', fontsize=label_fontsize)
# axs[1].tick_params(axis='both', labelsize=tick_labelsize)
# axs[1].legend(fontsize=legend_fontsize, loc='upper right')
# axs[1].grid(True, linestyle='--')  # Add dashed grid lines

# # Orientation Plot
# axs[2].plot(samples, orient_x, color='blue', linestyle='-', label='Orient X')
# axs[2].plot(samples, orient_y, color='orange', linestyle='-', label='Orient Y')
# axs[2].plot(samples, orient_z, color='green', linestyle='-', label='Orient Z')
# axs[2].plot(samples, orient_w, color='black', linestyle='-', label='Orient W')
# axs[2].set_title('Orientation Data (Quaternion)', fontsize=title_fontsize)
# axs[2].set_xlabel('Data [index]', fontsize=label_fontsize)
# axs[2].set_ylabel('Quaternion value', fontsize=label_fontsize)
# axs[2].tick_params(axis='both', labelsize=tick_labelsize)
# axs[2].legend(fontsize=legend_fontsize, loc='upper right')
# axs[2].grid(True, linestyle='--')  # Add dashed grid lines

# # Save the plot as high-resolution image
# plt.tight_layout()
# plt.savefig(r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\UKF_dibeneri\Filter\high_resolution_plot.png', dpi=600)  # Adjust dpi to 300 or higher for high resolution
# plt.show()

import pandas as pd 
import matplotlib.pyplot as plt

# Load data from CSV
data = pd.read_csv(r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\UKF_dibeneri\Data\jongkok\raw_imu.csv')

# Extract data for the gyroscope plot
samples = data['Time']
gyro_x = data['Gyro_x']
gyro_y = data['Gyro_y']
gyro_z = data['Gyro_z']

# Plot settings for only the gyroscope data
fig, ax = plt.subplots(figsize=(10, 4), facecolor='lightgrey')  # Create a single subplot

# Font size settings for compact layout
title_fontsize = 12
label_fontsize = 10
legend_fontsize = 8
tick_labelsize = 7

# Gyroscope Plot
ax.plot(samples, gyro_x, color='purple', linestyle='-', label='Gyro X')
ax.plot(samples, gyro_y, color='red', linestyle='-', label='Gyro Y')
ax.plot(samples, gyro_z, color='blue', linestyle='-', label='Gyro Z')
ax.set_title('Gyroscope Data', fontsize=title_fontsize)
ax.set_xlabel('Data [index]', fontsize=label_fontsize)
ax.set_ylabel('Gyroscope value', fontsize=label_fontsize)
ax.tick_params(axis='both', labelsize=tick_labelsize)
ax.legend(fontsize=legend_fontsize, loc='upper right')
ax.grid(True, linestyle='--')  # Add dashed grid lines

# Save the gyroscope plot as high-resolution image
plt.tight_layout()
plt.savefig(r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\UKF_dibeneri\Filter\gyro_plot.png', dpi=600)  # Save only the gyroscope plot
plt.show()
