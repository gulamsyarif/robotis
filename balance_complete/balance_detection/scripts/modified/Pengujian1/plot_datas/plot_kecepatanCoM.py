import pandas as pd
import os
import matplotlib.pyplot as plt

# Load CSV files (tambahkan .csv di akhir file path)
csv_files = [r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\modified\Pengujian1\berjalan1\raw_clean.csv',
             r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\modified\Pengujian1\berjalan1\UKF_clean.csv',
             r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\modified\Pengujian1\berjalan1\WT_clean.csv']

# Tentukan nama-nama plot sesuai keinginan
plot_labels = ['Raw IMU', 'Unscented Kalman Filtered IMU', 'Wavelet Transformed IMU']  # Ganti dengan label yang Anda inginkan

# Load the CSV files
dataframes = [pd.read_csv(file) for file in csv_files]

# Plot CoM (x, y, z) for each file
fig, ax = plt.subplots(3, 1, figsize=(10, 12))  # 3 subplots for x, y, z

for i, (df, label) in enumerate(zip(dataframes, plot_labels)):
    # Plot CoM x
    ax[0].plot(df['proj_com_x'], label=label)
    ax[0].set_title('Projected CoM X', fontsize=12)
    ax[0].set_xlabel('Index', fontsize=10)
    ax[0].set_ylabel('Projected com_x', fontsize=10)

    # Plot CoM y
    ax[1].plot(df['proj_com_y'], label=label)
    ax[1].set_title('Projected CoM Y', fontsize=12)
    ax[1].set_xlabel('Index', fontsize=10)
    ax[1].set_ylabel('Projected com_y', fontsize=10)

    # Plot CoM z
    ax[2].plot(df['proj_com_z'], label=label)
    ax[2].set_title('Projected CoM Z', fontsize=12)
    ax[2].set_xlabel('Index', fontsize=10)
    ax[2].set_ylabel('Projected com_z', fontsize=10)

# Add legend
for a in ax:
    a.legend()

# Adjust layout and space between plots
plt.tight_layout()  # This ensures subplots fit nicely
plt.subplots_adjust(hspace=0.4)  # Adjust the height spacing between subplots

# Show the plot
plt.show()
