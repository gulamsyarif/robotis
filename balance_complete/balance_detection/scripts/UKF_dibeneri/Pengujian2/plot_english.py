# import pandas as pd
# import matplotlib.pyplot as plt

# # Load CSV file
# csv_file = r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\UKF_dibeneri\Data\jongkok\UKF_complete.csv'
# df = pd.read_csv(csv_file)
# df = df.iloc[500:700]

# # Reset index to start from 0 for the x-axis
# df = df.reset_index(drop=True)

# # Tentukan koordinat support polygon berdasarkan support phase
# df['x_min'] = df['support_phase'].apply(lambda sp: -8.6 if sp == 'Double Support' else -4.3)
# df['x_max'] = df['support_phase'].apply(lambda sp: 8.6 if sp == 'Double Support' else 4.3)
# df['y_min'] = -6.5
# df['y_max'] = 6.5

# # Konversi x_min, x_max, y_min, y_max menjadi numerik jika perlu
# df['x_min'] = pd.to_numeric(df['x_min'], errors='coerce')
# df['x_max'] = pd.to_numeric(df['x_max'], errors='coerce')
# df['y_min'] = pd.to_numeric(df['y_min'], errors='coerce')
# df['y_max'] = pd.to_numeric(df['y_max'], errors='coerce')

# # Periksa jika ada nilai NaN dan isi dengan nilai default
# df.fillna({'x_min': -4.3, 'x_max': 4.3, 'y_min': -6.5, 'y_max': 6.5}, inplace=True)

# # Create subplots
# fig, axs = plt.subplots(2, 1, figsize=(10, 12))
# fig.suptitle("Comparison of CoM Coordinates Across Filtering Methods while Robot in Kicking Condition", fontsize=16)

# # Plot for X values
# axs[0].fill_between(df.index, df['x_min'], df['x_max'], color='cornsilk', label='Support Polygon Area')
# # axs[0].plot(df.index, df['zmp_x'], color='palevioletred', linewidth=2, label='ZMP')
# axs[0].plot(df.index, df['com_x'], color='olive', linestyle='--', linewidth=2, label='CoM')
# axs[0].set_title('ZMP, CoM, and Support Polygon Area Over Time (X-axis)')
# axs[0].set_xlabel('Time (ms)')
# axs[0].set_ylabel('Value (cm)')
# axs[0].legend()

# # Plot for Y values
# axs[1].fill_between(df.index, df['y_min'], df['y_max'], color='cornsilk', label='Support Polygon Area')
# # axs[1].plot(df.index, df['zmp_y'], color='palevioletred', linewidth=2, label='ZMP')
# axs[1].plot(df.index, df['com_y'], color='olive', linestyle='--', linewidth=2, label='CoM')
# axs[1].set_title('ZMP, CoM, and Support Polygon Area Over Time (Y-axis)')
# axs[1].set_xlabel('Time (ms)')
# axs[1].set_ylabel('Value (cm)')
# axs[1].legend()

# # Show plot
# plt.tight_layout()
# plt.subplots_adjust(hspace=0.3, top=0.90)
# plt.savefig(r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\UKF_dibeneri\Pengujian2\jongkok.png', dpi=600)  # Save only the gyroscope plot
# plt.show()

import pandas as pd
import matplotlib.pyplot as plt

# Load CSV files
csv_files = [
    r'C:\Users\syari\OneDrive\Documents\Skripsii\balance_complete\balance_detection\scripts\UKF_dibeneri\Data\jongkok\UKF_complete.csv',
    r'C:\Users\syari\OneDrive\Documents\Skripsii\balance_complete\balance_detection\scripts\UKF_dibeneri\Data\berdiri_statis\UKF_complete.csv',
    r'C:\Users\syari\OneDrive\Documents\Skripsii\balance_complete\balance_detection\scripts\UKF_dibeneri\Data\berjalan1\UKF_complete.csv',
    r'C:\Users\syari\OneDrive\Documents\Skripsii\balance_complete\balance_detection\scripts\UKF_dibeneri\Data\nendang\UKF_complete.csv'
]

# Initialize figure and axes
fig, axs = plt.subplots(4, 2, figsize=(15, 20))
fig.suptitle("Perbandingan Koordinat CoM pada Berbagai Posisi", fontsize=18)

# Define custom titles for each subplot
custom_titles = [
    "Duduk Statis - CoM, and Support Polygon Area (X-axis)",
    "Static Squat - CoM, and Support Polygon Area (Y-axis)",
    "Static Standing - CoM, and Support Polygon Area (X-axis)",
    "Static Standing - CoM, and Support Polygon Area (Y-axis)",
    "Walking - CoM, and Support Polygon Area (X-axis)",
    "Walking - CoM, and Support Polygon Area (Y-axis)",
    "Kicking - CoM, and Support Polygon Area (X-axis)",
    "Kicking - CoM, and Support Polygon Area (Y-axis)"
]

# Iterate through each CSV file and corresponding subplot
for idx, csv_file in enumerate(csv_files):
    # Load and filter data
    df = pd.read_csv(csv_file)
    df = df.iloc[500:1500]
    df = df.reset_index(drop=True)

    # Tentukan koordinat support polygon berdasarkan support phase
    df['x_min'] = df['support_phase'].apply(lambda sp: -8.6 if sp == 'Double Support' else -4.3)
    df['x_max'] = df['support_phase'].apply(lambda sp: 8.6 if sp == 'Double Support' else 4.3)
    df['y_min'] = -6.5
    df['y_max'] = 6.5

    # Konversi x_min, x_max, y_min, y_max menjadi numerik jika perlu
    df['x_min'] = pd.to_numeric(df['x_min'], errors='coerce')
    df['x_max'] = pd.to_numeric(df['x_max'], errors='coerce')
    df['y_min'] = pd.to_numeric(df['y_min'], errors='coerce')
    df['y_max'] = pd.to_numeric(df['y_max'], errors='coerce')

    # Fill NaN values
    df.fillna({'x_min': -4.3, 'x_max': 4.3, 'y_min': -6.5, 'y_max': 6.5}, inplace=True)

    # Plot X-axis data
    axs[idx, 0].fill_between(df.index, df['x_min'], df['x_max'], color='cornsilk', label='Support Polygon Area')
    axs[idx, 0].plot(df.index, df['com_x'], color='olive', linestyle='--', linewidth=2, label='CoM')
    axs[idx, 0].plot(df.index, df['zmp_x'], color='palevioletred', linewidth=2, label='ZMP')  # Add ZMP
    axs[idx, 0].set_title(custom_titles[idx * 2], fontsize=10, pad=10)  # Use custom title for X-axis plot
    axs[idx, 0].set_xlabel('Time (ms)')
    axs[idx, 0].set_ylabel('Value (cm)')
    axs[idx, 0].legend(loc='upper right', fontsize=8)

    # Plot Y-axis data
    axs[idx, 1].fill_between(df.index, df['y_min'], df['y_max'], color='cornsilk', label='Support Polygon Area')
    axs[idx, 1].plot(df.index, df['com_y'], color='olive', linestyle='--', linewidth=2, label='CoM')
    axs[idx, 1].plot(df.index, df['zmp_y'], color='palevioletred', linewidth=2, label='ZMP')  # Add ZMP
    axs[idx, 1].set_title(custom_titles[idx * 2 + 1], fontsize=10, pad=10)  # Use custom title for Y-axis plot
    axs[idx, 1].set_xlabel('Time (ms)')
    axs[idx, 1].set_ylabel('Value (cm)')
    axs[idx, 1].legend(loc='upper right', fontsize=8)

# Adjust layout and save the plot
plt.tight_layout()
plt.subplots_adjust(hspace=0.5, top=0.95)
plt.savefig(r'C:\Users\syari\OneDrive\Documents\Skripsii\balance_complete\balance_detection\scripts\UKF_dibeneri\Pengujian2\pengujian2.png', dpi=600)
plt.show()



