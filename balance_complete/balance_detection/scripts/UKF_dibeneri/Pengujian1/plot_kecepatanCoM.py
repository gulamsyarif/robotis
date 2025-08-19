import pandas as pd
import matplotlib.pyplot as plt

# List of files and labels
csv_files_with_labels = [
    (r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\UKF_dibeneri\Pengujian1\berjalan1\WT_complete.csv', 'Wavelet Transformed IMU'),
    (r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\UKF_dibeneri\Pengujian1\berjalan1\UKF_complete.csv', 'Unscented Kalman Filtered IMU'),
    (r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\UKF_dibeneri\Pengujian1\berjalan1\raw_complete.csv', 'Raw IMU')
]

# Read and filter data for a specific range
data_frames = []
offsets = [0, 110, 220]  # Offsets to separate data with distance
for (file_path, label), offset in zip(csv_files_with_labels, offsets):
    df = pd.read_csv(file_path)
    df_filtered = df.loc[700:800, ['proj_com_x', 'proj_com_y', 'proj_com_z']].reset_index(drop=True)
    df_filtered.index += offset  # Adding offset to separate data segments
    df_filtered['Label'] = label    
    data_frames.append(df_filtered)

# Combine data for easier plotting
combined_df = pd.concat(data_frames, ignore_index=False)

# Plot for proj_com_x, proj_com_y, and proj_com_z without sharex
fig, axes = plt.subplots(3, 1, figsize=(12, 10))
components = ['proj_com_x', 'proj_com_y', 'proj_com_z']
titles = ['Comparison for Projected CoM X', 'Comparison for Projected CoM Y', 'Comparison for Projected CoM Z']
fig.suptitle("Comparison of Projected CoM on Ground Coordinates Across Filtering Methods while Robot in Walking State", fontsize=16)

# Vertical lines position for segment separation
vertical_lines = [110, 220]

for i, component in enumerate(components):
    for label in combined_df['Label'].unique():
        subset = combined_df[combined_df['Label'] == label]
        axes[i].plot(subset.index, subset[component], label=label)
    
    axes[i].set_title(titles[i])
    axes[i].set_ylabel(component)
    axes[i].grid(True, which='both', axis='y', linestyle='--', color='gray', alpha=0.5)
    axes[i].legend()

    # Position legend to lower left corner
    axes[i].legend(loc='lower left')

    # Add vertical separation lines
    for x_pos in vertical_lines:
        axes[i].axvline(x=x_pos, color='gray', linestyle='-', linewidth=1.5, alpha=0.5)

    # Set custom xticks to display "0 50 100" for each segment
    xticks_positions = [0, 50, 100, 120, 170, 220, 240, 290, 340]
    xticks_labels = ['0', '50', '100', '0', '50', '100', '0', '50', '100']
    axes[i].set_xticks(xticks_positions)
    axes[i].set_xticklabels(xticks_labels)

# Set x-axis range to display distance between segments
axes[0].set_xlim(-10, 350)  # Add x-axis limits to give space between segments

# Set display to capture small variations with a tighter scale
for ax in axes:
    ax.margins(y=0.1)  # Looser y margin

# Adjust layout to create space for the title
plt.tight_layout(rect=[0, 0, 1, 0.95])  # Adjusting the top to create space for the main title

plt.show()
