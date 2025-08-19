import pandas as pd
import matplotlib.pyplot as plt

# List of files and labels
csv_files_with_labels = [
    (r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\UKF_dibeneri\Pengujian1\nendang\WT_complete.csv', 'Wavelet Transformed IMU'),
    (r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\UKF_dibeneri\Pengujian1\nendang\UKF_complete.csv', 'Unscented Kalman Filtered IMU'),
    (r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\UKF_dibeneri\Pengujian1\nendang\raw_complete.csv', 'Raw IMU')
]

# Rentang data yang difilter dan offset untuk memisahkan segmen data
data_range = (400, 600)
offsets = [0, 220, 440]

# Membaca dan memfilter data dari setiap file
data_frames = []
for (file_path, label), offset in zip(csv_files_with_labels, offsets):
    df = pd.read_csv(file_path)
    df_filtered = df.loc[data_range[0]:data_range[1], ['com_x', 'com_y', 'com_z']].reset_index(drop=True)
    df_filtered.index += offset  # Menambahkan offset untuk memisahkan segmen data
    df_filtered['Label'] = label
    data_frames.append(df_filtered)

# Menggabungkan semua data untuk kemudahan plotting
combined_df = pd.concat(data_frames, ignore_index=False)

# Pengaturan plot untuk com_x, com_y, dan com_z tanpa sharex
fig, axes = plt.subplots(3, 1, figsize=(12, 10))
components = ['com_x', 'com_y', 'com_z']
titles = ['Comparison for CoM X', 'Comparison for CoM Y', 'Comparison for CoM Z']
y_labels = ['CoM X (cm)', 'CoM Y (cm)', 'CoM Z (cm)']  # Custom y-axis labels
fig.suptitle("Comparison of CoM Coordinates Across Filtering Methods while Robot in Kicking Condition", fontsize=16)

# Posisi garis vertikal untuk pemisahan segmen
vertical_lines = [220, 440]
xticks_positions = [0, 50, 100, 150, 200, 220, 270, 320, 370, 420, 440, 490, 540, 590, 640]
xticks_labels = ['0', '50', '100', '150', '200', '0', '50', '100', '150', '200', '0', '50', '100', '150', '200']

# Menyimpan batas maksimum sumbu x
x_max = None

for i, component in enumerate(components):
    for label in combined_df['Label'].unique():
        subset = combined_df[combined_df['Label'] == label]
        axes[i].plot(subset.index, subset[component], label=label)

    axes[i].set_title(titles[i])
    axes[i].set_ylabel(y_labels[i])
    axes[i].set_xlabel('Data [index]')
    axes[i].grid(True, which='both', axis='y', linestyle='--', color='gray', alpha=0.5)
    axes[2].legend(loc='lower right', bbox_to_anchor=(1.1, 0.9), borderaxespad=0.)

    # Menambahkan garis pemisah vertikal dan label sumbu X khusus
    for x_pos in vertical_lines:
        axes[i].axvline(x=x_pos, color='gray', linestyle='-', linewidth=1.5, alpha=0.5)
    axes[i].set_xticks(xticks_positions)
    axes[i].set_xticklabels(xticks_labels)

    # Mengambil batas maksimum untuk sumbu x
    if x_max is None or subset.index.max() > x_max:
        x_max = subset.index.max()

plt.subplots_adjust(hspace=0.6)

# Menetapkan batas sumbu x yang sama berdasarkan nilai maksimum
for ax in axes:
    ax.set_xlim(0, x_max + 10)  # Menetapkan batas yang sama untuk semua sumbu x, dengan sedikit ruang

# Menyesuaikan tampilan
for ax in axes:
    ax.margins(y=0.1)  # Margin y untuk variasi kecil

# Mengatur layout untuk menyesuaikan judul utama
plt.tight_layout(rect=[1, 1, 1, 0.95])
plt.show()
