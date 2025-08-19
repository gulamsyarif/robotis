import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import cv2

# Tentukan file CSV yang akan digunakan
csv_file = r'C:\Users\syari\OneDrive\Documents\Skripsii\balance_complete\balance_detection\scripts\UKF_dibeneri\Data\berjalan1\UKF_complete.csv'

# Load CSV file
df = pd.read_csv(csv_file)
# df = df.iloc[300:1500]
df = df.reset_index(drop=True)

# Tentukan koordinat support polygon berdasarkan support phase
df['x_min'] = df['support_phase'].apply(lambda sp: -8.6 if sp == 'Double Support' else -4.3)
df['x_max'] = df['support_phase'].apply(lambda sp: 8.6 if sp == 'Double Support' else 4.3)
df['y_min'] = -6.5
df['y_max'] = 6.5

df['x_min'] = pd.to_numeric(df['x_min'], errors='coerce')
df['x_max'] = pd.to_numeric(df['x_max'], errors='coerce')
df['y_min'] = pd.to_numeric(df['y_min'], errors='coerce')
df['y_max'] = pd.to_numeric(df['y_max'], errors='coerce')

# Fill NaN values
df.fillna({'x_min': -4.3, 'x_max': 4.3, 'y_min': -6.5, 'y_max': 6.5}, inplace=True)

# Fungsi untuk menentukan kondisi keseimbangan
def check_balance(zmp_x, zmp_y, com_x, com_y, x_min, x_max, y_min, y_max):
    in_support_polygon = (x_min <= zmp_x <= x_max) and (y_min <= zmp_y <= y_max)
    com_in_polygon = (x_min <= com_x <= x_max) and (y_min <= com_y <= y_max)

    distance_zmp = np.sqrt(zmp_x**2 + zmp_y**2)
    distance_com = np.sqrt(com_x**2 + com_y**2)

    if in_support_polygon and com_in_polygon:
        return "Robot Seimbang"
    elif in_support_polygon and not com_in_polygon:
        return "Robot Tidak Seimbang"
    elif not in_support_polygon and com_in_polygon:
        return "Robot Tidak Seimbang"
    elif not in_support_polygon and distance_zmp < 500 and distance_com < 500:
        return "Robot Tidak Seimbang"
    elif not in_support_polygon and distance_zmp >= 500 and distance_com >= 500:
        return "Robot Jatuh"
    else:
        return "Kondisi Tidak Dikenali"

# Inisialisasi figure dan axis
fig, axs = plt.subplots(1, 2, figsize=(15, 8))
fig.suptitle("Perbandingan Posisi CoM, ZMP, dan area Support Polygon", fontsize=20, y=0.97)

# Variabel global untuk menyimpan elemen teks balance condition
balance_text = None

# Fungsi untuk memperbarui frame pada animasi
def update_frame(i):
    global balance_text

    axs[0].cla()
    axs[1].cla()

    window_size = 100
    start_idx = max(0, i - window_size)

    # Plot X-axis data
    axs[0].fill_between(df.index[start_idx:i], df['x_min'][start_idx:i], df['x_max'][start_idx:i], color='lightblue', label='Support Polygon Area')
    axs[0].plot(df.index[start_idx:i], df['com_x'][start_idx:i], color='green', linestyle='--', linewidth=2, label='CoM')
    axs[0].plot(df.index[start_idx:i], df['zmp_x'][start_idx:i], color='red', linewidth=2, label='ZMP')
    axs[0].set_title("Sumbu X", fontsize=16, pad=10)
    axs[0].set_xlabel('Data [index]')
    axs[0].set_ylabel('Nilai (cm)')

    # Plot Y-axis data
    axs[1].fill_between(df.index[start_idx:i], df['y_min'][start_idx:i], df['y_max'][start_idx:i], color='lightblue', label='Support Polygon Area')
    axs[1].plot(df.index[start_idx:i], df['com_y'][start_idx:i], color='green', linestyle='--', linewidth=2, label='CoM')
    axs[1].plot(df.index[start_idx:i], df['zmp_y'][start_idx:i], color='red', linewidth=2, label='ZMP')
    axs[1].set_title("Sumbu Y", fontsize=16, pad=10)
    axs[1].set_xlabel('Data [index]')
    axs[1].set_ylabel('Nilai (cm)')

    # Check balance condition
    balance_condition = check_balance(df['zmp_x'][i], df['zmp_y'][i], df['com_x'][i], df['com_y'][i], df['x_min'][i], df['x_max'][i], df['y_min'][i], df['y_max'][i])

    # Tetapkan warna box berdasarkan kondisi
    if balance_condition == "Robot Seimbang":
        bbox_props = dict(facecolor='palegreen', edgecolor='silver', boxstyle='round,pad=0.5')
    elif balance_condition == "Robot Tidak Seimbang":
        bbox_props = dict(facecolor='cornsilk', edgecolor='silver', boxstyle='round,pad=0.5')
    elif balance_condition == "Robot Jatuh":
        bbox_props = dict(facecolor='salmon', edgecolor='silver', boxstyle='round,pad=0.5')
    else:
        bbox_props = dict(facecolor='lightgray', edgecolor='silver', boxstyle='round,pad=0.5')

    # Tambahkan teks baru dengan kotak
    global balance_text
    if balance_text is not None:
        balance_text.remove()

    balance_text = fig.text(
        0.5, 0.87, balance_condition, ha='center', va='center', fontsize=16, color='black', bbox=bbox_props
    )

    # Membuat legend global
    handles, labels = axs[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='lower center', bbox_to_anchor=(0.52, 0.02), ncol=3, fontsize=16)

    # Menyesuaikan layout agar tidak memotong suptitle dan legend
    plt.tight_layout(rect=[0, 0.05, 1, 0.92])

# Membuat animasi
ani = animation.FuncAnimation(fig, update_frame, frames=len(df), interval=16, repeat=False)
plt.show()
