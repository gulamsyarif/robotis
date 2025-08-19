import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import cv2

# Tentukan file CSV yang akan digunakan
csv_file = r'C:\Users\syari\OneDrive\Documents\Skripsii\balance_complete\balance_detection\scripts\UKF_dibeneri\Data\berjalan1\UKF_complete.csv'

# Load CSV file
df = pd.read_csv(csv_file)
# df = df.iloc[300:310]
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

# Inisialisasi figure dan axis
fig, axs = plt.subplots(2, 1, figsize=(10, 10))

# Judul grafik
fig.suptitle("Perbandingan Koordinat CoM pada Posisi Jongkok Statis", fontsize=20, y=0.92)

# Setup writer untuk menyimpan video menggunakan OpenCV
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter(r'C:\Users\syari\OneDrive\Documents\Skripsii\balance_complete\balance_detection\scripts\UKF_dibeneri\VideoDemo\berjalan1.avi', fourcc, 10.0, (1920, 1080))

# Fungsi untuk memperbarui grafik pada setiap frame
def update_frame(i):
    # Clear previous plots
    axs[0].cla()
    axs[1].cla()

    # Plot X-axis data
    axs[0].fill_between(df.index[:i], df['x_min'][:i], df['x_max'][:i], color='lightblue', label='Support Polygon Area')
    axs[0].plot(df.index[:i], df['com_x'][:i], color='green', linestyle='--', linewidth=2, label='CoM')
    axs[0].plot(df.index[:i], df['zmp_x'][:i], color='red', linewidth=2, label='ZMP')  # Add ZMP
    axs[0].set_title("Jongkok Statis (Sumbu X)", fontsize=16, pad=10)
    axs[0].set_xlabel('Data [index]')
    axs[0].set_ylabel('Nilai (cm)')

    # Plot Y-axis data
    axs[1].fill_between(df.index[:i], df['y_min'][:i], df['y_max'][:i], color='lightblue', label='Support Polygon Area')
    axs[1].plot(df.index[:i], df['com_y'][:i], color='green', linestyle='--', linewidth=2, label='CoM')
    axs[1].plot(df.index[:i], df['zmp_y'][:i], color='red', linewidth=2, label='ZMP')  # Add ZMP
    axs[1].set_title("Jongkok Statis (Sumbu Y)", fontsize=16, pad=10)
    axs[1].set_xlabel('Data [index]')
    axs[1].set_ylabel('Nilai (cm)')

    # Mengumpulkan handle dan label dari subplot pertama
    handles, labels = axs[0].get_legend_handles_labels()

    # Membuat legend global
    fig.legend(handles, labels, loc='lower center', bbox_to_anchor=(0.5, 0.02), ncol=3, fontsize=16)

    # Menyesuaikan layout agar tidak memotong suptitle dan legend
    plt.tight_layout(rect=[0, 0.05, 1, 0.92])  # Tambahkan margin untuk suptitle

    # Render gambar
    fig.canvas.draw()

    # Get the image from the canvas as an array (in RGBA format)
    img = np.array(fig.canvas.renderer.buffer_rgba())

    # Ubah menjadi format RGB
    img = cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)

    # Write the frame to the video
    out.write(img)

# Membuat animasi
ani = animation.FuncAnimation(fig, update_frame, frames=len(df), repeat=False)

# Menyimpan video
out.release()

# Tampilkan plot jika diperlukan
plt.show()
