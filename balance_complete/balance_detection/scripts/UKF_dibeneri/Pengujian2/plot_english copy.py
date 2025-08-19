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

# Adjust position of the main title
fig.suptitle(
    "Perbandingan Koordinat CoM pada Berbagai Posisi", 
    fontsize=20, 
    y=0.92  # Mengatur jarak dari suptitle ke subplot pertama
)

# Define custom titles for each subplot
custom_titles = [
    "Jongkok Statis (Sumbu X)",
    "Jongkok Statis (Sumbu Y)",
    "Berdiri Statis (Sumbu X)",
    "Berdiri Statis (Sumbu Y)",
    "Berjalan (Sumbu X)",
    "Berjalan (Sumbu Y)",
    "Menendang (Sumbu X)",
    "Menendang (Sumbu Y)"
]

# Iterate through each CSV file and corresponding subplot
for idx, csv_file in enumerate(csv_files):
    # Load and filter data
    df = pd.read_csv(csv_file)
    df = df.iloc[300:1500]
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
    axs[idx, 0].fill_between(df.index, df['x_min'], df['x_max'], color='lightblue', label='Support Polygon Area')
    axs[idx, 0].plot(df.index, df['com_x'], color='green', linestyle='--', linewidth=2, label='CoM')
    axs[idx, 0].plot(df.index, df['zmp_x'], color='red', linewidth=2, label='ZMP')  # Add ZMP
    axs[idx, 0].set_title(custom_titles[idx * 2], fontsize=16, pad=10)
    axs[idx, 0].set_xlabel('Data [index]')
    axs[idx, 0].set_ylabel('Nilai (cm)')

    # Plot Y-axis data
    axs[idx, 1].fill_between(df.index, df['y_min'], df['y_max'], color='lightblue', label='Support Polygon Area')
    axs[idx, 1].plot(df.index, df['com_y'], color='green', linestyle='--', linewidth=2, label='CoM')
    axs[idx, 1].plot(df.index, df['zmp_y'], color='red', linewidth=2, label='ZMP')  # Add ZMP
    axs[idx, 1].set_title(custom_titles[idx * 2 + 1], fontsize=16, pad=10)
    axs[idx, 1].set_xlabel('Data [index]')
    axs[idx, 1].set_ylabel('Nilai (cm)')

# Mengumpulkan handle dan label dari subplot pertama
handles, labels = axs[0, 0].get_legend_handles_labels()

# Membuat legend global
fig.legend(handles, labels, loc='lower center', bbox_to_anchor=(0.5, 0.02), ncol=3, fontsize=16)

# Menyesuaikan layout agar tidak memotong suptitle dan legend
plt.tight_layout(rect=[0, 0.05, 1, 0.92])  # Tambahkan margin untuk suptitle

# Simpan plot
plt.savefig(
    r'C:\Users\syari\OneDrive\Documents\Skripsii\balance_complete\balance_detection\scripts\UKF_dibeneri\Pengujian2\pengujian2.png',
    dpi=600,
    bbox_inches='tight'  # Menjamin suptitle dan legend termasuk dalam gambar
)

# Tampilkan plot
plt.show()
