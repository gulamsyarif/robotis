import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

# Daftar nama file dan label kondisinya
file_names = {
    r'C:\Users\syari\OneDrive\Documents\Skripsii\balance_complete\balance_detection\scripts\modified\Pengujian4\datas\idle.csv': 'Idle',
    r'C:\Users\syari\OneDrive\Documents\Skripsii\balance_complete\balance_detection\scripts\modified\Pengujian4\datas\roslaunch.csv': 'ROS',
    r'C:\Users\syari\OneDrive\Documents\Skripsii\balance_complete\balance_detection\scripts\modified\Pengujian4\datas\raw.csv': 'ROS + Kalkulasi',
    r'C:\Users\syari\OneDrive\Documents\Skripsii\balance_complete\balance_detection\scripts\modified\Pengujian4\datas\WT.csv': 'ROS + Kalkulasi + DWT',
    r'C:\Users\syari\OneDrive\Documents\Skripsii\balance_complete\balance_detection\scripts\modified\Pengujian4\datas\UKF.csv': 'ROS + Kalkulasi + UKF'
}

# Dataframe untuk menyimpan statistik gabungan
all_statistics = pd.DataFrame()

# Menggunakan palet warna dari seaborn
sns.set_palette("husl")
colors = sns.color_palette("husl", len(file_names))

# Ukuran font
title_font_size = 14
label_font_size = 12
tick_font_size = 10

# Gaya garis untuk setiap kondisi
line_styles = ['-', '--', '-.', ':', (0, (3, 1, 1, 1))]

# Membuat plot untuk masing-masing kondisi
plt.figure(figsize=(16, 8))

# Plot 1: Memory Usage
plt.subplot(1, 2, 1)
for idx, (file, label) in enumerate(file_names.items()):
    # Membaca CSV
    data = pd.read_csv(file)

    # Menghitung statistik
    statistics = data.describe().loc[['mean', 'std', 'min', 'max']].T
    statistics['Condition'] = label
    all_statistics = pd.concat([all_statistics, statistics])

    # Plot data
    plt.plot(
        data.index, 
        data['Memory Usage Percentage (%)'], 
        label=label, 
        color=colors[idx], 
        linestyle=line_styles[idx], 
        linewidth=2
    )
plt.xlabel('Waktu (s)', fontsize=label_font_size)
plt.ylabel('Persentase Memori (%)', fontsize=label_font_size)
plt.title('Penggunaan Memori', fontsize=title_font_size, fontweight='bold')
plt.grid(color='gray', linestyle='--', linewidth=0.5)
plt.xticks(fontsize=tick_font_size)
plt.yticks(fontsize=tick_font_size)

# Plot 2: CPU Usage
plt.subplot(1, 2, 2)
for idx, (file, label) in enumerate(file_names.items()):
    data = pd.read_csv(file)
    plt.plot(
        data.index, 
        data['CPU Usage Percentage (%)'], 
        label=label, 
        color=colors[idx], 
        linestyle=line_styles[idx], 
        linewidth=2
    )
plt.xlabel('Waktu (s)', fontsize=label_font_size)
plt.ylabel('Persentase CPU (%)', fontsize=label_font_size)
plt.title('Penggunaan CPU', fontsize=title_font_size, fontweight='bold')
plt.grid(color='gray', linestyle='--', linewidth=0.5)
plt.xticks(fontsize=tick_font_size)
plt.yticks(fontsize=tick_font_size)

# Menambahkan legenda di bagian kanan luar plot
plt.legend(
    labels=file_names.values(), 
    loc='center left', 
    bbox_to_anchor=(1.05, 0.5), 
    fontsize=label_font_size, 
    frameon=True, 
    shadow=True, 
    borderpad=1.0
)

# Menyusun tata letak agar tidak saling tumpang tindih
plt.tight_layout(rect=[0, 0, 0.9, 1])

# Menyimpan grafik dalam resolusi tinggi
plt.savefig(
    r'C:\Users\syari\OneDrive\Documents\Skripsii\balance_complete\balance_detection\scripts\modified\Pengujian4\datas\modified_graph.png',
    dpi=600
)

# Menampilkan grafik
plt.show()

# Menampilkan statistik
print("Statistik Gabungan:")
print(all_statistics)
