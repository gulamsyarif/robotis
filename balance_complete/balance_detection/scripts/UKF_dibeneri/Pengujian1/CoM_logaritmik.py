# import pandas as pd
# import numpy as np
# import matplotlib.pyplot as plt

# # Daftar file dan label untuk plot
# csv_files_with_labels = [
#     (r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\UKF_dibeneri\Data\berjalan1\WT_complete.csv', 'Wavelet Transformed IMU'),
#     (r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\UKF_dibeneri\Data\berjalan1\UKF_complete.csv', 'Unscented Kalman Filtered IMU'),
#     (r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\UKF_dibeneri\Data\berjalan1\raw_complete.csv', 'Raw IMU')
# ]

# # Kolom yang akan diambil dari setiap file CSV
# components = ['com_x', 'com_y', 'com_z']

# # Membuat plot logaritmik
# plt.figure(figsize=(10, 6))
# for file_path, label in csv_files_with_labels:
#     # Membaca data dari file CSV
#     data = pd.read_csv(file_path)
    
#     # Mengambil logaritma dari kolom yang diinginkan
#     log_data = data[components].applymap(lambda x: np.log10(abs(x)) if x != 0 else 0)
    
#     # Plot masing-masing komponen
#     for component in components:
#         plt.plot(log_data[component], label=f'{label} - {component}')

# # Mengatur judul dan label sumbu
# plt.title('Logarithmic Plot of IMU Components')
# plt.xlabel('Index')
# plt.ylabel('Logarithmic Value (log10)')
# plt.legend()
# plt.grid(True)
# plt.show()

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Daftar file dan label untuk plot
csv_files_with_labels = [
    (r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\UKF_dibeneri\Data\berjalan1\WT_complete.csv', 'Wavelet Transformed IMU'),
    (r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\UKF_dibeneri\Data\berjalan1\UKF_complete.csv', 'Unscented Kalman Filtered IMU'),
    (r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\UKF_dibeneri\Data\berjalan1\raw_complete.csv', 'Raw IMU')
]

# Kolom yang akan diambil dari setiap file CSV
components = ['com_x', 'com_y', 'com_z']

# Membuat subplots untuk setiap komponen
fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
fig.suptitle('Logarithmic Plot of IMU Components', y=0.94)  # Atur jarak dengan y parameter

# Mengatur setiap plot untuk masing-masing komponen
for idx, component in enumerate(components):
    for file_path, label in csv_files_with_labels:
        # Membaca data dari file CSV
        data = pd.read_csv(file_path)
        
        # Mengambil logaritma dari kolom yang diinginkan
        log_data = data[component].apply(lambda x: np.log10(abs(x)) if x != 0 else 0)
        
        # Plot data untuk komponen spesifik
        axs[idx].plot(log_data, label=f'{label}')
    
    # Menambahkan judul dan label untuk masing-masing subplot
    axs[idx].set_title(f'Logarithmic Plot of {component}')
    axs[idx].set_ylabel('Logarithmic Value (log10)')
    axs[idx].legend()
    axs[idx].grid(True)

# Menambahkan label sumbu x pada plot terakhir
axs[-1].set_xlabel('Index')

plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.subplots_adjust(hspace=0.2)
plt.savefig(r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\UKF_dibeneri\Pengujian1\CoM.png', dpi=600)  # Save only the gyroscope plot
plt.show()

# import pandas as pd
# import numpy as np
# import matplotlib.pyplot as plt

# # Daftar file dan label untuk plot
# csv_files_with_labels = [
#     (r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\UKF_dibeneri\Data\berjalan1\WT_complete.csv', 'Wavelet Transformed IMU'),
#     (r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\UKF_dibeneri\Data\berjalan1\UKF_complete.csv', 'Unscented Kalman Filtered IMU'),
#     (r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\UKF_dibeneri\Data\berjalan1\raw_complete.csv', 'Raw IMU')
# ]

# # Kolom yang akan diambil dari setiap file CSV
# components = ['com_x', 'com_y', 'com_z']

# # Rentang y-axis khusus untuk masing-masing komponen
# y_ranges = {
#     'com_x': (-2, 2),   # Sesuaikan rentang ini sesuai data 'com_x'
#     'com_y': (-2, 2),   # Sesuaikan rentang ini sesuai data 'com_y'
#     'com_z': (-3, 1)    # Sesuaikan rentang ini sesuai data 'com_z'
# }

# # Membuat subplots untuk setiap komponen
# fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
# fig.suptitle('Logarithmic Plot of IMU Components (Normalized with Custom Y-Axis Ranges)')

# # Mengatur setiap plot untuk masing-masing komponen
# for idx, component in enumerate(components):
#     for file_path, label in csv_files_with_labels:
#         # Membaca data dari file CSV
#         data = pd.read_csv(file_path)
        
#         # Normalisasi data ke rentang -1 hingga 1
#         max_abs_value = data[component].abs().max()
#         normalized_data = data[component] / max_abs_value if max_abs_value != 0 else data[component]
        
#         # Mengambil logaritma dari data yang sudah dinormalisasi
#         log_data = normalized_data.apply(lambda x: np.log10(abs(x)) if x != 0 else 0)
        
#         # Plot data untuk komponen spesifik
#         axs[idx].plot(log_data, label=f'{label}')
    
#     # Menambahkan judul dan label untuk masing-masing subplot
#     axs[idx].set_title(f'Logarithmic Plot of {component}')
#     axs[idx].set_ylabel('Logarithmic Value (log10)')
#     axs[idx].legend()
#     axs[idx].grid(True)
    
#     # Menetapkan rentang y-axis sesuai komponen
#     axs[idx].set_ylim(y_ranges[component])

# # Menambahkan label sumbu x pada plot terakhir
# axs[-1].set_xlabel('Index')

# plt.tight_layout(rect=[0, 0.03, 1, 0.95])
# plt.show()
