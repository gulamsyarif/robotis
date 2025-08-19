# import pandas as pd
# import ast
# import numpy as np
# import matplotlib.pyplot as plt
# from sklearn.cluster import KMeans

# # Membaca data joint states
# joint_data = pd.read_csv(r'C:\Users\syari\Downloads\balance_detection\balance_detection\scripts\DONE\data_lagi\joint_states.csv')

# # Ekstrak nilai torsi dari kolom 'Joint States'
# joint_data['Joint States'] = joint_data['Joint States'].apply(ast.literal_eval)

# # Memisahkan nilai ke dalam kolom terpisah
# torque_data = pd.DataFrame(joint_data['Joint States'].tolist(), columns=[f'Torque_{i}' for i in range(len(joint_data['Joint States'][0]))])

# # Gabungkan kembali dengan data waktu
# joint_data = pd.concat([joint_data['Time'], torque_data], axis=1)

# # Konversi data torsi menjadi float
# joint_data = joint_data.astype(float)

# # Menghitung rata-rata dan deviasi standar torsi
# joint_data['mean_torque'] = joint_data.mean(axis=1)
# joint_data['std_torque'] = joint_data.std(axis=1)

# # Menyiapkan fitur untuk clustering
# X = joint_data[['mean_torque', 'std_torque']]

# # Menggunakan K-Means untuk mengelompokkan data menjadi 2 kelas
# kmeans = KMeans(n_clusters=2, n_init=10, random_state=42)
# joint_data['cluster'] = kmeans.fit_predict(X)

# # Menentukan label fase berdasarkan cluster
# joint_data['support_phase'] = np.where(joint_data['cluster'] == 0, 'single support', 'double support')

# # Visualisasi hasil clustering
# plt.figure(figsize=(10, 6))
# plt.scatter(joint_data['mean_torque'], joint_data['std_torque'], c=joint_data['cluster'], cmap='viridis', alpha=0.5)
# plt.scatter(kmeans.cluster_centers_[:, 0], kmeans.cluster_centers_[:, 1], s=300, c='red', marker='X', label='Centroids')
# plt.title('Clustering K-Means untuk Fase Support')
# plt.xlabel('Mean Torque')
# plt.ylabel('Std Torque')
# plt.legend()
# plt.grid()
# plt.show()

# # Menghitung durasi setiap fase
# duration_stats = joint_data['support_phase'].value_counts()
# print("Durasi total untuk setiap fase:")
# print(duration_stats)

# # Menghitung mean untuk setiap kelas
# mean_values = joint_data.groupby('support_phase').mean()
# print("\nMean untuk setiap kelas:")
# print(mean_values[['mean_torque', 'std_torque']])

# # Cek jumlah data di setiap cluster
# print("Jumlah data di setiap cluster:")
# print(joint_data['cluster'].value_counts())

# # Statistik deskriptif dari data torsi
# print("\nStatistik deskriptif dari mean_torque:")
# print(joint_data['mean_torque'].describe())

# print("Statistik deskriptif dari std_torque:")
# print(joint_data['std_torque'].describe())

import pandas as pd
import numpy as np

# Baca data dari CSV
joint_data = pd.read_csv(r'C:\Users\syari\Downloads\balance_detection\balance_detection\scripts\DONE\simpan_data_wt\1sensor_data_berjalan\joint_states.csv')
imu_data = pd.read_csv(r'C:\Users\syari\Downloads\balance_detection\balance_detection\scripts\DONE\simpan_data_wt\1sensor_data_berjalan\initial_imu_data.csv')

# Ekstrak data yang relevan
joint_states = joint_data['Joint States'].apply(lambda x: eval(x)).tolist()
imu_time = imu_data['Time'].tolist()
imu_accel_y = imu_data['Accel_y'].tolist()  # Mengambil akselerasi pada sumbu Y

# Pemetaan ID ke index 'name' saat ini
id_to_index_map = {
    1: 9,   # l_sho_pitch
    2: 18,  # r_sho_pitch
    3: 10,  # l_sho_roll
    4: 19,  # r_sho_roll
    5: 4,   # l_el
    6: 13,  # r_el
    7: 6,   # l_hip_roll
    8: 15,  # r_hip_roll
    9: 5,   # l_hip_pitch
    10: 14, # r_hip_pitch
    11: 8,  # l_knee
    12: 17, # r_knee
    13: 2,  # l_ank_pitch
    14: 11, # r_ank_pitch
    15: 3,  # l_ank_roll
    16: 12, # r_ank_roll
    19: 0,  # head_pan
    20: 1,  # head_tilt
}

# Buat fungsi untuk menentukan fase dukungan
def detect_support_phase(joint_states, imu_accel_y):
    # Misalkan Anda ingin fokus pada l_ank_pitch dan r_ank_pitch
    l_ank_pitch = np.array([state[id_to_index_map[13]] for state in joint_states])  # l_ank_pitch
    r_ank_pitch = np.array([state[id_to_index_map[14]] for state in joint_states])  # r_ank_pitch

    phases = []
    for l_ank, r_ank, accel in zip(l_ank_pitch, r_ank_pitch, imu_accel_y):
        # Logika untuk mendeteksi fase
        if l_ank > 0 and r_ank > 0:  # Kedua sendi positif
            phases.append('Double Support')
        elif l_ank <= 0 and r_ank <= 0:  # Kedua sendi negatif
            phases.append('Single Support')
        else:
            # Tambahkan logika untuk mendeteksi dengan menggunakan akselerasi
            if accel < -0.5:  # Threshold untuk mendeteksi ketika kaki mengangkat
                phases.append('Single Support')  # Asumsikan kita dalam fase single support
            else:
                phases.append('Double Support')  # Jika tidak ada akselerasi vertikal yang signifikan

    return phases

# Dapatkan fase dukungan
support_phases = detect_support_phase(joint_states, imu_accel_y)

# Sinkronisasi panjang data dengan menggunakan waktu dari IMU
min_length = min(len(imu_time), len(support_phases))
output_df = pd.DataFrame({
    'Time': imu_time[:min_length],  # Menggunakan waktu dari IMU
    'Support Phase': support_phases[:min_length]
})

# Simpan ke CSV
output_df.to_csv('support_phases.csv', index=False)
print("Output disimpan sebagai 'support_phases.csv'")