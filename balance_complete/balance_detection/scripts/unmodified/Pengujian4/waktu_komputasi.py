import psutil
import csv
import time

# Daftar untuk menyimpan data
data = []

# Mengumpulkan data sebanyak 50 kali
for _ in range(50):
    # Mengambil informasi CPU dan Memori
    cpu_usage = psutil.cpu_percent(interval=1)  # Mengambil penggunaan CPU dalam persen dengan interval 1 detik
    memory_info = psutil.virtual_memory()  # Mengambil informasi memori

    # Menghitung penggunaan memori
    memory_usage = memory_info.percent  # Persentase penggunaan memori
    available_memory_gb = memory_info.available / (1024 ** 3)  # Mengonversi dari byte ke GB
    total_memory_gb = memory_info.total / (1024 ** 3)          # Total memori dalam GB
    used_memory_gb = memory_info.used / (1024 ** 3)            # Memori yang digunakan dalam GB

    # Menyusun data untuk setiap iterasi
    row = [
        total_memory_gb,
        used_memory_gb,
        available_memory_gb,
        memory_usage,
        cpu_usage,
    ]
    data.append(row)

# Menyimpan data ke file CSV
with open('/home/robotis/catkin_ws/src/balance_detection/scripts/Pengujian4/datas/idle.csv', 'w') as csvfile:
    writer = csv.writer(csvfile)
    # Menulis header
    writer.writerow(["Total Memory (GB)", "Used Memory (GB)", "Available Memory (GB)", "Memory Usage Percentage (%)", "CPU Usage Percentage (%)"])
    # Menulis data
    writer.writerows(data)

print ("Informasi penggunaan sistem telah disimpan ke 'system_usage_info.csv'.")
