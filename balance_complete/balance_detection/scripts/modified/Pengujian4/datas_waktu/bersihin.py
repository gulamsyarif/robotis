import csv

# Nama file input dan output
input_file = r"C:\Users\syari\OneDrive\Documents\Skripsii\balance_complete\balance_detection\scripts\modified\Pengujian4\datas_waktu\WT.csv"
output_file = r"C:\Users\syari\OneDrive\Documents\Skripsii\balance_complete\balance_detection\scripts\modified\Pengujian4\datas_waktu\WT_output.csv"

# Inisialisasi list untuk menyimpan data waktu
times_data = []

# Membaca file CSV asli
with open(input_file, mode='r') as file:
    reader = csv.reader(file)
    for row in reader:
        if "[INFO]" in row[0]:
            if "CoM Computation Time:" in row[0]:
                com_time = float(row[0].split("CoM Computation Time:")[1].split("seconds")[0].strip())
            elif "ZMP Calculation Time:" in row[0]:
                zmp_time = float(row[0].split("ZMP Calculation Time:")[1].split("seconds")[0].strip())
            elif "Support Polygon Calculation Time:" in row[0]:
                support_time = float(row[0].split("Support Polygon Calculation Time:")[1].split("seconds")[0].strip())
                # Tambahkan data ke list setelah mendapatkan semua waktu
                times_data.append([com_time, zmp_time, support_time])

# Menulis data waktu ke file CSV baru
with open(output_file, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["CoM Computation Time", "ZMP Calculation Time", "Support Polygon Calculation Time"])
    writer.writerows(times_data)

print(f"File baru telah dibuat: {output_file}")

