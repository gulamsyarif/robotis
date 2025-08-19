import re
import csv

# Fungsi untuk mengekstrak data dari log
def extract_data_from_log(line):
    dynamic_com_pattern = r"Calculated Dynamic CoM: x=([-.\d]+), y=([-.\d]+), z=([-.\d]+)"
    projected_com_pattern = r"Projected CoM on the ground: x=([-.\d]+), y=([-.\d]+), z=([-.\d]+)"
    moments_pattern = r"Calculated Moments: Mx=([-.\d]+), My=([-.\d]+)"
    zmp_pattern = r"Calculated Dynamic ZMP: x=([-.\d]+), y=([-.\d]+)"
    support_phase_pattern = r"Support Phase: (\w+ \w+), Area: ([-.\d]+), Perimeter: ([-.\d]+)"

    data = {}

    if "Calculated Dynamic CoM" in line:
        match = re.search(dynamic_com_pattern, line)
        if match:
            data["com_x"], data["com_y"], data["com_z"] = match.groups()

    if "Projected CoM on the ground" in line:
        match = re.search(projected_com_pattern, line)
        if match:
            data["proj_com_x"], data["proj_com_y"], data["proj_com_z"] = match.groups()

    if "Calculated Moments" in line:
        match = re.search(moments_pattern, line)
        if match:
            data["moment_x"], data["moment_y"] = match.groups()

    if "Calculated Dynamic ZMP" in line:
        match = re.search(zmp_pattern, line)
        if match:
            data["zmp_x"], data["zmp_y"] = match.groups()

    if "Support Phase" in line:
        match = re.search(support_phase_pattern, line)
        if match:
            data["support_phase"], data["area"], data["perimeter"] = match.groups()

    return data

# Fungsi untuk memproses seluruh file CSV dan menggabungkan data yang terpisah dalam beberapa baris
def process_log_csv(input_csv_path, output_csv_path):
    with open(input_csv_path, 'r') as infile, open(output_csv_path, 'w', newline='') as outfile:
        writer = csv.DictWriter(outfile, fieldnames=[
            "com_x", "com_y", "com_z",
            "proj_com_x", "proj_com_y", "proj_com_z",
            "moment_x", "moment_y",
            "zmp_x", "zmp_y",
            "support_phase", "area", "perimeter"
        ])
        writer.writeheader()

        current_data = {}

        for line in infile:
            # Mengumpulkan data dari baris log
            data = extract_data_from_log(line)
            if data:
                # Menyimpan data sementara
                current_data.update(data)

            # Jika semua data untuk satu set sudah terkumpul (berdasarkan Support Phase yang muncul terakhir)
            if "support_phase" in data:
                writer.writerow(current_data)
                # Reset data setelah menulis satu set
                current_data = {}

# Contoh pemanggilan fungsi
input_csv_path = r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\modified\Pengujian2\jongkok\raw.csv'  # Ganti dengan path CSV Anda
output_csv_path = r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\modified\Pengujian2\jongkok\raw_clean.csv'
process_log_csv(input_csv_path, output_csv_path)

print(f"Data telah diproses dan disimpan ke {output_csv_path}")
