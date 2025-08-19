import pandas as pd

# Fungsi untuk menghitung statistik
def compute_statistics(file_path):
    data = pd.read_csv(file_path)
    return {
        "File": file_path,
        "CoM Mean": data["CoM Computation Time"].mean(),
        "CoM Min": data["CoM Computation Time"].min(),
        "CoM Max": data["CoM Computation Time"].max(),
        "ZMP Mean": data["ZMP Calculation Time"].mean(),
        "ZMP Min": data["ZMP Calculation Time"].min(),
        "ZMP Max": data["ZMP Calculation Time"].max(),
        "SP Mean": data["Support Polygon Calculation Time"].mean(),
        "SP Min": data["Support Polygon Calculation Time"].min(),
        "SP Max": data["Support Polygon Calculation Time"].max(),
    }

# Definisikan file CSV secara manual
csv_files = [
    r"C:\Users\syari\OneDrive\Documents\Skripsii\balance_complete\balance_detection\scripts\modified\Pengujian4\datas_waktu\celan\raw_output.csv",  # Ganti dengan nama file pertama
    r"C:\Users\syari\OneDrive\Documents\Skripsii\balance_complete\balance_detection\scripts\modified\Pengujian4\datas_waktu\celan\WT_output.csv",  # Ganti dengan nama file kedua
    r"C:\Users\syari\OneDrive\Documents\Skripsii\balance_complete\balance_detection\scripts\modified\Pengujian4\datas_waktu\celan\UKF_output.csv"   # Ganti dengan nama file ketiga
]

# Proses setiap file dan tampilkan statistik
for file_path in csv_files:
    try:
        stats = compute_statistics(file_path)
        print(f"{stats['File']}:")
        print(f"  CoM: Mean={stats['CoM Mean']:.6f}, Min={stats['CoM Min']:.6f}, Max={stats['CoM Max']:.6f}")
        print(f"  ZMP: Mean={stats['ZMP Mean']:.6f}, Min={stats['ZMP Min']:.6f}, Max={stats['ZMP Max']:.6f}")
        print(f"  SP:  Mean={stats['SP Mean']:.6f}, Min={stats['SP Min']:.6f}, Max={stats['SP Max']:.6f}")
        print()
    except Exception as e:
        print(f"Error processing {file_path}: {e}")
