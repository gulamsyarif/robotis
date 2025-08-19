import csv
import subprocess
import re

# Fungsi untuk mengeksekusi skrip dan menangkap output
def run_script_and_capture_output(script_name):
    # Eksekusi skrip dan dapatkan output
    process = subprocess.Popen(['python', script_name], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    output, _ = process.communicate()
    return output.decode('utf-8')

# Fungsi untuk memproses log dan menyimpannya ke CSV
def save_logs_to_csv(logs, csv_file):
    # Regex untuk mengekstrak informasi dari log
    log_pattern = re.compile(r'\[INFO\] \[\d+\.\d+\]: (.+)')
    
    with open(csv_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Timestamp', 'Log Info'])  # Menulis header
        
        for line in logs.splitlines():
            match = log_pattern.match(line)
            if match:
                timestamp = line.split(']')[0][1:]  # Mendapatkan timestamp
                log_info = match.group(1)  # Mendapatkan log info
                writer.writerow([timestamp, log_info])  # Menulis ke CSV

# Nama skrip yang akan dijalankan
script_name = 'ZMP.py'
# Nama file CSV yang akan disimpan
csv_file = 'log_output.csv'

# Menjalankan skrip dan menangkap output
logs = run_script_and_capture_output(script_name)
# Menyimpan logs ke file CSV
save_logs_to_csv(logs, csv_file)

print(f'Logs telah disimpan ke {csv_file}')
