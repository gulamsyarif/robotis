import subprocess
from multiprocessing import Process

def run_script(script_name, output_file):
    with open(output_file, 'w') as outfile:
        subprocess.call(['python', script_name], stdout=outfile)

if __name__ == '__main__':
    # Daftar skrip dan nama file output
    scripts = [
        ('ZMP.py', 'datas_waktu/WT.csv'),
        ('ZMP_UKF.py', 'datas_waktu/UKF.csv'),
        ('ZMP_raw.py', 'datas_waktu/raw.csv'),
        ('raw.py', None)  # Tidak ada pengalihan output untuk raw.py
    ]

    # Membuat dan memulai proses untuk setiap skrip
    processes = []
    for script, output in scripts:
        if output:  # Jika ada file output, jalankan dengan pengalihan
            p = Process(target=run_script, args=(script, output))
        else:  # Jika tidak ada pengalihan output
            p = Process(target=subprocess.call, args=(['python', script],))
        processes.append(p)
        p.start()

    # Menunggu semua proses selesai
    for p in processes:
        p.join()

    print("Semua skrip telah selesai dijalankan.")
