#!/usr/bin/env python
import subprocess
import rospy

def run_script(script_path):
    """Menjalankan skrip Python sebagai proses terpisah dan menangkap outputnya."""
    process = subprocess.Popen(['python', script_path], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    return process

def main():
    rospy.init_node('imu_controller_node', anonymous=True)

    # Path ke skrip yang ingin dijalankan
    scripts = [
        '/home/robotis/catkin_ws/src/balance_detection/scripts/COPY/IMU_raw.py',
        '/home/robotis/catkin_ws/src/balance_detection/scripts/COPY/IMU_UKF.py',
        '/home/robotis/catkin_ws/src/balance_detection/scripts/COPY/IMU_wavelet_filter.py',
    ]

    # Menjalankan setiap skrip dalam proses terpisah
    processes = []
    for script in scripts:
        rospy.loginfo("Menjalankan skrip: %s" % script)
        process = run_script(script)
        processes.append(process)

    try:
        # Menjaga node tetap hidup sambil menjalankan proses lain
        rospy.spin()
    except KeyboardInterrupt:
        # Menghentikan semua proses jika ada interupsi
        for process in processes:
            process.terminate()
        print("Semua skrip telah dihentikan.")

    # Memeriksa output dari proses
    for process in processes:
        stdout, stderr = process.communicate()
        if stderr:
            print("Error:", stderr)
        else:
            print("Output:", stdout)

if __name__ == "__main__":
    main()
