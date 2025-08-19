import rospy
from sensor_msgs.msg import Imu
import numpy as np
import matplotlib.pyplot as plt

# Fungsi Haar Wavelet Transform
def haar_wavelet_transform(data, level=2):
    output = np.copy(data)
    results = []
    length = len(data)

    for _ in range(level):
        detail_coeffs = np.zeros(length // 2)
        approx_coeffs = np.zeros(length // 2)

        for i in range(length // 2):
            approx_coeffs[i] = (output[2 * i] + output[2 * i + 1]) / 2
            detail_coeffs[i] = (output[2 * i] - output[2 * i + 1]) / 2

        results.append((approx_coeffs, detail_coeffs))
        output = approx_coeffs
        length //= 2
    return results

# Fungsi Rekonstruksi Sinyal
def haar_wavelet_reconstruction(coeffs):
    length = len(coeffs[-1][0]) * 2
    for i in range(len(coeffs)-1, -1, -1):
        approx_coeffs, detail_coeffs = coeffs[i]
        new_length = len(approx_coeffs) * 2
        reconstructed = np.zeros(new_length)

        for j in range(len(approx_coeffs)):
            reconstructed[2 * j] = approx_coeffs[j] + detail_coeffs[j]
            reconstructed[2 * j + 1] = approx_coeffs[j] - detail_coeffs[j]

        coeffs[i] = (reconstructed, detail_coeffs)
    
    return reconstructed

# Fungsi Soft Thresholding
def soft_thresholding(coeffs, threshold):
    thresholded_coeffs = []
    for (approx, detail) in coeffs:
        thresholded_detail = np.sign(detail) * np.maximum(np.abs(detail) - threshold, 0)
        thresholded_coeffs.append((approx, thresholded_detail))
    return thresholded_coeffs

# Fungsi Denoising Utama
def dwt_denoise_custom(signal, level=2, threshold=0.5):
    coeffs = haar_wavelet_transform(signal, level=level)
    thresholded_coeffs = soft_thresholding(coeffs, threshold)
    denoised_signal = haar_wavelet_reconstruction(thresholded_coeffs)
    return denoised_signal

# Fungsi untuk Mengolah Data IMU dari ROS
gyro_data_x = []
gyro_data_y = []
gyro_data_z = []

def imu_callback(data):
    global gyro_data_x, gyro_data_y, gyro_data_z
    # Simpan data gyroscope dari topik
    gyro_data_x.append(data.angular_velocity.x)
    gyro_data_y.append(data.angular_velocity.y)
    gyro_data_z.append(data.angular_velocity.z)

    # Jika data cukup untuk diproses (misalnya 3000 sampel)
    if len(gyro_data_x) >= 3000:
        # Konversi data menjadi array numpy untuk pemrosesan
        gyro_x = np.array(gyro_data_x)
        gyro_y = np.array(gyro_data_y)
        gyro_z = np.array(gyro_data_z)

        # Lakukan denoising
        denoised_x = dwt_denoise_custom(gyro_x)
        denoised_y = dwt_denoise_custom(gyro_y)
        denoised_z = dwt_denoise_custom(gyro_z)

        # Plot hasilnya
        plt.figure(figsize=(15, 10))
        plt.subplot(3, 1, 1)
        plt.plot(gyro_x, label="Raw x Gyro", color="blue")
        plt.plot(denoised_x, label="Denoised x Gyro", color="orange")
        plt.title("Gyroscope X-Axis")
        plt.xlabel("Sample")
        plt.ylabel("x Gyro")
        plt.legend()

        plt.subplot(3, 1, 2)
        plt.plot(gyro_y, label="Raw y Gyro", color="blue")
        plt.plot(denoised_y, label="Denoised y Gyro", color="orange")
        plt.title("Gyroscope Y-Axis")
        plt.xlabel("Sample")
        plt.ylabel("y Gyro")
        plt.legend()

        plt.subplot(3, 1, 3)
        plt.plot(gyro_z, label="Raw z Gyro", color="blue")
        plt.plot(denoised_z, label="Denoised z Gyro", color="orange")
        plt.title("Gyroscope Z-Axis")
        plt.xlabel("Sample")
        plt.ylabel("z Gyro")
        plt.legend()

        plt.tight_layout()
        plt.show()

        # Reset data untuk batch berikutnya
        gyro_data_x, gyro_data_y, gyro_data_z = [], [], []

def main():
    rospy.init_node('imu_data_processor', anonymous=True)
    rospy.Subscriber('/robotis/open_cr/imu', Imu, imu_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
