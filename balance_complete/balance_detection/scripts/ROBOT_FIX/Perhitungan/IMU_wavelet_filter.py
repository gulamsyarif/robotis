#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
import numpy as np

# Mendefinisikan filter Haar Wavelet dengan faktor normalisasi
h = np.array([1/np.sqrt(2), 1/np.sqrt(2)])  # Low-pass filter
g = np.array([1/np.sqrt(2), -1/np.sqrt(2)])  # High-pass filter

# Fungsi Haar Wavelet Transform (dengan faktor normalisasi)
def haar_wavelet_transform(data, level=2):
    results = []
    output = np.copy(data)
    length = len(data)
    
    for _ in range(level):
        approx_coeffs = np.zeros(length // 2)
        detail_coeffs = np.zeros(length // 2)
        
        for k in range(length // 2):
            # Menghitung koefisien aproksimasi dengan low-pass filter h
            approx_coeffs[k] = h[0] * output[2 * k] + h[1] * output[2 * k + 1]
            # Menghitung koefisien detail dengan high-pass filter g
            detail_coeffs[k] = g[0] * output[2 * k] + g[1] * output[2 * k + 1]
        
        results.append((approx_coeffs, detail_coeffs))
        output = approx_coeffs
        length //= 2
    
    return results

# Fungsi rekonstruksi
def haar_wavelet_reconstruction(coeffs):
    length = len(coeffs[-1][0]) * 2
    for i in range(len(coeffs)-1, -1, -1):
        approx_coeffs, detail_coeffs = coeffs[i]
        new_length = len(approx_coeffs) * 2
        reconstructed = np.zeros(new_length)
        
        for k in range(len(approx_coeffs)):
            reconstructed[2 * k] = h[0] * approx_coeffs[k] + g[0] * detail_coeffs[k]
            reconstructed[2 * k + 1] = h[1] * approx_coeffs[k] + g[1] * detail_coeffs[k]
        
        coeffs[i] = (reconstructed, detail_coeffs)
    return reconstructed

# Fungsi soft thresholding 
def soft_thresholding(coeffs, threshold):
    thresholded_coeffs = []
    for (approx, detail) in coeffs:
        thresholded_detail = np.sign(detail) * np.maximum(np.abs(detail) - threshold, 0)
        thresholded_coeffs.append((approx, thresholded_detail))
    return thresholded_coeffs

# Fungsi Denoise DWT 
def dwt_denoise_custom(signal, level=2, threshold=0.5):
    coeffs = haar_wavelet_transform(signal, level=level)
    thresholded_coeffs = soft_thresholding(coeffs, threshold)
    denoised_signal = haar_wavelet_reconstruction(thresholded_coeffs)
    return denoised_signal

buffer_size = 10
gyro_x_buffer, gyro_y_buffer, gyro_z_buffer = [], [], []
denoised_gyro_data = None  # Variabel global untuk menyimpan hasil denoised

def imu_callback(msg):
    global gyro_x_buffer, gyro_y_buffer, gyro_z_buffer, denoised_gyro_data

    gyro_x = msg.angular_velocity.x
    gyro_y = msg.angular_velocity.y
    gyro_z = msg.angular_velocity.z

    gyro_x_buffer.append(gyro_x)
    gyro_y_buffer.append(gyro_y)
    gyro_z_buffer.append(gyro_z)

    # Proses jika buffer penuh
    if len(gyro_x_buffer) >= buffer_size:
        denoised_x = dwt_denoise_custom(np.array(gyro_x_buffer))
        denoised_y = dwt_denoise_custom(np.array(gyro_y_buffer))
        denoised_z = dwt_denoise_custom(np.array(gyro_z_buffer))

        if len(denoised_x) > 0:
            denoised_gyro_data = np.array([denoised_x[0], denoised_y[0], denoised_z[0]])
            rospy.loginfo("Denoised Gyro: [%f, %f, %f]" % (denoised_gyro_data[0], denoised_gyro_data[1], denoised_gyro_data[2]))

        # Bersihkan buffer setelah diproses
        gyro_x_buffer, gyro_y_buffer, gyro_z_buffer = [], [], []

def get_denoised_gyro(gyro_data):
    return denoised_gyro_data

# Node utama
def main():
    rospy.init_node('imu_denoise_node', anonymous=True)
    rospy.Subscriber("robotis/open_cr/imu", Imu, imu_callback)
    rospy.spin()

if __name__ == "__main__":
    main()

