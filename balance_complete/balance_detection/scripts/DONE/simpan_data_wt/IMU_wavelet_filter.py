#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
import numpy as np
import csv
import os

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

buffer_size = 10
gyro_x_buffer, gyro_y_buffer, gyro_z_buffer = [], [], []
denoised_gyro_data = None  # Variabel global untuk menyimpan hasil denoised
initial_imu_data = []  # List to store initial IMU data
denoised_data = []  # List to store denoised gyro data
data_counter = 0  # Counter for data points

def imu_callback(msg):
    global gyro_x_buffer, gyro_y_buffer, gyro_z_buffer, denoised_gyro_data, initial_imu_data, denoised_data, data_counter

    gyro_x = msg.angular_velocity.x
    gyro_y = msg.angular_velocity.y
    gyro_z = msg.angular_velocity.z

    # Simpan IMU awal
    if data_counter == 0:
        initial_imu_data.append([gyro_x, gyro_y, gyro_z])

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
            denoised_data.append(denoised_gyro_data.tolist())
            rospy.loginfo("Denoised Gyro: [%f, %f, %f]" % (denoised_gyro_data[0], denoised_gyro_data[1], denoised_gyro_data[2]))

        # Bersihkan buffer setelah diproses
        gyro_x_buffer, gyro_y_buffer, gyro_z_buffer = [], [], []
        data_counter += 1  # Increment the counter

    # Stop after collecting 3000 data points
    if data_counter >= 3000:
        save_to_csv(initial_imu_data, "initial_imu_data.csv")
        save_to_csv(denoised_data, "denoised_gyro_data.csv")
        rospy.signal_shutdown("Collected 3000 data points. Shutting down.")

def save_to_csv(data, filename):
    """Saves data to a CSV file."""
    with open(filename, mode='wb') as file:
        writer = csv.writer(file)
        writer.writerows(data)

def get_denoised_gyro(gyro_data):
    return denoised_gyro_data

# Node utama
def main():
    rospy.init_node('imu_denoise_node', anonymous=True)
    rospy.Subscriber("robotis/open_cr/imu", Imu, imu_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
