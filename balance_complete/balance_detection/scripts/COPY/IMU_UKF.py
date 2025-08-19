#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
import numpy as np

# Parameter UKF
alpha = 1e-3
beta = 2
kappa = 0
n = 3  # Dimensi state: gyro x, y, z
lambda_ = alpha**2 * (n + kappa) - n
gamma = np.sqrt(n + lambda_)

# Fungsi perhitungan titik sigma
def calculate_sigma_points(x, P):
    sigma_points = np.zeros((2 * n + 1, n))
    sigma_points[0] = x
    sqrt_P = np.linalg.cholesky((n + lambda_) * P)
    for i in range(n):
        sigma_points[i + 1] = x + sqrt_P[i]
        sigma_points[i + 1 + n] = x - sqrt_P[i]
    return sigma_points

# Fungsi prediksi state
def predict_sigma_points(sigma_points, dt):
    predicted_sigma_points = np.zeros_like(sigma_points)
    for i, sp in enumerate(sigma_points):
        predicted_sigma_points[i] = sp  # Identitas untuk simplifikasi (gyro tetap linear)
    return predicted_sigma_points

# Fungsi prediksi rata-rata dan kovariansi
def predict_mean_and_covariance(predicted_sigma_points, process_noise_cov):
    weights_mean = np.full(2 * n + 1, 0.5 / (n + lambda_))
    weights_mean[0] = lambda_ / (n + lambda_)
    weights_cov = np.copy(weights_mean)
    weights_cov[0] += 1 - alpha**2 + beta

    x_pred = np.sum(weights_mean[:, None] * predicted_sigma_points, axis=0)
    P_pred = process_noise_cov
    for i in range(2 * n + 1):
        diff = predicted_sigma_points[i] - x_pred
        P_pred += weights_cov[i] * np.outer(diff, diff)
    return x_pred, P_pred

# Fungsi pembaruan UKF
def update(x_pred, P_pred, z, R):
    sigma_points = calculate_sigma_points(x_pred, P_pred)
    z_sigma_points = sigma_points

    z_pred = np.mean(z_sigma_points, axis=0)
    P_zz = R + np.sum([(sp - z_pred)[:, None] * (sp - z_pred)[None, :] for sp in z_sigma_points], axis=0)
    P_xz = np.sum([(sigma_points[i] - x_pred)[:, None] * (z_sigma_points[i] - z_pred)[None, :] for i in range(2 * n + 1)], axis=0)
    
    K = np.dot(P_xz, np.linalg.inv(P_zz))
    x_updated = x_pred + np.dot(K, (z - z_pred))
    P_updated = P_pred - np.dot(K, P_zz).dot(K.T)
    
    return x_updated, P_updated

# Filter UKF Denoising
def ukf_denoise(gyro_data, process_noise, measurement_noise, dt):
    # Inisialisasi state dan kovariansi
    x = np.zeros(3)
    P = np.eye(3)

    denoised_signal = []
    for z in gyro_data:
        # Prediksi
        sigma_points = calculate_sigma_points(x, P)
        predicted_sigma_points = predict_sigma_points(sigma_points, dt)
        x_pred, P_pred = predict_mean_and_covariance(predicted_sigma_points, process_noise)
        
        # Pembaruan dengan pengukuran gyro
        x, P = update(x_pred, P_pred, z, measurement_noise)
        
        denoised_signal.append(x)
    
    return np.array(denoised_signal)

buffer_size = 10
gyro_x_buffer, gyro_y_buffer, gyro_z_buffer = [], [], []

def imu_callback(msg):
    global gyro_x_buffer, gyro_y_buffer, gyro_z_buffer

    gyro_x = msg.angular_velocity.x
    gyro_y = msg.angular_velocity.y
    gyro_z = msg.angular_velocity.z

    gyro_x_buffer.append(gyro_x)
    gyro_y_buffer.append(gyro_y)
    gyro_z_buffer.append(gyro_z)

    if len(gyro_x_buffer) >= buffer_size:
        gyro_data = np.array([gyro_x_buffer, gyro_y_buffer, gyro_z_buffer]).T
        process_noise = np.eye(3) * 0.01
        measurement_noise = np.eye(3) * 0.1
        dt = 0.01

        denoised_gyro = ukf_denoise(gyro_data, process_noise, measurement_noise, dt)
        
        if len(denoised_gyro) > 0:
            rospy.loginfo("Denoised Gyro: [%f, %f, %f]" % (denoised_gyro[0, 0], denoised_gyro[0, 1], denoised_gyro[0, 2]))

        gyro_x_buffer, gyro_y_buffer, gyro_z_buffer = [], [], []

def main():
    rospy.init_node('imu_ukf_denoise_node', anonymous=True)
    rospy.Subscriber("robotis/open_cr/imu", Imu, imu_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
