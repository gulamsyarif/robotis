#!/usr/bin/env python 
import rospy
from sensor_msgs.msg import Imu, JointState
import numpy as np
import csv
import os

# ----------------------- KODE 1: Sensor Data Logger -----------------------

# Global variables to store sensor data
accel = None
orientation = None
gyro_raw = None
joint_positions = None
previous_com = None
previous_time = None
data_count = 0

# Placeholder for servo data (mass and CoM offsets)
servo_data = [
    {"id": 20, "mass": 0.09, "CoM_offset": (0, 2.85, 45.1)},
    {"id": 3, "mass": 0.079, "CoM_offset": (-10.8, 1.3, 37.5)},
    {"id": 4, "mass": 0.079, "CoM_offset": (10.8, 1.3, 37.5)},
    {"id": 5, "mass": 0.031, "CoM_offset": (-23.4, 0.9, 37.5)},
    {"id": 6, "mass": 0.031, "CoM_offset": (23.4, 0.9, 37.5)},
    {"id": 1, "mass": 0.086, "CoM_offset": (-4.5, 1.8, 35.2)},
    {"id": 2, "mass": 0.086, "CoM_offset": (4.5, 1.8, 35.2)},
    {"id": 19, "mass": 1.179, "CoM_offset": (0, 1.8, 35.2)},
    {"id": 9, "mass": 0.082, "CoM_offset": (-3.2, -2.7, 23.5)},
    {"id": 11, "mass": 0.164, "CoM_offset": (-3.2, 1, 23.5)},
    {"id": 10, "mass": 0.082, "CoM_offset": (3.2, -2.7, 23.5)},
    {"id": 12, "mass": 0.246, "CoM_offset": (3.2, 1, 23.5)},
    {"id": 13, "mass": 0.139, "CoM_offset": (-3.2, 0.7, 16.4)},
    {"id": 14, "mass": 0.139, "CoM_offset": (3.2, 0.7, 16.4)},
    {"id": 17, "mass": 0.082, "CoM_offset": (-3.2, -2.7, 3.7)},
    {"id": 15, "mass": 0.2, "CoM_offset": (-3.2, 0.4, 3.7)},
    {"id": 16, "mass": 0.282, "CoM_offset": (3.2, 0.4, 3.7)}
]

# File paths for CSV
data_dir = "1data_lagi"
if not os.path.exists(data_dir):
    os.makedirs(data_dir)

joint_state_file = os.path.join(data_dir, "joint_states.csv")
com_file = os.path.join(data_dir, "com.csv")
velocity_file = os.path.join(data_dir, "velocity.csv")
zmp_file = os.path.join(data_dir, "zmp.csv")
initial_imu_file = os.path.join(data_dir, "initial_imu_data.csv")
denoised_gyro_file = os.path.join(data_dir, "denoised_gyro_data.csv")

# Initialize CSV files with headers
def initialize_csv_files():
    with open(joint_state_file, 'w') as f:
        writer = csv.writer(f)
        writer.writerow(["Time", "Joint States"])

    with open(com_file, 'w') as f:
        writer = csv.writer(f)
        writer.writerow([
            "Time", 
            "CoM_raw_x", "CoM_raw_y", "CoM_raw_z",
            "CoM_wavelet_x", "CoM_wavelet_y", "CoM_wavelet_z",
            "CoM_UKF_x", "CoM_UKF_y", "CoM_UKF_z"
        ])

    with open(velocity_file, 'w') as f:
        writer = csv.writer(f)
        writer.writerow([
            "Time", 
            "Velocity_raw_x", "Velocity_raw_y", "Velocity_raw_z",
            "Velocity_wavelet_x", "Velocity_wavelet_y", "Velocity_wavelet_z",
            "Velocity_UKF_x", "Velocity_UKF_y", "Velocity_UKF_z"
        ])

    with open(zmp_file, 'w') as f:
        writer = csv.writer(f)
        writer.writerow([
            "Time", 
            "ZMP_raw_x", "ZMP_raw_y",
            "ZMP_wavelet_x", "ZMP_wavelet_y",
            "ZMP_UKF_x", "ZMP_UKF_y"
        ])

    with open(initial_imu_file, 'w') as f:
        writer = csv.writer(f)
        writer.writerow([
            "Time", 
            "Accel_x", "Accel_y", "Accel_z", 
            "Orientation_x", "Orientation_y", "Orientation_z", "Orientation_w", 
            "Gyro_x", "Gyro_y", "Gyro_z"
        ])

    with open(denoised_gyro_file, 'w') as f:
        writer = csv.writer(f)
        writer.writerow([
            "Time", 
            "Denoised_Gyro_wavelet_x", "Denoised_Gyro_wavelet_y", "Denoised_Gyro_wavelet_z",
            "Denoised_Gyro_UKF_x", "Denoised_Gyro_UKF_y", "Denoised_Gyro_UKF_z"
        ])

initialize_csv_files()

# Denoising parameters for Wavelet
wavelet_buffer_size = 10
gyro_x_wavelet_buffer, gyro_y_wavelet_buffer, gyro_z_wavelet_buffer = [], [], []
initial_imu_data = []  # List to store initial IMU data
denoised_data_wavelet = []  # List to store denoised gyro data from Wavelet
denoised_data_ukf = []  # List to store denoised gyro data from UKF

# ----------------------- KODE 2: UKF Denoising -----------------------

# Parameter UKF
alpha = 1e-3
beta = 2
kappa = 0
n = 3  # Dimensi state: gyro x, y, z
lambda_ = alpha**2 * (n + kappa) - n
gamma = np.sqrt(n + lambda_)

# Fungsi perhitungan titik sigma
def calculate_sigma_points(x, P):
    # Regularization term
    epsilon = 1e-6
    
    # Log the covariance matrix for debugging
    rospy.loginfo("Covariance matrix P before adjustment:\n{}".format(P))

    # Ensure covariance matrix is positive definite
    if np.any(np.linalg.eigvals(P) <= 0):
        P += epsilon * np.eye(P.shape[0])
        rospy.loginfo("Adjusted Covariance matrix P:\n{}".format(P))

    sigma_points = np.zeros((2 * n + 1, n))
    sigma_points[0] = x
    try:
        sqrt_P = np.linalg.cholesky((n + lambda_) * P)
    except np.linalg.LinAlgError:
        # If still fails, make P more robust
        P += epsilon * np.eye(P.shape[0])
        rospy.loginfo("Cholesky failed, adjusted Covariance matrix P:\n{}".format(P))
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
    P_pred = process_noise_cov.copy()
    for i in range(2 * n + 1):
        diff = predicted_sigma_points[i] - x_pred
        P_pred += weights_cov[i] * np.outer(diff, diff)
    return x_pred, P_pred

# Fungsi pembaruan UKF
def update_ukf(x_pred, P_pred, z, R):
    sigma_points = calculate_sigma_points(x_pred, P_pred)
    z_sigma_points = sigma_points  # Asumsikan fungsi observasi adalah identitas

    # Prediksi pengukuran
    z_pred = np.mean(z_sigma_points, axis=0)
    P_zz = R.copy()
    for i in range(2 * n + 1):
        diff = z_sigma_points[i] - z_pred
        P_zz += np.outer(diff, diff)

    # Cross covariance
    P_xz = np.zeros((n, n))
    for i in range(2 * n + 1):
        diff_x = sigma_points[i] - x_pred
        diff_z = z_sigma_points[i] - z_pred
        P_xz += (0.5 / (n + lambda_)) * np.outer(diff_x, diff_z)

    # Kalman gain
    K = np.dot(P_xz, np.linalg.inv(P_zz))

    # Update state and covariance
    x_updated = x_pred + np.dot(K, (z - z_pred))
    P_updated = P_pred - np.dot(K, P_zz).dot(K.T)

    return x_updated, P_updated

def ukf_denoise(gyro_data, process_noise, measurement_noise, dt):
    # Initialize state and covariance
    x = np.zeros(3)
    P = np.eye(3) * 0.01  # Starting covariance
    
    denoised_signal = []
    for z in gyro_data:
        # Prediction
        sigma_points = calculate_sigma_points(x, P)
        predicted_sigma_points = predict_sigma_points(sigma_points, dt)
        x_pred, P_pred = predict_mean_and_covariance(predicted_sigma_points, process_noise)

        # Ensure the predicted covariance is positive definite
        if np.any(np.linalg.eigvals(P_pred) <= 0):
            P_pred += 1e-6 * np.eye(P_pred.shape[0])  # Regularize if necessary

        # Update with gyro measurement
        x, P = update_ukf(x_pred, P_pred, z, measurement_noise)
        
        denoised_signal.append(x)
    
    return np.array(denoised_signal)

# Buffer untuk UKF
ukf_buffer_size = 10
gyro_ukf_buffer = []

# ----------------------- Fungsi Denoising Wavelet -----------------------

# Haar Wavelet Transform Functions
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

def haar_wavelet_reconstruction(coeffs):
    length = len(coeffs[-1][0]) * 2
    reconstructed = coeffs[-1][0]
    for i in range(len(coeffs)-1, -1, -1):
        approx_coeffs, detail_coeffs = coeffs[i]
        new_length = len(approx_coeffs) * 2
        temp = np.zeros(new_length)
        for j in range(len(approx_coeffs)):
            temp[2 * j] = approx_coeffs[j] + detail_coeffs[j]
            temp[2 * j + 1] = approx_coeffs[j] - detail_coeffs[j]
        reconstructed = temp
    return reconstructed

def soft_thresholding(coeffs, threshold):
    thresholded_coeffs = []
    for (approx, detail) in coeffs:
        thresholded_detail = np.sign(detail) * np.maximum(np.abs(detail) - threshold, 0)
        thresholded_coeffs.append((approx, thresholded_detail))
    return thresholded_coeffs

def dwt_denoise_custom(signal, level=2, threshold=0.5):
    coeffs = haar_wavelet_transform(signal, level=level)
    thresholded_coeffs = soft_thresholding(coeffs, threshold)
    denoised_signal = haar_wavelet_reconstruction(thresholded_coeffs)
    return denoised_signal

# ----------------------- Callback Functions -----------------------

def imu_callback(data):
    global accel, orientation, gyro_raw
    global gyro_x_wavelet_buffer, gyro_y_wavelet_buffer, gyro_z_wavelet_buffer
    global denoised_data_wavelet, denoised_data_ukf
    global gyro_ukf_buffer

    time_now = rospy.Time.now().to_sec()

    # Store IMU data
    accel = data.linear_acceleration
    orientation = data.orientation
    gyro_raw = data.angular_velocity

    # Log initial IMU data
    initial_imu_data.append([
        time_now, 
        accel.x, accel.y, accel.z, 
        orientation.x, orientation.y, orientation.z, orientation.w, 
        gyro_raw.x, gyro_raw.y, gyro_raw.z
    ])

    # Save to CSV every 100 data points
    if len(initial_imu_data) >= 100:
        with open(initial_imu_file, 'a') as f:
            writer = csv.writer(f)
            writer.writerows(initial_imu_data)
        initial_imu_data[:] = []  # Clear data after saving

    # ------------------- Denoising dengan Wavelet -------------------
    gyro_x_wavelet_buffer.append(gyro_raw.x)
    gyro_y_wavelet_buffer.append(gyro_raw.y)
    gyro_z_wavelet_buffer.append(gyro_raw.z)

    denoised_wavelet = None
    if len(gyro_x_wavelet_buffer) >= wavelet_buffer_size:
        denoised_wavelet_x = dwt_denoise_custom(np.array(gyro_x_wavelet_buffer))[-1]
        denoised_wavelet_y = dwt_denoise_custom(np.array(gyro_y_wavelet_buffer))[-1]
        denoised_wavelet_z = dwt_denoise_custom(np.array(gyro_z_wavelet_buffer))[-1]

        denoised_data_wavelet.append([
            time_now, 
            denoised_wavelet_x, denoised_wavelet_y, denoised_wavelet_z
        ])

        # Save denoised wavelet data to CSV every 100 data points
        if len(denoised_data_wavelet) >= 100:
            with open(denoised_gyro_file, 'a') as f:
                writer = csv.writer(f)
                writer.writerows(denoised_data_wavelet)
            denoised_data_wavelet[:] = []  # Clear data after saving

        denoised_wavelet = [denoised_wavelet_x, denoised_wavelet_y, denoised_wavelet_z]

        # Clear buffers
        gyro_x_wavelet_buffer.pop(0)
        gyro_y_wavelet_buffer.pop(0)
        gyro_z_wavelet_buffer.pop(0)

    # ------------------- Denoising dengan UKF -------------------
    gyro_ukf_buffer.append([gyro_raw.x, gyro_raw.y, gyro_raw.z])

    denoised_ukf = None
    if len(gyro_ukf_buffer) >= ukf_buffer_size:
        gyro_data = np.array(gyro_ukf_buffer)
        process_noise = np.eye(3) * 0.01
        measurement_noise = np.eye(3) * 0.1
        dt = 0.01

        denoised_ukf_result = ukf_denoise(gyro_data, process_noise, measurement_noise, dt)

        # Ambil data terakhir sebagai representasi denoised saat ini
        denoised_ukf = denoised_ukf_result[-1]

        denoised_data_ukf.append([
            time_now, 
            denoised_ukf[0], denoised_ukf[1], denoised_ukf[2]
        ])

        # Save denoised UKF data to CSV every 100 data points
        if len(denoised_data_ukf) >= 100:
            with open(denoised_gyro_file, 'a') as f:
                writer = csv.writer(f)
                writer.writerows(denoised_data_ukf)
            denoised_data_ukf[:] = []  # Clear data after saving

        # Log denoised UKF data
        rospy.loginfo("Denoised Gyro UKF: [%f, %f, %f]" % (denoised_ukf[0], denoised_ukf[1], denoised_ukf[2]))

        # Clear buffer
        gyro_ukf_buffer.pop(0)

    # Simpan hasil denoising ke file denoised_gyro_file
    if denoised_wavelet is not None and denoised_ukf is not None and all(denoised_wavelet) and all(denoised_ukf):
        with open(denoised_gyro_file, 'a') as f:
            writer = csv.writer(f)
            writer.writerow([
                time_now, 
                denoised_wavelet[0], denoised_wavelet[1], denoised_wavelet[2],
                denoised_ukf[0], denoised_ukf[1], denoised_ukf[2]
            ])

    # Simpan data denoised ke variabel global untuk perhitungan CoM, Velocity, ZMP
    # Simpan dalam bentuk dictionary untuk mempermudah akses
    imu_denosing_results = {
        "raw": [gyro_raw.x, gyro_raw.y, gyro_raw.z],
        "wavelet": denoised_wavelet if denoised_wavelet else [0.0, 0.0, 0.0],
        "UKF": denoised_ukf.tolist() if denoised_ukf is not None else [0.0, 0.0, 0.0]
    }

    # Update global variable for CoM calculation
    global imu_denosing_latest
    imu_denosing_latest = imu_denosing_results

def joint_state_callback(data):
    global joint_positions
    time_now = rospy.Time.now().to_sec()
    
    # Store joint states
    joint_positions = data.position
    
    # Log joint states
    with open(joint_state_file, 'a') as f:
        writer = csv.writer(f)
        writer.writerow([time_now, joint_positions])
    
    # Calculate CoM, velocity, and ZMP
    calculate_com_and_zmp(time_now)

# Variable untuk menyimpan hasil denoising terbaru
imu_denosing_latest = {
    "raw": [0.0, 0.0, 0.0],
    "wavelet": [0.0, 0.0, 0.0],
    "UKF": [0.0, 0.0, 0.0]
}

def calculate_com_and_zmp(current_time):
    global joint_positions, previous_com, previous_time, data_count, imu_denosing_latest

    # Calculate the CoM (Center of Mass) for each denoising method
    for method in ["raw", "wavelet", "UKF"]:
        gyro_data = imu_denosing_latest[method]
        # Jika diperlukan, gunakan gyro_data untuk perhitungan lebih lanjut
        # Namun dalam KODE 1, gyro_data tidak digunakan secara langsung dalam perhitungan CoM
        # Asumsi bahwa gyro_data dapat digunakan untuk integrasi posisi atau lainnya jika diperlukan

        # Untuk saat ini, kita akan fokus pada perhitungan CoM berdasarkan joint positions
        # Jika gyro_data digunakan untuk menghitung posisi atau orientasi, tambahkan logika di sini

    # Implementasi perhitungan CoM berdasarkan joint_positions dan metode denoising
    if joint_positions is not None:
        total_mass = sum(servo["mass"] for servo in servo_data)
        com = {
            "raw": [0.0, 0.0, 0.0],
            "wavelet": [0.0, 0.0, 0.0],
            "UKF": [0.0, 0.0, 0.0]
        }

        for i, servo in enumerate(servo_data):
            angle = joint_positions[i]  # Asumsikan joint_positions berisi sudut joint
            offset = np.array(servo["CoM_offset"])
            # Gunakan fungsi tertentu untuk menghitung posisi berdasarkan sudut dan offset
            position = np.array([offset[0], offset[1], offset[2]])  # Ganti dengan perhitungan posisi aktual jika diperlukan
            for method in ["raw", "wavelet", "UKF"]:
                com[method][0] += position[0] * servo["mass"]
                com[method][1] += position[1] * servo["mass"]
                com[method][2] += position[2] * servo["mass"]

        for method in ["raw", "wavelet", "UKF"]:
            com[method][0] /= total_mass
            com[method][1] /= total_mass
            com[method][2] /= total_mass

        # Calculate velocity
        if previous_time is None:
            previous_time = current_time  # Initialize previous_time on first run

        # Calculate delta time
        dt = current_time - previous_time
        previous_time = current_time

        # Initialize the velocity dictionary
        velocity = {
            "raw": [0.0, 0.0, 0.0],
            "wavelet": [0.0, 0.0, 0.0],
            "UKF": [0.0, 0.0, 0.0]
        }

        # Check if dt is too small to avoid large velocity values due to numerical instability
        if dt > 1e-6 and previous_com is not None:  # Check that dt is reasonable
            for method in ["raw", "wavelet", "UKF"]:
                # Calculate velocity as change in position over change in time
                velocity[method][0] = (com[method][0] - previous_com[method][0]) / dt
                velocity[method][1] = (com[method][1] - previous_com[method][1]) / dt
                velocity[method][2] = (com[method][2] - previous_com[method][2]) / dt
        else:
            # If first run or dt is too small, set velocity to zero
            for method in ["raw", "wavelet", "UKF"]:
                velocity[method] = [0.0, 0.0, 0.0]

        # Update previous_com to current com
        previous_com = com.copy()


        # Calculate ZMP (Zero Moment Point) - Placeholder
        zmp = {
            "raw": [com["raw"][0], com["raw"][1]],
            "wavelet": [com["wavelet"][0], com["wavelet"][1]],
            "UKF": [com["UKF"][0], com["UKF"][1]]
        }

        # Save CoM, Velocity, and ZMP data
        with open(com_file, 'a') as f:
            writer = csv.writer(f)
            writer.writerow([
                current_time,
                com["raw"][0], com["raw"][1], com["raw"][2],
                com["wavelet"][0], com["wavelet"][1], com["wavelet"][2],
                com["UKF"][0], com["UKF"][1], com["UKF"][2]
            ])

        with open(velocity_file, 'a') as f:
            writer = csv.writer(f)
            writer.writerow([
                current_time,
                velocity["raw"][0], velocity["raw"][1], velocity["raw"][2],
                velocity["wavelet"][0], velocity["wavelet"][1], velocity["wavelet"][2],
                velocity["UKF"][0], velocity["UKF"][1], velocity["UKF"][2]
            ])

        with open(zmp_file, 'a') as f:
            writer = csv.writer(f)
            writer.writerow([
                current_time,
                zmp["raw"][0], zmp["raw"][1],
                zmp["wavelet"][0], zmp["wavelet"][1],
                zmp["UKF"][0], zmp["UKF"][1]
            ])

        data_count += 1  # Increment data count

def main():
    rospy.init_node('sensor_data_logger_combined', anonymous=True)

    rospy.Subscriber("/robotis/open_cr/imu", Imu, imu_callback)
    rospy.Subscriber("/robotis/present_joint_states", JointState, joint_state_callback)

    rospy.spin()

if __name__ == '__main__':
    main()

