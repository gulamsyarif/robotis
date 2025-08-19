import numpy as np
import pandas as pd

# Parameter UKF
alpha = 0.2
beta = 3
kappa = 0
n = 3  # State dimension: gyro x, y, z
lambda_ = alpha**2 * (n + kappa) - n
gamma = np.sqrt(n + lambda_)

# Sigma point calculation function
def calculate_sigma_points(x, P):
    sigma_points = np.zeros((2 * n + 1, n))
    sigma_points[0] = x
    sqrt_P = np.linalg.cholesky((n + lambda_) * P)
    for i in range(n):
        sigma_points[i + 1] = x + sqrt_P[i]
        sigma_points[i + 1 + n] = x - sqrt_P[i]
    return sigma_points

# Noise covariances
process_noise = np.eye(3) * 0.2
measurement_noise = np.eye(3) * 0.2

# Predict sigma points
def predict_sigma_points(sigma_points, dt):
    predicted_sigma_points = np.zeros_like(sigma_points)
    for i, sp in enumerate(sigma_points):
        predicted_sigma_points[i] = sp
    return predicted_sigma_points

# Increased regularization factor to stabilize the covariance
regularization_factor = 1e-8

# Predict mean and covariance function with regularization and clipping
def predict_mean_and_covariance(predicted_sigma_points, process_noise_cov):
    weights_mean = np.full(2 * n + 1, 0.5 / (n + lambda_))
    weights_mean[0] = lambda_ / (n + lambda_)
    weights_cov = np.copy(weights_mean)
    weights_cov[0] += 1 - alpha**2 + beta

    # Ensure weights are normalized
    weights_mean /= np.sum(weights_mean)
    weights_cov /= np.sum(weights_cov)

    x_pred = np.sum(weights_mean[:, None] * predicted_sigma_points, axis=0)
    P_pred = process_noise_cov.copy()
    for i in range(2 * n + 1):
        diff = predicted_sigma_points[i] - x_pred
        P_pred += weights_cov[i] * np.outer(diff, diff)

    # Apply regularization and clipping to prevent overflow
    P_pred += np.eye(n) * regularization_factor
    P_pred = np.clip(P_pred, -1e3, 1e3)  # Clip values to prevent overflow

    return x_pred, P_pred

# Update function for UKF
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

# UKF Denoising filter
def ukf_denoise(gyro_data, process_noise, measurement_noise, dt):
    x = np.zeros(3)
    P = np.eye(3)

    denoised_signal = []
    for z in gyro_data:
        sigma_points = calculate_sigma_points(x, P)
        predicted_sigma_points = predict_sigma_points(sigma_points, dt)
        x_pred, P_pred = predict_mean_and_covariance(predicted_sigma_points, process_noise)

        x, P = update(x_pred, P_pred, z, measurement_noise)
        denoised_signal.append(x)

    return np.array(denoised_signal)

# Load IMU data from CSV
imu_data = pd.read_csv(r'C:\Users\syari\OneDrive\Documents\Skripsii\balance_complete\balance_detection\scripts\UKF_dibeneri\PengujianFilter\raw_imu.csv')
gyro_data = imu_data[['Gyro_x', 'Gyro_y', 'Gyro_z']].values
dt = 0.01

# Apply the UKF denoising filter
denoised_gyro = ukf_denoise(gyro_data, process_noise, measurement_noise, dt)

# Save the results to UKF_filtered.csv
denoised_df = pd.DataFrame(denoised_gyro, columns=['Denoised_Gyro_x', 'Denoised_Gyro_y', 'Denoised_Gyro_z'])
denoised_df.to_csv(r'C:\Users\syari\OneDrive\Documents\Skripsii\balance_complete\balance_detection\scripts\UKF_dibeneri\PengujianFilter\UKF\besar.csv', index=False)
print("Filtered data saved to UKF_filtered.csv")
