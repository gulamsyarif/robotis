import numpy as np
import pandas as pd

# Define Haar Wavelet filters with normalization factor
h = np.array([1/np.sqrt(2), 1/np.sqrt(2)])  # Low-pass filter
g = np.array([1/np.sqrt(2), -1/np.sqrt(2)])  # High-pass filter

# Haar Wavelet Transform function (with normalization factors)
def haar_wavelet_transform(data, level=2):
    results = []
    output = np.copy(data)
    length = len(data)
    
    for _ in range(level):
        approx_coeffs = np.zeros(length // 2)
        detail_coeffs = np.zeros(length // 2)
        
        for k in range(length // 2):
            # Calculate c_k (approximation coefficients) with low-pass filter h
            approx_coeffs[k] = h[0] * output[2 * k] + h[1] * output[2 * k + 1]
            # Calculate d_k (detail coefficients) with high-pass filter g
            detail_coeffs[k] = g[0] * output[2 * k] + g[1] * output[2 * k + 1]
        
        print(f"Level {i+1}: Approximation Coefficients:", approx_coeffs)
        print(f"Level {i+1}: Detail Coefficients:", detail_coeffs)

        results.append((approx_coeffs, detail_coeffs))
        output = approx_coeffs
        length //= 2
    
    return results

# Reconstruction function
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

# Soft thresholding function
def soft_thresholding(coeffs, threshold):
    thresholded_coeffs = []
    for (approx, detail) in coeffs:
        print(f"Original Detail Coefficients: {detail}")
        thresholded_detail = np.sign(detail) * np.maximum(np.abs(detail) - threshold, 0)
        print(f"Thresholded Detail Coefficients: {thresholded_detail}")
        thresholded_coeffs.append((approx, thresholded_detail))
    return thresholded_coeffs

# DWT denoising function
def dwt_denoise_custom(signal, level=2, threshold=1e-3):
    coeffs = haar_wavelet_transform(signal, level=level)
    thresholded_coeffs = soft_thresholding(coeffs, threshold)
    denoised_signal = haar_wavelet_reconstruction(thresholded_coeffs)
    if len(data) < 2**level:
        raise ValueError("Data length is too short for the specified level of transform")
    return denoised_signal

# Load data from CSV file
data = pd.read_csv(r'C:\Users\syari\OneDrive\Documents\Skripsii\balance_complete\balance_detection\scripts\UKF_dibeneri\PengujianFilter\raw_imu.csv')
gyro_x_data = data['Gyro_x'].values
gyro_y_data = data['Gyro_y'].values
gyro_z_data = data['Gyro_z'].values

# Buffer size for denoising process
buffer_size = 64
denoised_gyro_x, denoised_gyro_y, denoised_gyro_z = [], [], []

# Process data in buffer with DWT denoising
for i in range(0, len(gyro_x_data), buffer_size):
    gyro_x_buffer = gyro_x_data[i:i + buffer_size]
    gyro_y_buffer = gyro_y_data[i:i + buffer_size]
    gyro_z_buffer = gyro_z_data[i:i + buffer_size]

    # Check if buffer is full
    if len(gyro_x_buffer) == buffer_size:
        denoised_x = dwt_denoise_custom(np.array(gyro_x_buffer))
        denoised_y = dwt_denoise_custom(np.array(gyro_y_buffer))
        denoised_z = dwt_denoise_custom(np.array(gyro_z_buffer))

        denoised_gyro_x.extend(denoised_x)
        denoised_gyro_y.extend(denoised_y)
        denoised_gyro_z.extend(denoised_z)

# Save denoised results to a new CSV file
denoised_df = pd.DataFrame({
    'Denoised_Gyro_x': denoised_gyro_x,
    'Denoised_Gyro_y': denoised_gyro_y,
    'Denoised_Gyro_z': denoised_gyro_z
})

denoised_df.to_csv(r'C:\Users\syari\OneDrive\Documents\Skripsii\balance_complete\balance_detection\scripts\UKF_dibeneri\PengujianFilter\DWT\c.csv', index=False)
