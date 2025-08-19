import numpy as np
import pandas as pd

# Define Haar Wavelet filters with normalization factor
def define_wavelet(wavelet_type='haar'):
    if wavelet_type == 'haar':
        h = np.array([1/np.sqrt(2), 1/np.sqrt(2)])  # Low-pass filter
        g = np.array([1/np.sqrt(2), -1/np.sqrt(2)])  # High-pass filter
    else:
        raise ValueError("Currently only Haar wavelet is supported.")
    return h, g

# Haar Wavelet Transform function
def haar_wavelet_transform(data, h, g, level=2):
    results = []
    output = np.copy(data)
    length = len(data)
    
    for _ in range(level):
        approx_coeffs = np.zeros(length // 2)
        detail_coeffs = np.zeros(length // 2)
        
        for k in range(length // 2):
            approx_coeffs[k] = h[0] * output[2 * k] + h[1] * output[2 * k + 1]
            detail_coeffs[k] = g[0] * output[2 * k] + g[1] * output[2 * k + 1]
        
        results.append((approx_coeffs, detail_coeffs))
        output = approx_coeffs
        length //= 2
    
    return results

# Reconstruction function
def haar_wavelet_reconstruction(coeffs, h, g):
    for i in range(len(coeffs) - 1, -1, -1):
        approx_coeffs, detail_coeffs = coeffs[i]
        length = len(approx_coeffs) * 2
        reconstructed = np.zeros(length)
        
        for k in range(len(approx_coeffs)):
            reconstructed[2 * k] = h[0] * approx_coeffs[k] + g[0] * detail_coeffs[k]
            reconstructed[2 * k + 1] = h[1] * approx_coeffs[k] + g[1] * detail_coeffs[k]
        
        coeffs[i] = (reconstructed, detail_coeffs)
    return coeffs[0][0]

# Soft thresholding function
def soft_thresholding(coeffs, threshold):
    thresholded_coeffs = []
    for approx, detail in coeffs:
        thresholded_detail = np.sign(detail) * np.maximum(np.abs(detail) - threshold, 0)
        thresholded_coeffs.append((approx, thresholded_detail))
    return thresholded_coeffs

# DWT denoising function
def dwt_denoise_custom(signal, h, g, level=2, threshold=0.5):
    coeffs = haar_wavelet_transform(signal, h, g, level=level)
    thresholded_coeffs = soft_thresholding(coeffs, threshold)
    denoised_signal = haar_wavelet_reconstruction(thresholded_coeffs, h, g)
    return denoised_signal

# Load data from CSV file
data = pd.read_csv(r'C:\Users\syari\OneDrive\Documents\Skripsii\balance_complete\balance_detection\scripts\UKF_dibeneri\PengujianFilter\raw_imu.csv')
gyro_x_data = data['Gyro_x'].values
gyro_y_data = data['Gyro_y'].values
gyro_z_data = data['Gyro_z'].values

# Buffer size for denoising process
buffer_size = 64

# Define wavelet filters
wavelet_type = 'haar'
h, g = define_wavelet(wavelet_type)

# Experiment with different levels and thresholds
levels = [1, 2, 3, 5]  # Test levels
thresholds = [1e-4, 5e-4, 8e-4, 5e-3]  # Test thresholds

results = []

for level in levels:
    for threshold in thresholds:
        denoised_gyro_x, denoised_gyro_y, denoised_gyro_z = [], [], []
        
        # Process data in buffer
        for i in range(0, len(gyro_x_data), buffer_size):
            gyro_x_buffer = gyro_x_data[i:i + buffer_size]
            gyro_y_buffer = gyro_y_data[i:i + buffer_size]
            gyro_z_buffer = gyro_z_data[i:i + buffer_size]
            
            if len(gyro_x_buffer) == buffer_size:
                denoised_x = dwt_denoise_custom(np.array(gyro_x_buffer), h, g, level=level, threshold=threshold)
                denoised_y = dwt_denoise_custom(np.array(gyro_y_buffer), h, g, level=level, threshold=threshold)
                denoised_z = dwt_denoise_custom(np.array(gyro_z_buffer), h, g, level=level, threshold=threshold)
                
                denoised_gyro_x.extend(denoised_x)
                denoised_gyro_y.extend(denoised_y)
                denoised_gyro_z.extend(denoised_z)
        
        # Save results to a DataFrame for this configuration
        denoised_df = pd.DataFrame({
            'Denoised_Gyro_x': denoised_gyro_x,
            'Denoised_Gyro_y': denoised_gyro_y,
            'Denoised_Gyro_z': denoised_gyro_z
        })
        output_path = rf'C:\Users\syari\OneDrive\Documents\Skripsii\balance_complete\balance_detection\scripts\UKF_dibeneri\PengujianFilter\DWT\DWT_Filter_L{level}_T{threshold}.csv'
        denoised_df.to_csv(output_path, index=False)
        results.append((level, threshold, output_path))

print("Filtering complete. Results saved:")
for level, threshold, path in results:
    print(f"Level: {level}, Threshold: {threshold}, Path: {path}")
