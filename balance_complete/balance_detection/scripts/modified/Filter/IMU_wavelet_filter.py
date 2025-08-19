import numpy as np
import pandas as pd

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

# Baca data dari file CSV
data = pd.read_csv(r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\modified\Filter\nendang\raw_imu.csv')
gyro_x_data = data['Gyro_x'].values
gyro_y_data = data['Gyro_y'].values
gyro_z_data = data['Gyro_z'].values

# Buffer ukuran tetap untuk proses denoising
buffer_size = 10
denoised_gyro_x, denoised_gyro_y, denoised_gyro_z = [], [], []

# Memproses data dalam buffer dengan DWT denoising
for i in range(0, len(gyro_x_data), buffer_size):
    gyro_x_buffer = gyro_x_data[i:i + buffer_size]
    gyro_y_buffer = gyro_y_data[i:i + buffer_size]
    gyro_z_buffer = gyro_z_data[i:i + buffer_size]

    # Cek jika buffer penuh
    if len(gyro_x_buffer) == buffer_size:
        denoised_x = dwt_denoise_custom(np.array(gyro_x_buffer))
        denoised_y = dwt_denoise_custom(np.array(gyro_y_buffer))
        denoised_z = dwt_denoise_custom(np.array(gyro_z_buffer))

        denoised_gyro_x.extend(denoised_x)
        denoised_gyro_y.extend(denoised_y)
        denoised_gyro_z.extend(denoised_z)

# Menyimpan hasil denoised ke file CSV baru
denoised_df = pd.DataFrame({
    'Denoised_Gyro_x': denoised_gyro_x,
    'Denoised_Gyro_y': denoised_gyro_y,
    'Denoised_Gyro_z': denoised_gyro_z
})
denoised_df.to_csv(r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\modified\Filter\nendang\WT_filtered.csv', index=False)
