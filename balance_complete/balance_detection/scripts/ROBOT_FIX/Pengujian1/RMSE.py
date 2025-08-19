import numpy as np

# Fungsi untuk menghitung RMSE dan Akurasi
def calculate_rmse_accuracy(y_actual, y_predicted):
    # Konversi ke numpy array
    y_actual = np.array(y_actual)
    y_predicted = np.array(y_predicted)
    
    # Hitung RMSE
    rmse = np.sqrt(np.mean((y_actual - y_predicted) ** 2))
    
    # Hitung rata-rata nilai aktual (untuk persentase kesalahan)
    mean_actual = np.mean(np.abs(y_actual))
    error_percentage = (rmse / mean_actual) * 100  # Dalam persen
    
    # Hitung persentase akurasi
    accuracy_percentage = 100 - error_percentage
    
    return rmse, mean_actual, error_percentage, accuracy_percentage

# Contoh data
y_predicted = [0.9961, 0.9961, 0.9961, 0.9964, 0.9963]     # Nilai sebenarnya
y_actual = [0.9950, 0.9959, 0.9961, 0.9961, 0.9961]  # Nilai prediksi

# Hitung RMSE, mean_actual, error_percentage, dan akurasi
rmse, mean_actual, error_percentage, accuracy = calculate_rmse_accuracy(y_actual, y_predicted)

# Cetak hasil dengan format yang rapi
print(f"Rata-rata nilai aktual (mean_actual): {mean_actual:.4f}")
print(f"RMSE: {rmse:.4f}")
print(f"Persentase kesalahan (error_percentage): {error_percentage:.2f}%")
print(f"Akurasi: {accuracy:.2f}%")
