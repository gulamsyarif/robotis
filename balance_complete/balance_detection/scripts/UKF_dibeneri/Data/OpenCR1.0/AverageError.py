import numpy as np

# Fungsi untuk menghitung rata-rata error berdasarkan rumus akurasi
def average_error_by_accuracy(true_values, predicted_values):
    true_sum = np.sum(true_values)  # Total nilai aktual
    predicted_sum = np.sum(predicted_values)  # Total nilai prediksi

    if predicted_sum == 0:  # Hindari pembagian nol
        avg_error = float('inf')
    else:
        avg_error = abs((true_sum / predicted_sum) - 1) * 100  # Rata-rata error dalam persen

    return avg_error, true_sum, predicted_sum

# Fungsi untuk menghitung akurasi dari rata-rata error
def accuracy_from_error(avg_error):
    accuracy = max(100 - avg_error, 0)  # Pastikan akurasi tidak negatif
    return accuracy

# Fungsi untuk menghitung MAPE
def mean_absolute_percentage_error(true_values, predicted_values):
    true_values = np.array(true_values)
    predicted_values = np.array(predicted_values)
    
    # Hindari pembagian nol
    with np.errstate(divide='ignore', invalid='ignore'):
        mape = np.mean(np.abs((true_values - predicted_values) / true_values)) * 100

    return np.nan_to_num(mape, nan=float('inf'))  # Ganti NaN dengan inf atau nilai valid

# Data Accelerometer (X, Y, Z) dari Sensor Robot dan OpenCR
accelerometer_robot = {
    'X': [0.10, 0.11, 0.12, 0.10, 0.10],
    'Y': [0.14, 0.14, 0.15, 0.14, 0.17],
    'Z': [9.81, 9.81, 9.80, 9.79, 9.80]
}

accelerometer_opencr = {
    'X': [0.11, 0.09, 0.11, 0.10, 0.08],
    'Y': [0.12, 0.12, 0.12, 0.13, 0.14],
    'Z': [9.80, 9.81, 9.80, 9.80, 9.80]
}

# Data Gyroscope (X, Y, Z) dari Sensor Robot dan OpenCR
gyroscope_robot = {
    'X': [-2.99, -1.79, -1.07, -6.47, -3.88],
    'Y': [0.10, 0.11, 0.10, 0.09, 0.10],
    'Z': [0.05, 0.05, 0.05, 0.05, 0.05]
}

gyroscope_opencr = {
    'X': [-0.04, -0.04, -0.04, -0.04, -0.04],
    'Y': [0.02, 0.02, 0.01, 0.01, 0.01],
    'Z': [0.01, 0.01, 0.01, 0.01, 0.01]
}

# Data Orientation (X, Y, Z, W) dari Sensor Robot dan OpenCR
orientation_robot = {
    'X': [0.03, 0.02, 0.02, 0.03, 0.02],
    'Y': [0.02, 0.02, 0.01, 0.02, 0.02],
    'Z': [0.01, 0.02, 0.03, 0.01, 0.02],
    'W': [0.99, 0.99, 0.99, 0.99, 0.99]
}

orientation_opencr = {
    'X': [0.02, 0.02, 0.01, 0.02, 0.02],
    'Y': [0.02, 0.01, 0.01, 0.01, 0.02],
    'Z': [0.02, 0.02, 0.02, 0.02, 0.02],
    'W': [0.99, 0.99, 0.99, 0.99, 0.99]
}

# Fungsi untuk menghitung metrik dan mencetak hasil
def calculate_metrics(sensor_robot, sensor_opencr, label):
    for axis in sensor_robot.keys():
        avg_error, sum_actual, sum_predicted = average_error_by_accuracy(
            sensor_robot[axis], sensor_opencr[axis]
        )
        accuracy = accuracy_from_error(avg_error)
        mape = mean_absolute_percentage_error(
            sensor_robot[axis], sensor_opencr[axis]
        )
        print(f"=== {label} {axis}-axis ===")
        print(f"Sum of Actual: {sum_actual:.4f}")
        print(f"Sum of Predicted: {sum_predicted:.4f}")
        print(f"Average Error: {avg_error:.2f}%")
        print(f"Accuracy: {accuracy:.2f}%")
        print(f"MAPE: {mape:.2f}%\n")

# Menghitung metrik untuk Accelerometer, Gyroscope, dan Orientation
calculate_metrics(accelerometer_robot, accelerometer_opencr, "Accelerometer")
calculate_metrics(gyroscope_robot, gyroscope_opencr, "Gyroscope")
calculate_metrics(orientation_robot, orientation_opencr, "Orientation")
