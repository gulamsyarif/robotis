#!/usr/bin/env python

import rospy
import csv
from sensor_msgs.msg import Imu

# Inisialisasi variabel counter untuk menghitung set data
counter = 0
max_data = 5000

# Fungsi callback untuk mendapatkan data dari topic IMU
def imu_callback(data):
    global counter
    counter += 1

    # Buka atau buat file CSV untuk menulis data
    with open('imu_data_jongkok.csv', mode='a') as file:
        writer = csv.writer(file)
        
        # Tulis header di file CSV jika file kosong
        if file.tell() == 0:
            writer.writerow(['seq', 'secs', 'nsecs', 
                             'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w', 
                             'orientation_covariance_0', 'orientation_covariance_1', 'orientation_covariance_2',
                             'orientation_covariance_3', 'orientation_covariance_4', 'orientation_covariance_5',
                             'orientation_covariance_6', 'orientation_covariance_7', 'orientation_covariance_8',
                             'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z',
                             'angular_velocity_covariance_0', 'angular_velocity_covariance_1', 'angular_velocity_covariance_2',
                             'angular_velocity_covariance_3', 'angular_velocity_covariance_4', 'angular_velocity_covariance_5',
                             'angular_velocity_covariance_6', 'angular_velocity_covariance_7', 'angular_velocity_covariance_8',
                             'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z',
                             'linear_acceleration_covariance_0', 'linear_acceleration_covariance_1', 'linear_acceleration_covariance_2',
                             'linear_acceleration_covariance_3', 'linear_acceleration_covariance_4', 'linear_acceleration_covariance_5',
                             'linear_acceleration_covariance_6', 'linear_acceleration_covariance_7', 'linear_acceleration_covariance_8'])

        # Menulis data ke file CSV termasuk kovariansnya
        writer.writerow([data.header.seq, data.header.stamp.secs, data.header.stamp.nsecs,
                         data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w,
                         data.orientation_covariance[0], data.orientation_covariance[1], data.orientation_covariance[2],
                         data.orientation_covariance[3], data.orientation_covariance[4], data.orientation_covariance[5],
                         data.orientation_covariance[6], data.orientation_covariance[7], data.orientation_covariance[8],
                         data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z,
                         data.angular_velocity_covariance[0], data.angular_velocity_covariance[1], data.angular_velocity_covariance[2],
                         data.angular_velocity_covariance[3], data.angular_velocity_covariance[4], data.angular_velocity_covariance[5],
                         data.angular_velocity_covariance[6], data.angular_velocity_covariance[7], data.angular_velocity_covariance[8],
                         data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z,
                         data.linear_acceleration_covariance[0], data.linear_acceleration_covariance[1], data.linear_acceleration_covariance[2],
                         data.linear_acceleration_covariance[3], data.linear_acceleration_covariance[4], data.linear_acceleration_covariance[5],
                         data.linear_acceleration_covariance[6], data.linear_acceleration_covariance[7], data.linear_acceleration_covariance[8]])

    # Hentikan node jika jumlah data mencapai 5000 set
    if counter >= max_data:
        rospy.signal_shutdown('Mencapai batas 5000 set data')

def listener():
    # Inisialisasi node ROS
    rospy.init_node('imu_csv_logger', anonymous=True)

    # Subscribe ke topic IMU
    rospy.Subscriber("/robotis/open_cr/imu", Imu, imu_callback)

    # Menjalankan subscriber
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
