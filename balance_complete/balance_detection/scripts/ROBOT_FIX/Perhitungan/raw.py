#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
import csv
import os

# File CSV untuk IMU dan Joint States
imu_csv_file = "datas_waktu/raw_imu.csv"
joint_states_csv_file = "datas_waktu/joint_states.csv"

# Membuat file CSV dengan header
def create_csv_files():
    with open(imu_csv_file, mode='w') as imu_file:
        imu_writer = csv.writer(imu_file)
        imu_writer.writerow(["Time", "Accel_x", "Accel_y", "Accel_z", "Orientation_x", "Orientation_y", "Orientation_z", "Orientation_w", "Gyro_x", "Gyro_y", "Gyro_z"])
    
    with open(joint_states_csv_file, mode='w') as joint_file:
        joint_writer = csv.writer(joint_file)
        joint_writer.writerow(["Time", "Joint States"])

# Callback untuk IMU
def imu_callback(data):
    with open(imu_csv_file, mode='a') as imu_file:
        imu_writer = csv.writer(imu_file)
        imu_writer.writerow([
            data.header.stamp.to_sec(),
            data.linear_acceleration.x,
            data.linear_acceleration.y,
            data.linear_acceleration.z,
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w,
            data.angular_velocity.x,
            data.angular_velocity.y,
            data.angular_velocity.z
        ])

# Callback untuk Joint States
def joint_state_callback(data):
    with open(joint_states_csv_file, mode='a') as joint_file:
        joint_writer = csv.writer(joint_file)
        joint_writer.writerow([
            data.header.stamp.to_sec(),
            str(data.position)  # Mengkonversi posisi joint ke string
        ])

def main():
    rospy.init_node('data_saver', anonymous=True)
    create_csv_files()

    rospy.Subscriber("/robotis/open_cr/imu", Imu, imu_callback)
    rospy.Subscriber("/robotis/present_joint_states", JointState, joint_state_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
