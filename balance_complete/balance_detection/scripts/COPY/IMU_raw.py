#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
import csv

# Buat atau buka file CSV
csv_file_path = "imu_raw.csv"
with open(csv_file_path, mode='w') as file:
    writer = csv.writer(file)
    # Header CSV
    writer.writerow(["Seq", "Time (secs)", "Time (nsecs)", "Orientation_X", "Orientation_Y", "Orientation_Z", "Orientation_W",
                     "Angular_Vel_X", "Angular_Vel_Y", "Angular_Vel_Z", 
                     "Linear_Acc_X", "Linear_Acc_Y", "Linear_Acc_Z"])

def imu_callback(msg):
    # Ambil data dari pesan IMU
    seq = msg.header.seq
    time_secs = msg.header.stamp.secs
    time_nsecs = msg.header.stamp.nsecs
    orientation_x = msg.orientation.x
    orientation_y = msg.orientation.y
    orientation_z = msg.orientation.z
    orientation_w = msg.orientation.w
    angular_vel_x = msg.angular_velocity.x
    angular_vel_y = msg.angular_velocity.y
    angular_vel_z = msg.angular_velocity.z
    linear_acc_x = msg.linear_acceleration.x
    linear_acc_y = msg.linear_acceleration.y
    linear_acc_z = msg.linear_acceleration.z

    # Simpan data ke file CSV
    with open(csv_file_path, mode='a') as file:
        writer = csv.writer(file)
        writer.writerow([seq, time_secs, time_nsecs, orientation_x, orientation_y, orientation_z, orientation_w,
                         angular_vel_x, angular_vel_y, angular_vel_z,
                         linear_acc_x, linear_acc_y, linear_acc_z])

    rospy.loginfo("Data IMU disimpan: Seq=%d, Time=%d.%d" % (seq, time_secs, time_nsecs))

def main():
    rospy.init_node('imu_data_saver_node', anonymous=True)
    rospy.Subscriber("robotis/open_cr/imu", Imu, imu_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
