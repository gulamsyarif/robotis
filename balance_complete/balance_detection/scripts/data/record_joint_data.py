#!/usr/bin/env python

import rospy
import csv
from sensor_msgs.msg import JointState

# Inisialisasi variabel counter untuk menghitung set data
counter = 0
max_data = 5000

# Fungsi callback untuk mendapatkan data dari topic present_joint_states
def joint_state_callback(data):
    global counter
    counter += 1

    # Buka atau buat file CSV untuk menulis data
    with open('joint_states_data.csv', mode='a') as file:
        writer = csv.writer(file)
        
        # Tulis header di file CSV jika file kosong
        if file.tell() == 0:
            # Header mencakup data dari 'name', 'position', 'velocity', dan 'effort'
            header = ['seq', 'secs', 'nsecs']  # untuk header timestamp
            for joint in data.name:
                header.extend([joint + '_position', joint + '_velocity', joint + '_effort'])
            writer.writerow(header)

        # Menyiapkan baris data yang akan ditulis
        row = [data.header.seq, data.header.stamp.secs, data.header.stamp.nsecs]
        
        # Tambahkan data position, velocity, dan effort secara berurutan
        for i in range(len(data.name)):
            row.append(data.position[i])
            row.append(data.velocity[i])
            row.append(data.effort[i])

        # Menulis data ke file CSV
        writer.writerow(row)

    # Hentikan node jika jumlah data mencapai 5000 set
    if counter >= max_data:
        rospy.signal_shutdown('Mencapai batas 5000 set data')

def listener():
    # Inisialisasi node ROS
    rospy.init_node('joint_state_csv_logger', anonymous=True)

    # Subscribe ke topic present_joint_states
    rospy.Subscriber("/robotis/present_joint_states", JointState, joint_state_callback)

    # Menjalankan subscriber
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
