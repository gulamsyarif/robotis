def calculate_support_polygon(joint_states, imu_accel_z):
    start_time = time.time()
    l_ank_pitch = np.array([state[id_to_index_map[13]] for state in joint_states])
    r_ank_pitch = np.array([state[id_to_index_map[14]] for state in joint_states])

    # Deteksi fase support
    if l_ank_pitch[0] > 0 and r_ank_pitch[0] > 0:
        current_phase = 'Double Support'
    elif l_ank_pitch[0] <= 0 and r_ank_pitch[0] <= 0:
        current_phase = 'Single Support'
    else:
        if imu_accel_z < -0.35:
            current_phase = 'Single Support'
        else:
            current_phase = 'Double Support'

    # Tentukan koordinat berdasarkan fase
    if current_phase == 'Single Support':
        points = np.array([
            [0, 0],
            [0, -12.3],
            [1.1, -13],
            [7.1, -13],
            [8.7, -12.3],
            [8.7, 0]
        ])
    else:  # Double Support
        points = np.array([
            [0, 0],
            [0, -12.3],
            [1.1, -13],
            [16.6, -13],
            [17.7, -12.3],
            [17.7, 0]
        ])

    # Hitung luas dan keliling
    x = points[:, 0]
    y = points[:, 1]
    area = 0.5 * np.abs(np.dot(x[:-1], y[1:]) - np.dot(y[:-1], x[1:]))
    perimeter = np.sum(np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1)))

    computation_time = time.time() - start_time  # Akhiri pengukuran waktu
    rospy.loginfo("Support Polygon Calculation Time: {:.6f} seconds".format(computation_time))

    return {
        "phase": current_phase,
        "area": area,
        "perimeter": perimeter
    }
