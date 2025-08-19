#!/usr/bin/env python3
import numpy as np

# Constants
GRAVITY = 9.81  # m/s^2

# Data for servo segments
SERVO_SEGMENT_DATA = {
    1: {'mass': 0.095, 'length': 0.0257, 'com_offset': [0, 3.5, 40]},    # Neck
    2: {'mass': 0.095, 'length': 0.0257, 'com_offset': [0, -3.5, 40]},    # Upper Arm (left)
    3: {'mass': 0.102, 'length': 0.0847, 'com_offset': [0, 10.9, 38]},    # Upper Arm (right)
    4: {'mass': 0.104, 'length': 0.0847, 'com_offset': [0, -10.9, 38]},  # Lower Arm (left)
    5: {'mass': 0.135, 'length': 0.1473, 'com_offset': [0, 17.5, 37.1]},  # Lower Arm (right)
    6: {'mass': 0.135, 'length': 0.1473, 'com_offset': [0, -17.5, 37.1]},     # Upper Leg (left)
    7: {'mass': 0.097, 'length': 0.0282, 'com_offset': [0, 3.2, 28.5]},     # Upper Leg (right)
    8: {'mass': 0.097, 'length': 0.0282, 'com_offset': [0, -3.2, 28.5]},  # Lower Leg (left)
    9: {'mass': 0.097, 'length': 0.0202, 'com_offset': [-0.5, 3.5, 15.8]},  # Lower Leg (right)
    10: {'mass': 0.097, 'length': 0.0202, 'com_offset': [-0.5, -3.5, 15.8]},   # Pelvis (left)
    11: {'mass': 0.144, 'length': 0.1067, 'com_offset': [0, 3.5, 16]},   # Pelvis (right)
    12: {'mass': 0.144, 'length': 0.1067, 'com_offset': [0, -3.5, 16]}, # Lower Torso
    13: {'mass': 0.144, 'length': 0.1091, 'com_offset': [0, 3.5, 14.3]}, # Upper Torso
    14: {'mass': 0.144, 'length': 0.1091, 'com_offset': [0, -3.5, 14.3]},   # Head
    15: {'mass': 0.160, 'length': 0.0404, 'com_offset': [0, 3.4, 3.1]}, # Shoulder (left)
    16: {'mass': 0.160, 'length': 0.0404, 'com_offset': [0, -3.4, 3.1]}, # Shoulder (right)
    17: {'mass': 0.180, 'length': 0.0116, 'com_offset': [-0.5, 3.5, 3.1]},   # Hip (left)
    18: {'mass': 0.180, 'length': 0.0116, 'com_offset': [-0.5, -3.5, 3.1]},   # Hip (right)
    19: {'mass': 0.249, 'length': 0.027, 'com_offset': [0, 0, 41.4]},   # Knee (left)
    20: {'mass': 0.249, 'length': 0.068, 'com_offset': [3.5, -2.3, 45]},   # Knee (right)
}

# Function to calculate the Center of Mass (CoM) of the robot
def calculate_com(body_parts):
    """
    Calculate the Center of Mass (CoM) of the robot.

    :param body_parts: A list of dictionaries where each dictionary represents
                       a body part with its mass and position.
    :return: A tuple (com_x, com_y, com_z) representing the CoM coordinates.
    """
    total_mass = 0
    weighted_sum = np.array([0.0, 0.0, 0.0])

    for part in body_parts:
        mass = part['mass']
        pos = np.array(part['pos'])
        total_mass += mass
        weighted_sum += mass * pos

    # Calculate the Center of Mass
    com = weighted_sum / total_mass
    return com[0], com[1], com[2]

# Function to calculate the Zero Moment Point (ZMP)
def calculate_zmp(com_x, com_y, com_z, acc_x, acc_y, acc_z, height_com):
    """
    Calculate the Zero Moment Point (ZMP) of the robot based on the CoM and accelerations.

    :param com_x: X coordinate of the Center of Mass (m)
    :param com_y: Y coordinate of the Center of Mass (m)
    :param com_z: Z coordinate of the Center of Mass (m)
    :param acc_x: X acceleration from IMU (m/s^2)
    :param acc_y: Y acceleration from IMU (m/s^2)
    :param acc_z: Z acceleration from IMU (m/s^2)
    :param height_com: Height of the Center of Mass (m)
    :return: A tuple (zmp_x, zmp_y) representing the ZMP coordinates in the X-Y plane.
    """
    # Calculate the ZMP using the following equations:
    # ZMP_x = com_x - (com_z / GRAVITY) * acc_x
    # ZMP_y = com_y - (com_z / GRAVITY) * acc_y

    zmp_x = com_x - (height_com / GRAVITY) * acc_x
    zmp_y = com_y - (height_com / GRAVITY) * acc_y

    return zmp_x, zmp_y

# Function to calculate the Support Polygon Area
def calculate_support_polygon(foot_positions):
    """
    Calculate the area of the support polygon based on the foot positions.

    :param foot_positions: A list of tuples representing the (x, y) positions of the feet.
                           Example: [(x1, y1), (x2, y2)]
    :return: The area of the support polygon (m^2).
    """
    # Use the Shoelace formula to calculate the area of the polygon
    # Polygon defined by foot contact points (x, y)
    x = [pos[0] for pos in foot_positions]
    y = [pos[1] for pos in foot_positions]

    # Ensure that we close the polygon by appending the first point at the end
    x.append(x[0])
    y.append(y[0])

    # Shoelace formula for area calculation
    area = 0.5 * np.abs(np.dot(x[:-1], y[1:]) - np.dot(y[:-1], x[1:]))

    return area


# Example usage of the functions
if __name__ == '__main__':
    # Example body parts data using SERVO_SEGMENT_DATA (mass in kg, positions in meters [x, y, z])
    body_parts = [
        {'mass': data['mass'], 'pos': data['com_offset']}
        for part_id, data in SERVO_SEGMENT_DATA.items()
    ]

    # Calculate the Center of Mass
    com_x, com_y, com_z = calculate_com(body_parts)
    print(f"Center of Mass: ({com_x:.3f}, {com_y:.3f}, {com_z:.3f})")

    # Example IMU acceleration data (in m/s^2)
    acc_x, acc_y, acc_z = 0.1, 0.2, 0.0

    # Calculate the ZMP (assuming height_com is com_z)
    zmp_x, zmp_y = calculate_zmp(com_x, com_y, com_z, acc_x, acc_y, acc_z, height_com=com_z)
    print(f"ZMP: ({zmp_x:.3f}, {zmp_y:.3f})")

    # Example foot positions for support polygon calculation
    foot_positions = [(0.1, 0.1), (-0.1, 0.1)]  # Left and right foot positions

    # Calculate the support polygon area
    support_polygon_area = calculate_support_polygon(foot_positions)
    print(f"Support Polygon Area: {support_polygon_area:.3f} m^2")
