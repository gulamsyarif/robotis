servo_data = [
    {"id": 20, "mass": 0.09, "CoM_offset": (0, 2.85, 45.1)},
    {"id": 3, "mass": 0.079, "CoM_offset": (-10.8, 1.3, 37.5)},
    {"id": 4, "mass": 0.079, "CoM_offset": (10.8, 1.3, 37.5)},
    {"id": 5, "mass": 0.031, "CoM_offset": (-23.4, 0.9, 37.5)},
    {"id": 6, "mass": 0.031, "CoM_offset": (23.4, 0.9, 37.5)},
    {"id": 1, "mass": 0.086, "CoM_offset": (-4.5, 1.8, 35.2)},
    {"id": 2, "mass": 0.086, "CoM_offset": (4.5, 1.8, 35.2)},
    {"id": 19, "mass": 1.179, "CoM_offset": (0, 1.8, 35.2)},
    {"id": 9, "mass": 0.082, "CoM_offset": (-3.2, -2.7, 23.5)},
    {"id": 11, "mass": 0.164, "CoM_offset": (-3.2, 1, 23.5)},
    {"id": 10, "mass": 0.082, "CoM_offset": (3.2, -2.7, 23.5)},
    {"id": 12, "mass": 0.246, "CoM_offset": (3.2, 1, 23.5)},
    {"id": 13, "mass": 0.139, "CoM_offset": (-3.2, 0.7, 16.4)},
    {"id": 14, "mass": 0.139, "CoM_offset": (3.2, 0.7, 16.4)},
    {"id": 17, "mass": 0.082, "CoM_offset": (-3.2, -2.7, 3.7)},
    {"id": 18, "mass": 0.082, "CoM_offset": (3.2, -2.7, 3.7)},
    {"id": 15, "mass": 0.282, "CoM_offset": (-3.2, 0.4, 3.7)},
    {"id": 16, "mass": 0.282, "CoM_offset": (3.2, 0.4, 3.7)},
    {"id": 7, "mass": 0.086, "CoM_offset": (-4.5, 1.8, 28.4)},
    {"id": 8, "mass": 0.086, "CoM_offset": (4.5, 1.8, 28.4)}
]

# Hitung total massa
total_mass = sum(servo['mass'] for servo in servo_data)

# Hitung momen total untuk masing-masing koordinat (x, y, z)
total_moment_x = sum(servo['mass'] * servo['CoM_offset'][0] for servo in servo_data)
total_moment_y = sum(servo['mass'] * servo['CoM_offset'][1] for servo in servo_data)
total_moment_z = sum(servo['mass'] * servo['CoM_offset'][2] for servo in servo_data)

# Hitung posisi CoM total
CoM_x = total_moment_x / total_mass
CoM_y = total_moment_y / total_mass
CoM_z = total_moment_z / total_mass

numerator_x = 0
numerator_y = 0
denominator = 0

for servo in servo_data:
    mass = servo["mass"]
    x, y, z = servo["CoM_offset"]
    numerator_x += mass * z * x
    numerator_y += mass * z * y
    denominator += mass * z

x_zmp = numerator_x / denominator
y_zmp = numerator_y / denominator

# Print hasil perhitungan CoM
print("Center of Mass (CoM) total robot:")
print("CoM_x:", CoM_x)
print("CoM_y:", CoM_y)
print("CoM_z:", CoM_z)
print(f"x_zmp: {x_zmp} cm, y_zmp: {y_zmp} cm")

