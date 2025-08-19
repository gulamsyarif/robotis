import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.patches import Polygon

# Load data from CSV
data = pd.read_csv(r'C:\Users\syari\Downloads\balance_detection_modified\balance_detection\scripts\UKF_dibeneri\Pengujian3\berjalan1\UKF_complete.csv').iloc[1000:3000] # Replace with the path to your CSV file

# Extract relevant columns for before freeze (COM) and during freeze (ZMP)
com_x = data['com_x']
com_y = data['com_y']
zmp_x = data['zmp_x']
zmp_y = data['zmp_y']

# Set freeze point to (0, 0) explicitly
freeze_point = (0, 0)

# Support Polygon Coordinates (replace with your actual data if needed)
support_polygon = np.array([
    [-8.6, 6.5], [8.6, 6.5], [8.6, -5.8],
    [8, -6.5], [-8, -6.5], [-8.6, -5.8]
])

# Adjusted Safe Region Coordinates (smaller region inside the support polygon)
safe_region = np.array([
    [-6.6, 4.5], [6.6, 4.5], [6.6, -3.8],
    [6, -4.5], [-6, -4.5], [-6.6, -3.8]
])

# Create the figure and axis
fig, ax = plt.subplots()

# Set aspect ratio to be equal
ax.set_aspect('equal', adjustable='datalim')

# Plot the paths (Before freeze using COM data, During freeze using ZMP data)
# Highlight the freeze point (which is set at 0,0)
ax.scatter(0, 0, color='indigo', label='Base Point')
ax.plot(com_x, com_y, color='olive', label='Center of Mass (COM)')
ax.plot(zmp_x, zmp_y, color='mediumvioletred', label='Zero Moment Point (ZMP)')

# Add polygons to represent the support polygon and safe region
poly_support = Polygon(support_polygon, closed=True, edgecolor='black', facecolor='lightyellow', label='Support Polygon')
ax.add_patch(poly_support)

poly_safe = Polygon(safe_region, closed=True, edgecolor='black', facecolor='cornsilk', label='Safe Region')
ax.add_patch(poly_safe)

# Adjust the axis limits to fit all data comfortably
x_min = min(com_x.min(), zmp_x.min(), support_polygon[:, 0].min(), safe_region[:, 0].min())
x_max = max(com_x.max(), zmp_x.max(), support_polygon[:, 0].max(), safe_region[:, 0].max())
y_min = min(com_y.min(), zmp_y.min(), support_polygon[:, 1].min(), safe_region[:, 1].min())
y_max = max(com_y.max(), zmp_y.max(), support_polygon[:, 1].max(), safe_region[:, 1].max())

# Set axis limits with some margin
ax.set_xlim([x_min - 0.5, x_max + 0.5])
ax.set_ylim([y_min - 0.5, y_max + 0.5])

# Add labels and a legend
ax.set_title('Comparison of COM and ZMP within Support Polygon when Robot Fell to the Ground')
ax.set_xlabel('X Position [cm]')
ax.set_ylabel('Y Position [cm]')
ax.legend()

# Display the plot
plt.show()
