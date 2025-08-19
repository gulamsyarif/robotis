import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.patches import Polygon

# Load CSV files
csv_files = [
    r'C:\Users\syari\OneDrive\Documents\Skripsii\balance_complete\balance_detection\scripts\UKF_dibeneri\Data\berdiri_statis\UKF_complete.csv',
    r'C:\Users\syari\OneDrive\Documents\Skripsii\balance_complete\balance_detection\scripts\UKF_dibeneri\Data\berdiri_tidak_seimbang\UKF_complete.csv',
    r'C:\Users\syari\OneDrive\Documents\Skripsii\balance_complete\balance_detection\scripts\UKF_dibeneri\Data\jatuh\UKF_complete.csv',
]

# Support Polygon and Safe Region Coordinates
support_polygon = np.array([
    [-8.6, 6.5], [8.6, 6.5], [8.6, -5.8],
    [8, -6.5], [-8, -6.5], [-8.6, -5.8]
])

safe_region = np.array([
    [-6.6, 4.5], [6.6, 4.5], [6.6, -3.8],
    [6, -4.5], [-6, -4.5], [-6.6, -3.8]
])

# List of custom titles for each subplot
titles = ["Static Squat - CoM and Support Polygon Projection", "Static Standing - CoM and Support Polygon Projection", "Walking - CoM and Support Polygon Projection"]

# Create a 2x2 subplot layout
fig, axs = plt.subplots(2, 2, figsize=(10, 10))
axs = axs.flatten()  # Flatten the 2x2 array for easier indexing

for i, csv_file in enumerate(csv_files):
    # Load data from the current CSV file
    data = pd.read_csv(csv_file).iloc[1000:3000]  # Adjust row range as needed
    
    # Extract COM data
    com_x = data['com_x']
    com_y = data['com_y']
    
    # Plotting on the current subplot
    ax = axs[i]
    ax.set_aspect('equal', adjustable='datalim')
    
    # Plot COM path and base point
    ax.scatter(0, 0, color='navy', label='Base Point', s=100, zorder=5)
    ax.plot(com_x, com_y, color='olive', label='Center of Mass (COM)')
    
    # Add polygons for support and safe regions
    poly_support = Polygon(support_polygon, closed=True, edgecolor='black', facecolor='lightyellow', label='Support Polygon')
    ax.add_patch(poly_support)
    
    poly_safe = Polygon(safe_region, closed=True, edgecolor='black', facecolor='cornsilk', label='Safe Region')
    ax.add_patch(poly_safe)
    
    # Set axis limits with some margin
    x_min = min(com_x.min(), support_polygon[:, 0].min(), safe_region[:, 0].min())
    x_max = max(com_x.max(), support_polygon[:, 0].max(), safe_region[:, 0].max())
    y_min = min(com_y.min(), support_polygon[:, 1].min(), safe_region[:, 1].min())
    y_max = max(com_y.max(), support_polygon[:, 1].max(), safe_region[:, 1].max())
    
    ax.set_xlim([x_min - 0.5, x_max + 0.5])
    ax.set_ylim([y_min - 0.5, y_max + 0.5])
    
    # Use custom title from the list
    ax.set_title(titles[i])
    ax.set_xlabel('X Position [cm]')
    ax.set_ylabel('Y Position [cm]')
    ax.legend()

# Adjust layout and display the plot
plt.tight_layout()
plt.savefig(r'C:\Users\syari\OneDrive\Documents\Skripsii\balance_complete\balance_detection\scripts\UKF_dibeneri\Pengujian3\Pengujian3.png', dpi=600)
plt.show()
