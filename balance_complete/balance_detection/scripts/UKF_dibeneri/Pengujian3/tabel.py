import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.patches import Polygon
from matplotlib import gridspec
from matplotlib import rcParams

# Set global font size
rcParams.update({'font.size': 14})

csv_files = [
    {'path': r'C:\Users\syari\OneDrive\Documents\Skripsii\balance_complete\balance_detection\scripts\UKF_dibeneri\Data\berdiri_statis\UKF_complete.csv', 'iloc_range': (6001, 6500)},
    {'path': r'C:\Users\syari\OneDrive\Documents\Skripsii\balance_complete\balance_detection\scripts\UKF_dibeneri\Data\berdiri_tidak_seimbang\UKF_complete.csv', 'iloc_range': (4550, 4600)},
    {'path': r'C:\Users\syari\OneDrive\Documents\Skripsii\balance_complete\balance_detection\scripts\UKF_dibeneri\Data\jatuh\UKF_complete.csv', 'iloc_range': (2030, 2080)},
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
titles = ["Robot Berdiri Statis", 
          "Robot Berdiri Tidak Seimbang", 
          "Robot Jatuh"]

# Create a custom grid layout (2x2 grid, but one subplot is centered in the second row)
fig = plt.figure(figsize=(15, 10))
gs = gridspec.GridSpec(2, 2, figure=fig)

# Axis for the subplots
ax1 = fig.add_subplot(gs[0, 0])  # First subplot in the first row
ax2 = fig.add_subplot(gs[0, 1])  # Second subplot in the first row
ax3 = fig.add_subplot(gs[1, 1:3])  # Third subplot in the second row, centered

# List of axes for easy iteration
axs = [ax1, ax2, ax3]

# Create a list of labels and their positions
labels = ['Base Point', 'Center of Mass (COM)', 'Zero Moment Point (ZMP)', 'Support Polygon', 'Safe Region']

# Iterate through each CSV file
for i, csv_file in enumerate(csv_files):
    # Load data from the current CSV file
    data = pd.read_csv(csv_file['path']).iloc[csv_file['iloc_range'][0]:csv_file['iloc_range'][1]]
    print(f"Data from file {i+1} loaded with range: {csv_file['iloc_range']}")

    # Extract COM data
    com_x = data['com_x']
    com_y = data['com_y']
    zmp_x = data['zmp_x']
    zmp_y = data['zmp_y']
    
    # Plotting on the current subplot
    ax = axs[i]
    ax.set_aspect('equal', adjustable='datalim')
    
    # Plot COM path and base point
    ax.scatter(0, 0, color='navy', label=labels[0], s=100, zorder=5)
    ax.plot(com_x, com_y, color='green', label=labels[1])
    ax.plot(zmp_x, zmp_y, color='red', label=labels[2], linestyle='dashed')

    
    # Add polygons for support and safe regions
    poly_support = Polygon(support_polygon, closed=True, edgecolor='black', facecolor='palegreen', label=labels[3])
    ax.add_patch(poly_support)
    
    poly_safe = Polygon(safe_region, closed=True, edgecolor='black', facecolor='lightgreen', label=labels[4])
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
    ax.set_xlabel('Posisi pada Sumbu X [cm]')
    ax.set_ylabel('Posisi pada Sumbu Y [cm]')
    
# Adjust layout and move the legend to the bottom left
fig.tight_layout()

# Collect handles and labels from the first subplot
handles, labels = ax1.get_legend_handles_labels()

# Add a global legend
fig.legend(handles, labels, loc='lower right', bbox_to_anchor=(0.4, 0.02), ncol=2, fontsize=14)

# Save the figure and show it
plt.savefig(r'C:\Users\syari\OneDrive\Documents\Skripsii\balance_complete\balance_detection\scripts\UKF_dibeneri\Pengujian3\Pengujian3edit.png', dpi=600)
plt.show()
