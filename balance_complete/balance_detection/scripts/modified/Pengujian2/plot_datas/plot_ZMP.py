import pandas as pd
import matplotlib.pyplot as plt

# Load CSV file
csv_file = r'C:\Users\syari\OneDrive\Documents\Skripsii\balance_complete\balance_detection\scripts\modified\Pengujian2\nendang\WT_clean.csv'  # Ganti dengan path yang sesuai
df = pd.read_csv(csv_file)

# Tentukan koordinat support polygon berdasarkan support phase
df['x_min'] = df['support_phase'].apply(lambda sp: -8.6 if sp == 'Double Support' else -4.3)
df['x_max'] = df['support_phase'].apply(lambda sp: 8.6 if sp == 'Double Support' else 4.3)
df['y_min'] = -6.5
df['y_max'] = 6.5

# Create subplots
fig, axs = plt.subplots(2, 1, figsize=(10, 12))

# Plot for X values
axs[0].fill_between(df.index, df['x_min'], df['x_max'], color='lightblue', label='Support Polygon Area')
axs[0].plot(df.index, df['zmp_x'], 'r-', label='ZMP')
axs[0].plot(df.index, df['com_x'], 'g--', label='CoM')
axs[0].set_title('ZMP, CoM, and Support Polygon Area Over Time (X-axis)')
axs[0].set_xlabel('Time (s)')
axs[0].set_ylabel('Value')
axs[0].legend()

# Plot for Y values
axs[1].fill_between(df.index, df['y_min'], df['y_max'], color='lightblue', label='Support Polygon Area')
axs[1].plot(df.index, df['zmp_y'], 'r-', label='ZMP')  # Ensure you have a 'zmp_y' column
axs[1].plot(df.index, df['com_y'], 'g--', label='CoM')  # Ensure you have a 'com_y' column
axs[1].set_title('ZMP, CoM, and Support Polygon Area Over Time (Y-axis)')
axs[1].set_xlabel('Time (s)')
axs[1].set_ylabel('Value')
axs[1].legend()

# Show plot
plt.tight_layout()
plt.show()
