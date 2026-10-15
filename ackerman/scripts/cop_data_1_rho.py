import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Read data from CSV file
# Assuming the CSV file is named 'data.csv' and is located in the same directory as the script
df = pd.read_csv('laser_data_1ms_2.csv')

# Extract data from the DataFrame
df['obs_kanan'] = np.where(df['Right Obstacle Angle'] > 0,
                        (df['Right Obstacle Angle'] - 180) * -1,
                        (df['Right Obstacle Angle'] + 180) * -1,
                        )
                
df['obs_kiri'] = np.where(df['Left Obstacle Angle'] > 0,
                        (df['Left Obstacle Angle'] - 180) * -1,
                        (df['Left Obstacle Angle'] + 180) * -1,
                        )

obs_kanan = df['obs_kanan'].values
obs_kiri = df['obs_kiri'].values
df['rho'] = np.where(df['Resultant Angle'] > 0,
                     (df['Resultant Angle'] - 180) * -1,
                     (df['Resultant Angle'] + 180) * -1)
rho = df['rho'].values

# X-axis: Index of the observations
x = range(len(obs_kanan))

# Create the plot
fig, ax1 = plt.subplots()

# ax1.set_xlabel('Time')
# ax1.set_ylabel('Sudut $\\it{Obstacle}$ (°)')
# ax1.plot(x, obs_kanan, label='$\\it{Obstacle}$ Kanan (°)', color='tab:blue')
# ax1.plot(x, obs_kiri, label='$\\it{Obstacle}$ Kiri (°)', color='tab:green')
# ax1.tick_params(axis='y')
# ax1.plot(x, velocity, label='Velocity', color='tab:red', linestyle='--')

ax1.set_xlabel('Time')
ax1.set_ylabel('Obstacle Angle (°)')
ax1.plot(x, obs_kanan, label='Right Obstacle (°)', color='tab:blue')
ax1.plot(x, obs_kiri, label='Left Obstacle (°)', color='tab:green')
ax1.tick_params(axis='y')


# Create a secondary y-axis for the velocity
ax2 = ax1.twinx()
ax2.set_ylabel('Robot Heading Angle (°)')
ax2.plot(x, rho, label='Robot Heading Angle (°)', color='tab:red', linestyle='--')
ax2.tick_params(axis='y')

# Add the legends
fig.tight_layout()
ax1.legend(loc='upper left')
ax2.legend(loc='upper right')

# Show the plot
plt.title('Grafik $\\it{Rho}$')
plt.show()
