import pandas as pd
import matplotlib.pyplot as plt

# Read data from CSV file
# Assuming the CSV file is named 'data.csv' and is located in the same directory as the script
df = pd.read_csv('laser_data_1ms_2.csv')

# Extract data from the DataFrame
obs_kanan = df['Right Obstacle Distance'].values
obs_kiri = df['Left Obstacle Distance'].values
r_kanan = df['r_kanan'].values
r_kiri = df['r_kiri'].values
velocity = df['Velocity'].values

# X-axis: Index of the observations
x = range(len(obs_kanan))

# Create the plot
fig, ax1 = plt.subplots()

# ax1.set_xlabel('Waktu')
# ax1.set_ylabel('Jarak Obstacle (m)', fontstyle='italic')
# ax1.plot(x, obs_kanan, label='$\\it{Obstacle}$ Kanan (m)', color='#003cff')
# ax1.plot(x, r_kanan, label='Radius Proksemik Kanan (m)', color='#89a4fa')
# ax1.plot(x, obs_kiri, label='$\\it{Obstacle}$ Kiri (m)', color='#fff200')
# ax1.plot(x, r_kiri, label='Radius Proksemik Kiri (m)', color='#ffe778')
# ax1.tick_params(axis='y')
# ax1.plot(x, velocity, label='Velocity', color='tab:red', linestyle='--')

ax1.set_xlabel('Time')
ax1.set_ylabel('Obstacle Distances (m)', fontstyle='italic')
ax1.plot(x, obs_kanan, label='Right Obstacles (m)', color='#0031D3')
ax1.plot(x, r_kanan, label='Right Proxemic Radius (m)', color='#89a4fa')
ax1.plot(x, obs_kiri, label='Left Obstacles (m)', color='#3CBD00')
ax1.plot(x, r_kiri, label='Left Proxemic Radius (m)', color='#B2FF8E')
ax1.tick_params(axis='y')



# Create a secondary y-axis for the velocity
ax2 = ax1.twinx()
ax2.set_ylabel('Velocity (m/s)',fontstyle='italic')
ax2.plot(x, velocity, label='$\\it{Velocity}$', color='#fc0303', linestyle='--')
ax2.tick_params(axis='y')

# Add the legends
fig.tight_layout()
ax1.legend(loc='upper left')
ax2.legend(loc='upper right')

# Show the plot
plt.title('Grafik $\\it{Velocity}$')
plt.show()
