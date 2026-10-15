import pandas as pd
import matplotlib.pyplot as plt

# Read data from CSV file
# Assuming the CSV file is named 'data.csv' and is located in the same directory as the script
df = pd.read_csv('laser_data_1ms_2.csv')

# Extract data from the DataFrame
df['Fstatis'] = df['Force Static'] + df['Force Static']
fstatis = df['Fstatis'].values
df['fnav'] = df['Resultant Magnitude'] - 2
fnav = df['fnav'].values

# X-axis: Index of the observations
x = range(len(fnav))

# Create the plot
fig, ax1 = plt.subplots()

ax1.set_xlabel('Time')
ax1.set_ylabel('Force (N)')
ax1.plot(x, fstatis, label='Fstatis', color='#003cff')
ax1.plot(x, fnav, label='Fnav', color='#fff200')
ax1.tick_params(axis='y')

# Add the legends
fig.tight_layout()
ax1.legend(loc='upper left')

# Show the plot
plt.title('$\\it{Force}$ $\\it{Force}$')
plt.show()
