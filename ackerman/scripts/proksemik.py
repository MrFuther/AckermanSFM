import numpy as np
import matplotlib.pyplot as plt

# Define the semi-major and semi-minor axes
a = 1.0  # Semi-major axis
b = 0.5  # Semi-minor axis

# Number of points
num_points = 90

# Calculate the angles for the quarter ellipse
angles = np.linspace(0, np.pi / 2, num_points)

# Calculate r for each angle using the ellipse formula
r_values = (a * b) / np.sqrt((b * np.cos(angles))**2 + (a * np.sin(angles))**2)

# Convert polar coordinates (r, theta) to Cartesian coordinates (x, y)
x_values = r_values * np.cos(angles)
y_values = r_values * np.sin(angles)

# Calculate distances and angles from the origin
distances = np.sqrt(x_values**2 + y_values**2)
vector_angles = np.arctan2(y_values, x_values)

# Print the coordinates, distances, and angles of the 80th point
index = 50  # Index 79 corresponds to the 80th point (0-based index)
print(f"Point {index+1}: x = {x_values[index]:.4f}, y = {y_values[index]:.4f}, distance = {distances[index]:.4f}, angle = {np.degrees(vector_angles[index]):.2f} degrees")

# Optionally, plot the points
plt.figure(figsize=(8, 8))
plt.plot(x_values, y_values, 'o', label='Quarter Ellipse Points')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Quarter Ellipse Plot')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()

# import numpy as np

# def calculate_coordinates(a, b, degree):
#     # Convert degree to radians
#     angle = np.deg2rad(degree)
    
#     # Calculate r for the given degree using the ellipse formula
#     r = (a * b) / np.sqrt((b * np.cos(angle))**2 + (a * np.sin(angle))**2)
    
#     # Convert polar coordinates (r, theta) to Cartesian coordinates (x, y)
#     x = r * np.cos(angle)
#     y = r * np.sin(angle)
    
#     # Calculate distance from the origin
#     distance = np.sqrt(x**2 + y**2)    
#     return distance, x, y

# # Define the semi-major and semi-minor axes
# a = 1.0  # Semi-major axis (180 degrees)
# b = 0.4  # Semi-minor axis (90 degrees)

# # Example degree input
# degree_input = 180

# # Calculate distance for the given degree
# distance_value, x_value, y_value = calculate_coordinates(a, b, degree_input)

# # Print the result
# print(f"At {degree_input} degrees: x = {x_value:.4f}, y = {y_value:.4f}, distance = {distance_value:.4f}")

# # Optionally, calculate and print distances for all degrees from 0 to 90
# for degree in range(91):
#     distance, x, y = calculate_coordinates(a, b, degree)
#     print(f"At {degree} degrees: x = {x:.4f}, y = {y:.4f}, distance = {distance:.4f}")

