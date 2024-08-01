import numpy as np

# Define roll, pitch, and yaw angles (in radians)
roll = np.radians(30)  # Example value, convert degrees to radians
pitch = np.radians(45)  # Example value
yaw = np.radians(60)    # Example value

# Rotation matrices
R_roll = np.array([[1, 0, 0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll), np.cos(roll)]])

R_pitch = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])

R_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                  [np.sin(yaw), np.cos(yaw), 0],
                  [0, 0, 1]])

# Compute the total rotation matrix
R = np.dot(R_yaw, np.dot(R_pitch, R_roll))

# Print the resulting matrix
print("Rotation Matrix R:")
print(R)
