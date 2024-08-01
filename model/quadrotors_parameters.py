import numpy as np

m = 2.2  # kg
g = 9.8
dt = 0.01
time = 20 # s
dis = 0.225  # Distance from the center of the motor to the center of the quadrotors. (m)
# Moment of inertia
J_xx = 2.214e-2
J_yy = 2.214e-2
J_zz = 4.203e-2
# Moment of inertia of motor and propellers
J_motor = 1.15e-5
# Fuselage drag coefficient
C_drag = 6.579e-2
# Comprehensive thrust coefficient of single propeller
C_thrust = 1.472e-6
# Comprehensive torque coefficient of single propeller
C_moment = 1.421e-8
Cr = 1600  # RPM

J = np.array([J_xx, 0, 0,
              0, J_yy, 0,
              0, 0, J_zz]).reshape(3, 3)

x_e = np.array([1, 0, 0]).reshape(3, 1)
y_e = np.array([0, 1, 0]).reshape(3, 1)
z_e = np.array([0, 0, 1]).reshape(3, 1)

thrust_matrix = np.array([C_thrust, C_thrust, C_thrust, C_thrust,
                          np.sqrt(2)*dis * C_thrust/2, -np.sqrt(2)*dis * C_thrust/2, -np.sqrt(2)*dis * C_thrust/2, np.sqrt(2)*dis * C_thrust/2,
                          np.sqrt(2)*dis * C_thrust/2, np.sqrt(2)*dis * C_thrust/2, -np.sqrt(2)*dis * C_thrust/2, -np.sqrt(2)*dis * C_thrust/2,
                          C_moment, -C_moment, C_moment, -C_moment]).reshape(4, 4)
