import numpy as np
import matplotlib.pyplot as plt

# with open('save_data.npy', 'r') as file:
#     loaded_data = json.load(file)
loaded_data = np.load('jacobian_evaluate.npy')
data = np.array(loaded_data)  # Convert back to NumPy array

# Trim data from around 3 seconds
time_mask = data[:, 0] >= 4
data = data[time_mask]

t = data[:, 0]                  # time
theta_m5_mujo_vel = data[:, 1]        
theta_m5_jaco_vel = data[:, 2]
theta_m6_mujo_vel = data[:, 3]        
theta_m6_jaco_vel = data[:, 4]

theta_5_vel = data[:,5]
theta_6_vel = data[:,6]         

# set font size for all plots
plt.rcParams.update({'font.size': 17})
# theta_m5
plt.figure(figsize=(10, 6))

plt.plot(t, theta_m5_jaco_vel, label='theta_m5_jaco_vel', color='black', linestyle='--')
plt.plot(t, theta_m5_mujo_vel, label='theta_m5_mujoco_vel', color='red', linestyle='-')

plt.xlabel('Time (s)')
plt.ylabel('Motor Velocity (rad/s)')
plt.title('Motor Velocities from Mujoco and Jaco')
plt.legend()
plt.grid(True)

# theta_m6
plt.figure(figsize=(10, 6))

plt.plot(t, theta_m6_jaco_vel, label='theta_m6_jaco_vel', color='black', linestyle='--')
plt.plot(t, theta_m6_mujo_vel, label='theta_m6_mujoco_vel', color='red', linestyle='-')

plt.xlabel('Time (s)')
plt.ylabel('Motor Velocity (rad/s)')
plt.title('Motor Velocities from Mujoco and Jaco')
plt.legend()
plt.grid(True)

plt.show()