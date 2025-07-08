import json
import numpy as np
import matplotlib.pyplot as plt

# with open('save_data.npy', 'r') as file:
#     loaded_data = json.load(file)
loaded_data = np.load('save_data.npy')
data = np.array(loaded_data)  # Convert back to NumPy array
t = data[:, 0]                  # time
ankle_roll_vel_mujoco = data[:, 1]        
ankle_pitch_vel_mujoco = data[:, 2]           

ankle_roll_jaco_vel = data[:, 3]             
ankle_pitch_jaco_vel = data[:, 4]

ankle_roll_pos = np.rad2deg(data[:,5])
ankle_pitch_pos = np.rad2deg(data[:,6])

plt.figure(figsize=(10, 6))

# 
plt.plot(t, ankle_roll_vel_mujoco, label='ankle_roll_vel_mujoco', color='blue', linestyle='-')
plt.plot(t, ankle_roll_jaco_vel, label='ankle_roll_jaco_vel', color='red')

plt.xlabel('Time (s)')
plt.ylabel('Ankle Velocity')
plt.title('Ankle Velocities from Mujoco and Jaco')
plt.legend()
plt.grid(True)

plt.figure(figsize=(10, 6))

# 
plt.plot(t, ankle_roll_pos, label='ankle_roll_angle', color='blue', linestyle='-')
plt.plot(t, ankle_pitch_pos, label='ankle_pitch_angle', color='red')

plt.xlabel('Time (s)')
plt.ylabel('Ankle angle (deg)')
plt.title('Ankle position')
plt.legend()
plt.grid(True)


plt.show()