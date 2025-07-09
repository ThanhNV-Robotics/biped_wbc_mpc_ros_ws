import numpy as np
import matplotlib.pyplot as plt

# with open('save_data.npy', 'r') as file:
#     loaded_data = json.load(file)
loaded_data = np.load('jacobian_evaluate.npy')
data = np.array(loaded_data)  # Convert back to NumPy array
t = data[:, 0]                  # time
theta_m5_mujo_vel = data[:, 1]        
theta_m5_jaco_vel = data[:, 2]           

plt.figure(figsize=(10, 6))

plt.plot(t, theta_m5_mujo_vel, label='theta_m5_vel_mujoco', color='blue', linestyle='-')
plt.plot(t, theta_m5_jaco_vel, label='theta_m5_jaco_vel', color='red')

plt.xlabel('Time (s)')
plt.ylabel('Motor Velocity')
plt.title('Motor Velocities from Mujoco and Jaco')
plt.legend()
plt.grid(True)


plt.show()