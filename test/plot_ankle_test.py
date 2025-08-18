import json
import numpy as np
import matplotlib.pyplot as plt

# with open('save_data.npy', 'r') as file:
#     loaded_data = json.load(file)
loaded_data = np.load('save_data.npy')
data = np.array(loaded_data)  # Convert back to NumPy array
t = data[:, 0]                  # time
ankle_roll = data[:, 1]        
ankle_pitch = data[:, 2]           



plt.figure(figsize=(10, 6))

# 
plt.plot(t, ankle_roll, label='ankle_roll', color='blue', linestyle='-')
plt.plot(t, ankle_pitch, label='ankle_pitch', color='red')

plt.xlabel('Time (s)')
plt.ylabel('Ankle Angle (rad)')
plt.title('Ankle Angle')
plt.legend()
plt.grid(True)

plt.figure(figsize=(10, 6))
plt.show()