# Biped Robot Software

## Init Submodules
The package biped_wb_mpc contains different git submodules from other source. We need to clone those:

```bash
cd biped_wb_mpc
git submodule update --init --recursive
```

This is the core source code for us to develope controllers for our robots. The main target is to utilize this source code and implement on our robot

Remember to use python 3.12 to compile catkin pkg

### 3. control_actions
Package to run some simple actions (move one joint, initial standing, etc)

### 4. humanoid_hardware
Package to set up hardware interface for real robot.


### robot_state_visualization
This package subscribe to joint_states and imu and visualize the robot on a virtual environment to check the sensor data.
