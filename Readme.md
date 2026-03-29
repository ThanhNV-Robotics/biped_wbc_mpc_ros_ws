# Biped Robot Software

## Init Submodules
The package biped_wb_mpc contains different git submodules from other source. We need to clone those:

```bash
cd biped_wb_mpc
git submodule update --init --recursive
```

This is the core source code for us to develope controllers for our robots. The main target is to utilize this source code and implement on our robot

## Packages
### 1. biped_robot_run
Contain launch file to run the robot's control system. We develope similar like the package robot_models in the biped_wb_mpc package.

### 2. mujoco_ros2_control
Package to simulate a dummy robot and publish joint_states and imu sensor data similar like the real robot.
In order to build this, first we download MuJoCo libraries from:

https://github.com/google-deepmind/mujoco/releases

Then we need to configure environment variable for mujoco directory:

```bash
export MUJOCO_DIR=/biped_wbc_mpc_ws/src/biped_wbc_mpc/mujoco-3.5.0
```
The MuJoCo 3.5.0 directory is located in the workspace root. The original source for mujoco_ros2_control is given in the link below:

https://github.com/moveit/mujoco_ros2_control

### 3. control_actions
Package to run some simple actions (move one joint, initial standing, etc)

### 4. humanoid_hardware
Package to set up hardware interface for real robot.


### robot_state_visualization
This package subscribe to joint_states and imu and visualize the robot on a virtual environment to check the sensor data.
