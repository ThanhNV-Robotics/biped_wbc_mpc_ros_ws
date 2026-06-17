# Biped MPC Controller

ROS2 Control controller integrating Centroidal MPC with PD impedance control for biped whole-body control with sim-to-real transfer.

## Overview

This controller bridges high-level Model Predictive Control (MPC) trajectory planning with low-level hardware interfaces that operate in PD impedance mode with feedforward torque, matching the behavior of real motor drivers.

### Control Architecture

**Real Robot:**
```
MPC Controller → (q_des, qd_des, τ_ff) → Motor Drivers → PD + FF → Motors
```

**Simulation:**
```
MPC Controller → (q_des, qd_des, τ_ff) → MuJoCo Interface → PD + FF → Simulated Robot
```

### Control Law

The controller sends three commands per joint:
- `q_des`: Desired position
- `qd_des`: Desired velocity  
- `τ_ff`: Feedforward torque from inverse dynamics

The hardware interface (simulation) or motor drivers (real robot) apply:
```
τ_total = Kp·(q_des - q) + Kd·(qd_des - qd) + τ_ff
```

## Package Structure

```
biped_mpc_controller/
├── include/biped_mpc_controller/
│   └── mpc_joint_controller.hpp          # Controller header
├── src/
│   └── mpc_joint_controller.cpp          # Controller implementation
├── config/
│   └── mpc_joint_controller.yaml         # Configuration & PD gains
├── launch/
│   └── biped_mpc_sim.launch.py           # Simulation launch file
├── CMakeLists.txt
├── package.xml
└── biped_mpc_controller_plugin.xml       # Plugin description

```

## Dependencies

- `controller_interface` - ROS2 Control interface
- `hardware_interface` - Hardware abstraction
- `humanoid_centroidal_mpc` - Centroidal MPC implementation
- `robot_core` - Robot state/action definitions
- `biped_description` - Robot URDF/MJCF
- `mujoco_ros2_control` - MuJoCo simulation interface (with PD impedance mode)

## Configuration

### Controller Parameters (`config/mpc_joint_controller.yaml`)

```yaml
mpc_joint_controller:
  ros__parameters:
    joints: [...]              # List of controlled joints
    task_file: "..."           # MPC task configuration
    urdf_file: "..."           # Robot URDF
    reference_file: "..."      # Reference trajectory config
    gait_file: "..."           # Gait configuration
    mpc_frequency: 100.0       # MPC solver frequency (Hz)
    verbose: true              # Debug output

pd_gains:
  kp:                          # Position gains for each joint
    left_hip_pitch_joint: 250.0
    # ... other joints
  kd:                          # Velocity gains for each joint
    left_hip_pitch_joint: 2.5
    # ... other joints
```

### Tuning PD Gains

The PD gains simulate motor driver behavior:
- **Kp (position gain)**: Higher values → stiffer position tracking
- **Kd (velocity gain)**: Higher values → more damping, smoother motion

Typical ranges for humanoid robots:
- Hip/knee joints: Kp = 150-300, Kd = 2-5
- Ankle joints: Kp = 50-100, Kd = 1-2

## Building

```bash
cd ~/biped_wbc_mpc_ws
colcon build --packages-select biped_mpc_controller
source install/setup.bash
```

## TODO: Complete MPC Initialization

The controller currently has a **placeholder for MPC initialization** that needs to be completed. In `src/mpc_joint_controller.cpp`, the `initialize_mpc()` function needs:

```cpp
bool MpcJointController::initialize_mpc()
{
  try
  {
    // 1. Load URDF and create RobotDescription
    robot_description_ = std::make_shared<robot::model::RobotDescription>(urdf_file_);
    
    // 2. Load model settings from task file
    // ModelSettings modelSettings = loadModelSettings(task_file_);
    
    // 3. Create Pinocchio interface
    // PinocchioInterface pinocchioInterface = createPinocchioInterface(...);
    
    // 4. Create CentroidalMpcRobotModel
    // auto mpcRobotModel = std::make_shared<CentroidalMpcRobotModel<scalar_t>>(...);
    
    // 5. Initialize MPC solver
    // auto mpc = createMpcSolver(...);
    
    // 6. Create CentroidalMpcMrtJointController
    mpc_controller_ = std::make_unique<ocs2::humanoid::CentroidalMpcMrtJointController>(
        *robot_description_,
        modelSettings,
        *mpcRobotModel,
        *mpc,
        pinocchioInterface,
        mpc_frequency_,
        visualizerPtr  // optional
    );
    
    // 7. Load PD gains
    mpc_controller_->loadJointPDGains(task_file_, verbose_);
    
    // 8. Start MPC thread
    mpc_controller_->startMpcThread(robot_state_);
    
    mpc_initialized_ = true;
    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize MPC: %s", e.what());
    return false;
  }
}
```

Refer to your existing MPC setup in `humanoid_centroidal_mpc_ros2` for the proper initialization sequence.

## Usage

### Simulation

```bash
# Terminal 1: Launch simulation with MPC controller
ros2 launch biped_mpc_controller biped_mpc_sim.launch.py

# Terminal 2: Monitor controller status
ros2 control list_controllers

# Terminal 3: View joint states
ros2 topic echo /joint_states
```

### Real Robot

Once the MPC initialization is complete and tested in simulation, the same controller can be deployed to the real robot by:

1. **Update hardware interface**: Configure `ros2_control` to use your real hardware interface instead of MuJoCo
2. **Verify PD gains**: Ensure PD gains in the config match your motor driver settings
3. **Launch**: Use the same launch structure with your hardware interface

```bash
ros2 launch biped_mpc_controller biped_robot.launch.py  # Create this for hardware
```

## Command Interfaces Claimed

The controller claims **3 command interfaces per joint**:
- `<joint>/position` - Desired position (q_des)
- `<joint>/velocity` - Desired velocity (qd_des)
- `<joint>/feedforward_effort` - Feedforward torque (τ_ff)

## State Interfaces Used

- `<joint>/position` - Measured position
- `<joint>/velocity` - Measured velocity
- `<joint>/effort` - Measured torque

## Architecture Benefits

### Sim-to-Real Transfer
- **Same control code** runs in simulation and on real hardware
- **Same PD + feedforward structure** as real motor drivers
- **Realistic simulation** of control delays and impedance behavior

### Modularity
- MPC planning layer is independent of low-level control
- Easy to swap MPC implementations
- Hardware-agnostic through ROS2 Control

### Safety
- PD impedance provides natural compliance
- Feedforward torque from inverse dynamics improves tracking
- Tunable stiffness/damping per joint

## Troubleshooting

### Controller fails to load
- Check that `biped_mpc_controller` is built and sourced
- Verify plugin XML is correctly exported
- Check controller config YAML syntax

### MPC not initialized
- Complete the `initialize_mpc()` function (see TODO above)
- Verify MPC configuration files exist
- Check task_file, urdf_file, etc. paths in config

### Poor tracking performance
- Tune PD gains in `mpc_joint_controller.yaml`
- Check MPC frequency (typically 50-200 Hz)
- Verify feedforward torques are being computed correctly

### Simulation instability
- Reduce PD gains (especially Kp)
- Increase damping (Kd)
- Check for kinematic constraints or joint limits

## References

- [ROS2 Control Documentation](https://control.ros.org/)
- [Controller Interface API](https://github.com/ros-controls/ros2_control)
- Centroidal MPC implementation: see `humanoid_centroidal_mpc` package

## License

Apache-2.0

## Author

Van Thanh Nguyen (thanhnguyenvan0596@gmail.com)
