# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This repository contains ROS2 Humble packages for the SO-100 robot arm (5-DOF configuration) with MoveIt2 integration and star tracking capabilities. The packages provide URDF models, Gazebo simulation support, hardware control interfaces, motion planning capabilities, and celestial object tracking functionality.

## Build and Run Commands

### Build packages
```bash
cd ~/ros2_ws
# Build all SO-100 packages
colcon build --packages-select so_100_arm star_tracker
# Or build individually
colcon build --packages-select so_100_arm
colcon build --packages-select star_tracker
source install/setup.bash
```

### Launch Commands

**Hardware Interface (Physical Robot):**
```bash
ros2 launch so_100_arm hardware.launch.py
# With visualization:
ros2 launch so_100_arm hardware.launch.py rviz:=true
```

**Simulation (Gazebo):**
```bash
ros2 launch so_100_arm gz.launch.py
```

**MoveIt2 Demo:**
```bash
ros2 launch so_100_arm moveit.launch.py
# With real hardware:
ros2 launch so_100_arm moveit.launch.py use_fake_hardware:=false
```

**RVIZ Visualization:**
```bash
ros2 launch so_100_arm rviz.launch.py
```

**Star Tracker (Celestial Object Tracking):**
```bash
# Basic launch with default target (Polaris)
ros2 launch star_tracker star_tracker.launch.py

# Track specific celestial objects
ros2 launch star_tracker star_tracker.launch.py target_object:=sun
ros2 launch star_tracker star_tracker.launch.py target_object:=moon
ros2 launch star_tracker star_tracker.launch.py target_object:=sirius

# With custom location (latitude, longitude, altitude)
ros2 launch star_tracker star_tracker.launch.py location_lat:=37.7749 location_lon:=-122.4194 location_alt:=10.0

# With environment variables
export LOCATION_LAT=37.7749
export LOCATION_LON=-122.4194
export TARGET_OBJECT=moon
ros2 launch star_tracker star_tracker.launch.py

# GPS-Enhanced Star Tracker (Adafruit Ultimate GPS v3)
ros2 launch star_tracker star_tracker_gps.launch.py

# GPS tracking with specific target
ros2 launch star_tracker star_tracker_gps.launch.py target_object:=moon

# GPS + IMU for GoTo mode (requires BNO055 IMU)
ros2 launch star_tracker star_tracker_gps.launch.py use_imu:=true goto_mode:=true

# GPS with custom serial port
ros2 launch star_tracker star_tracker_gps.launch.py gps_serial_port:=/dev/ttyUSB0
```

### Testing Commands

**Test servo communication (requires so_arm_100_hardware package):**
```bash
sudo chmod 666 /dev/ttyUSB0
ros2 run so_arm_100_hardware test_servo
```

**Send test trajectory:**
```bash
ros2 action send_goal /so_100_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [Shoulder_Rotation, Shoulder_Pitch, Elbow, Wrist_Pitch, Wrist_Roll],
    points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]
  }
}"
```

## Architecture

### Launch File Flow

1. **hardware.launch.py**: Launches physical robot control
   - Loads URDF via xacro with `use_fake_hardware:=false`
   - Starts robot_state_publisher
   - Launches ros2_control_node (controller_manager)
   - Spawns joint_state_broadcaster, then robot and gripper controllers
   - Optionally launches RViz

2. **moveit.launch.py**: MoveIt2 integration
   - Uses MoveItConfigsBuilder for configuration
   - Accepts `use_fake_hardware` parameter (default: true)
   - Launches full MoveIt2 demo stack

3. **gz.launch.py**: Gazebo simulation
   - Spawns robot in Gazebo with physics simulation
   - Configures gz_ros2_control plugin

4. **star_tracker.launch.py**: Celestial object tracking
   - Initializes star tracking node with location and target parameters
   - Calculates altitude/azimuth for celestial objects (sun, moon, stars)
   - Converts celestial coordinates to robot joint positions
   - Sends trajectory commands to arm controller

### Robot Configuration

**Joint Structure (5-DOF):**
- Shoulder_Rotation: Continuous rotation (-π to π rad)
- Shoulder_Pitch: Continuous rotation (-π to π rad)  
- Elbow: Continuous rotation (-π to π rad)
- Wrist_Pitch: Continuous rotation (-π to π rad)
- Wrist_Roll: Continuous rotation (-π to π rad)
- Gripper: Position control (0.0 closed to 0.085 open)

**Control Architecture:**
- Uses ros2_control framework
- Joint trajectory controller for arm motion
- Position controller for gripper
- Hardware interface plugin: `so_arm_100_hardware/SO100ARMHardwareInterface` (for real robot)
- Mock components for simulation/testing

### Key Dependencies

**Required ROS2 packages:**
- moveit2 (moveit_ros_move_group, moveit_ros_visualization, etc.)
- ros2_control (controller_manager, joint_trajectory_controller)
- robot_state_publisher
- xacro
- rviz2
- cv_bridge (for star_tracker)
- tf2_ros (for star_tracker)

**Hardware package (for physical robot):**
- so_arm_100_hardware: Provides hardware interface for Feetech SMS/STS servos

**Python packages (for star_tracker):**
- astropy (optional but recommended): Accurate astronomical calculations
- numpy: Mathematical operations
- Without astropy, basic calculations are used for testing

### Configuration Files

- **controllers.yaml**: Controller manager and controller configurations
- **initial_positions.yaml**: Default joint positions
- **joint_limits.yaml**: Velocity and position limits
- **moveit_controllers.yaml**: MoveIt2 controller settings
- **so_100_arm.srdf**: Semantic robot description (planning groups, end effectors)
- **so_100_arm.urdf.xacro**: Main robot description macro
- **so_100_arm.ros2_control.xacro**: ROS2 Control hardware interface configuration

## Development Notes

### Adding New Features
- Robot URDF is generated from xacro files in `config/` directory
- Controllers are configured in `config/controllers*.yaml`
- Launch files use substitutions for parameters - maintain this pattern
- Hardware interface expects Feetech servos on /dev/ttyUSB0 at 1Mbps

### Star Tracker Architecture
- **star_tracker_node.py**: Main tracking node that:
  - Subscribes to `/joint_states` for current robot position
  - Publishes to `/so_100_arm_controller/joint_trajectory` for movement commands
  - Uses FollowJointTrajectory action client for trajectory execution
  - Converts altitude/azimuth celestial coordinates to joint angles
- **Coordinate Conversion**: Simple mapping where shoulder rotation controls azimuth and shoulder pitch controls altitude
- **Supported Targets**: sun, moon, polaris, sirius (extensible)
- **Update Rate**: Configurable tracking frequency (default: 1 Hz)

### Common Issues
- USB permissions: Run `sudo chmod 666 /dev/ttyUSB0` before hardware launch
- Servo IDs are hardcoded 1-6 (1-5 for joints, 6 for gripper)
- MoveIt2 integration is functional but collision checking may need tuning
- Star tracker requires observer location (lat/lon/alt) for accurate tracking
- Without astropy installed, star tracker uses simplified calculations for testing