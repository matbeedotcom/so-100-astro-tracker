# Star Tracker Setup and Commands

## Build Commands

### Docker Environment Setup
```bash
# Start Docker container
docker-compose up -d so100-arm

# Access the container
docker exec -it nice_stonebraker bash

# OR use the container name from docker ps
docker exec -it $(docker ps -q --filter ancestor=so100-arm-pi5:latest) bash
```

### Building the Star Tracker Package
```bash
# Inside Docker container
cd /ros2_ws
source /opt/ros/humble/setup.bash

# Build star tracker package
colcon build --packages-select star_tracker

# Source the workspace
source install/setup.bash
```

## Run Commands

### 1. Basic Star Tracking (No Hardware)
```bash
# Track Polaris with default location (NYC)
ros2 run star_tracker star_tracker_node

# Track specific target
ros2 run star_tracker star_tracker_node --ros-args -p target_object:=moon
ros2 run star_tracker star_tracker_node --ros-args -p target_object:=sun
```

### 2. IMU-Enhanced Tracking

#### Start BNO055 IMU Publisher
```bash
# Simple test IMU publisher (if main BNO055 interface has issues)
python3 /ros2_ws/test_bno055_ros.py
```

#### Run Star Tracker with IMU
```bash
# Basic IMU tracking (open-loop)
ros2 run star_tracker star_tracker_node --ros-args \
  -p use_imu:=true \
  -p use_gps:=false

# GoTo mode with IMU feedback (closed-loop)
ros2 run star_tracker star_tracker_node --ros-args \
  -p use_gps:=false \
  -p use_imu:=true \
  -p goto_mode:=true \
  -p update_rate:=2.0
```

### 3. Launch Files (with GPS/IMU support)
```bash
# Basic star tracker
ros2 launch star_tracker star_tracker.launch.py

# With GPS and IMU (full system)
ros2 launch star_tracker star_tracker_gps.launch.py use_imu:=true goto_mode:=true

# GPS only (no IMU)
ros2 launch star_tracker star_tracker_gps.launch.py use_imu:=false
```

## IMU Calibration Procedure

### 1. Test IMU Connection
```bash
# Check I2C device (on host, not Docker)
i2cdetect -y 1
# Should show device at 0x28 or 0x29

# Test direct IMU reading
python3 /ros2_ws/test_bno055_direct.py
```

### 2. Calibrate IMU Orientation
```bash
# Start IMU publisher in one terminal
python3 /ros2_ws/test_bno055_ros.py

# Run calibration in another terminal
python3 /ros2_ws/calibrate_imu_pointing.py

# Follow prompts:
# - Position telescope LEVEL at HORIZON, press 'h'
# - Position telescope pointing UP at ZENITH, press 'z'
# - Position telescope pointing NORTH, press 'n'
# - Press 'c' to calculate calibration
```

### 3. Verify Calibration
```bash
# Monitor IMU readings and telescope pointing
python3 /ros2_ws/test_imu_tracking.py

# Check that:
# - Level = 0° altitude
# - Up = 90° altitude
# - North = 0° azimuth
```

## Testing and Debugging

### Monitor Topics
```bash
# Check IMU data
ros2 topic echo /imu/euler

# Check trajectory commands being sent
ros2 topic echo /so_100_arm_controller/joint_trajectory

# Check joint states
ros2 topic echo /joint_states

# List all topics
ros2 topic list
```

### Test Hardware Interface
```bash
# Send test trajectory (arm should move)
ros2 action send_goal /so_100_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [Shoulder_Rotation, Shoulder_Pitch, Elbow, Wrist_Pitch, Wrist_Roll],
    points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]
  }
}"
```

## Configuration Files

### IMU Calibration Values
Location: `/ros2_ws/src/star_tracker/config/imu_calibration.yaml`

Current calibrated values:
- Pitch offset: +132.3° (altitude correction)
- Yaw offset: -14.4° (azimuth correction)

### Star Alignment
Location: `~/star_alignment.json` (if using alignment mode)

## Troubleshooting

### IMU Not Publishing
```bash
# Check if BNO055 is detected
i2cdetect -y 1

# Try simple publisher
python3 /ros2_ws/test_bno055_ros.py

# Check for errors in main interface
ros2 run star_tracker bno055_interface
```

### Star Tracker Not Moving Arm
1. Check hardware interface is running
2. Verify trajectory commands: `ros2 topic echo /so_100_arm_controller/joint_trajectory`
3. Check servo power and connections
4. Verify joint limits aren't preventing motion

### Calibration Issues
- Ensure IMU is firmly mounted on end-effector
- Calibrate away from magnetic interference
- Use consistent positions for calibration points
- Save calibration file after successful calibration

## Common Parameter Combinations

```bash
# Development/testing (no hardware)
ros2 run star_tracker star_tracker_node --ros-args \
  -p use_gps:=false \
  -p use_imu:=false

# IMU tracking only
ros2 run star_tracker star_tracker_node --ros-args \
  -p use_gps:=false \
  -p use_imu:=true \
  -p goto_mode:=true \
  -p update_rate:=2.0

# Full system (GPS + IMU)
ros2 launch star_tracker star_tracker_gps.launch.py \
  use_imu:=true \
  goto_mode:=true \
  target_object:=moon

# Custom location (no GPS)
ros2 run star_tracker star_tracker_node --ros-args \
  -p use_gps:=false \
  -p location_lat:=37.7749 \
  -p location_lon:=-122.4194 \
  -p target_object:=polaris
```