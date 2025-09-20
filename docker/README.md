# SO-100 Robot Arm Docker Setup

This Docker setup provides a complete ROS2 Humble environment for the SO-100 robot arm with MoveIt2 motion planning and star tracking capabilities.

## Prerequisites

- Docker and Docker Compose installed
- For GUI applications (RViz, Gazebo):
  - Linux: X11 forwarding configured
  - Windows: WSL2 with WSLg or X server (VcXsrv, Xming)
  - macOS: XQuartz installed

## Quick Start

1. **Build the Docker image:**
```bash
docker-compose build
```

2. **Start the container:**
```bash
# For simulation mode (default)
docker-compose up -d

# For hardware mode (physical robot)
ROBOT_MODE=hardware docker-compose up -d

# With star tracker service
docker-compose --profile star-tracking up -d
```

3. **Enter the container:**
```bash
docker-compose exec so100-arm bash
```

4. **Launch the robot:**
```bash
# Inside the container
launch_robot  # Uses ROBOT_MODE environment variable

# Or manually:
ros2 launch so_100_arm moveit.launch.py  # Simulation
ros2 launch so_100_arm hardware.launch.py  # Hardware
```

## Configuration

Edit the `.env` file to configure:

- **Robot Mode**: `simulation`, `hardware`, or `mock`
- **Location**: Set your latitude, longitude, and altitude for star tracking
- **Target Object**: Choose celestial object to track (sun, moon, stars)
- **Serial Port**: Configure USB port for physical robot

## Star Tracker Setup

The star tracker uses the robot arm to point a camera at celestial objects, compensating for Earth's rotation.

### Supported Targets:
- Polaris (North Star) - stationary reference
- Sun - solar tracking
- Moon - lunar tracking
- Custom RA/Dec coordinates

### Usage:
```bash
# Start star tracking service
docker-compose --profile star-tracking up -d star-tracker

# Monitor tracking
docker-compose logs -f star-tracker
```

## Development Workflow

1. **Edit code locally** - Source files are mounted as volumes
2. **Rebuild if needed:**
```bash
docker-compose exec so100-arm bash -c "cd /ros2_ws && colcon build"
```
3. **Test changes:**
```bash
docker-compose exec so100-arm bash -c "source /ros2_ws/install/setup.bash && ros2 launch so_100_arm moveit.launch.py"
```

## Troubleshooting

### GUI Applications Not Showing

**Linux:**
```bash
xhost +local:docker
```

**Windows (WSL2):**
```bash
export DISPLAY=:0
```

### USB Device Access

```bash
# Check device
ls -l /dev/ttyUSB*

# Set permissions
sudo chmod 666 /dev/ttyUSB0

# Add user to dialout group
sudo usermod -aG dialout $USER
```

### Container Commands

```bash
# View logs
docker-compose logs -f

# Restart services
docker-compose restart

# Clean rebuild
docker-compose down
docker-compose build --no-cache
docker-compose up -d
```

## Testing

### Test Joint Movement
```bash
docker-compose exec so100-arm ros2 topic pub /so_100_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{
  joint_names: ["Shoulder_Rotation", "Shoulder_Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll"],
  points: [{
    positions: [0.0, 0.0, 0.0, 0.0, 0.0],
    time_from_start: {sec: 2}
  }]
}'
```

### Monitor Topics
```bash
docker-compose exec so100-arm ros2 topic list
docker-compose exec so100-arm ros2 topic echo /joint_states
```

## Architecture

```
┌─────────────────────────────────────┐
│         Docker Container            │
│                                     │
│  ┌─────────────────────────────┐   │
│  │     ROS2 Humble Core        │   │
│  └─────────────────────────────┘   │
│                                     │
│  ┌─────────────────────────────┐   │
│  │      MoveIt2 Planning       │   │
│  └─────────────────────────────┘   │
│                                     │
│  ┌─────────────────────────────┐   │
│  │    SO-100 Arm Package       │   │
│  └─────────────────────────────┘   │
│                                     │
│  ┌─────────────────────────────┐   │
│  │    Star Tracker Node        │   │
│  └─────────────────────────────┘   │
└─────────────────────────────────────┘
            │           │
            ▼           ▼
    [USB Serial]   [PiCamera]
         │              │
    [Robot Arm]    [Star View]
```

## Resources

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [MoveIt2 Documentation](https://moveit.ros.org/)
- [Docker ROS Guidelines](https://hub.docker.com/_/ros/)