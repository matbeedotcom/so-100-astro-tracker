#!/bin/bash
set -e

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# Configure ROS2 environment
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}

echo "========================================="
echo "SO-100 Robot Arm Docker Container"
echo "========================================="
echo "ROS2 Distribution: Humble"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "Robot Mode: ${ROBOT_MODE:-simulation}"
echo "========================================="

# Function to launch robot based on mode
launch_robot() {
    case "${ROBOT_MODE}" in
        "hardware")
            echo "Launching hardware interface..."
            echo "Make sure the robot is connected to ${SERIAL_PORT:-/dev/ttyUSB0}"
            ros2 launch so_100_arm hardware.launch.py
            ;;
        "simulation")
            echo "Launching simulation mode..."
            ros2 launch so_100_arm moveit.launch.py use_fake_hardware:=true
            ;;
        "mock")
            echo "Launching mock hardware mode..."
            ros2 launch so_100_arm moveit.launch.py use_fake_hardware:=true
            ;;
        *)
            echo "Unknown ROBOT_MODE: ${ROBOT_MODE}"
            echo "Valid options: hardware, simulation, mock"
            ;;
    esac
}

# Function to launch star tracker
launch_star_tracker() {
    echo "Starting star tracker node..."
    echo "Target: ${TARGET_OBJECT:-polaris}"
    echo "Location: Lat=${LOCATION_LAT}, Lon=${LOCATION_LON}, Alt=${LOCATION_ALT}"
    
    # Launch star tracker node (to be implemented)
    # ros2 run star_tracker star_tracker_node
}

# If running as main container, provide interactive options
if [ "$1" = "bash" ] || [ -z "$1" ]; then
    echo ""
    echo "Available commands:"
    echo "  launch_robot       - Launch robot based on ROBOT_MODE environment variable"
    echo "  launch_tracker     - Launch star tracker node"
    echo ""
    echo "Quick launch commands:"
    echo "  ros2 launch so_100_arm moveit.launch.py                    # MoveIt with mock hardware"
    echo "  ros2 launch so_100_arm hardware.launch.py                  # Real hardware"
    echo "  ros2 launch so_100_arm gz.launch.py                        # Gazebo simulation"
    echo ""
    echo "Star Tracker commands:"
    echo "  ros2 launch star_tracker star_tracker.launch.py           # Basic star tracking"
    echo "  ros2 launch star_tracker star_tracker_gps.launch.py        # GPS-enhanced tracking"
    echo "  ros2 launch star_tracker test_star_tracker.launch.py       # Automated testing"
    echo ""
    echo "Test commands:"
    echo "  python3 star_tracker/validate_tests.py                     # Standalone validation"
    echo "  python3 star_tracker/test_astropy_validation.py            # Astropy calculations"
    echo "  python3 star_tracker/integration_tests.py                  # Integration tests"
    echo "  ros2 topic list                                            # List all topics"
    echo "  ros2 topic echo /joint_states                              # Monitor joint states"
    echo "  ros2 run so_arm_100_hardware test_servo                    # Test servo communication"
    echo ""
fi

# Execute command passed to docker run
exec "$@"