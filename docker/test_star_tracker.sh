#!/bin/bash

# Test script to run star tracker with GPS/IMU emulation in Docker
set -e

echo "=============================================="
echo "Star Tracker Test: Moon Tracking from Toronto"
echo "=============================================="

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Navigate to workspace
cd /home/ros/ros2_ws

# Copy source files if needed (when running in Docker with volumes)
if [ ! -d "src/star_tracker" ]; then
    echo "Copying star tracker source..."
    cp -r /home/ros/ros2_ws/src/so_100_arm/star_tracker src/
fi

# Build the star_tracker package
echo "Building star_tracker package..."
colcon build --packages-select star_tracker --symlink-install

# Source the workspace
source install/setup.bash

# Create results directory
mkdir -p /tmp/test_results

# Launch the test with multiple nodes in background
echo "Starting mock providers..."

# Start GPS provider (Toronto location)
ros2 run star_tracker mock_providers gps &
GPS_PID=$!

# Start IMU provider
ros2 run star_tracker mock_providers imu &
IMU_PID=$!

# Start arm emulator
ros2 run star_tracker mock_providers arm &
ARM_PID=$!

# Wait for nodes to initialize
sleep 5

# Start star tracker node
echo "Starting star tracker for moon tracking..."
timeout 60 ros2 run star_tracker star_tracker_node --ros-args \
    -p location_lat:=43.6532 \
    -p location_lon:=-79.3832 \
    -p location_alt:=76.0 \
    -p target_object:=moon \
    -p use_gps:=true \
    -p use_imu:=true \
    -p goto_mode:=true \
    -p update_rate:=2.0 \
    -p verbose_logging:=true &
TRACKER_PID=$!

# Monitor for 60 seconds
echo "Running test for 60 seconds..."
sleep 60

# Stop all nodes
echo "Stopping test nodes..."
kill $GPS_PID $IMU_PID $ARM_PID $TRACKER_PID 2>/dev/null || true

# Save any logs
echo "Test complete!"
echo ""
echo "Summary:"
echo "- Location: Toronto (43.65°N, 79.38°W)"
echo "- Target: Moon"
echo "- GPS: Emulated"
echo "- IMU: Emulated (BNO055)"
echo "- GoTo Mode: Enabled"

# Copy results if they exist
if [ -f "/tmp/toronto_moon_tracking.log" ]; then
    cp /tmp/toronto_moon_tracking.log /tmp/test_results/
    echo "- Tracking log saved"
fi

echo ""
echo "Test completed successfully!"