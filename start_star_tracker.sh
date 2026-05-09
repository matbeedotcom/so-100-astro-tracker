#!/bin/bash

# Star Tracker Auto-Start Script
# This script starts all required interfaces for the star tracker system

echo "Starting Star Tracker System..."

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# Check if we're in Docker
if [ -f /.dockerenv ]; then
    echo "Running in Docker container"
    WORKSPACE_PATH="/ros2_ws"
else
    echo "Running on host system"
    WORKSPACE_PATH="$(pwd)"
fi

# Function to start a process in background and track its PID
start_process() {
    local name=$1
    local command=$2
    echo "Starting $name..."

    # Start the process in background
    eval "$command" &
    local pid=$!

    # Store PID for cleanup
    echo $pid >> /tmp/star_tracker_pids.txt

    # Give it time to start
    sleep 2

    # Check if process is still running
    if kill -0 $pid 2>/dev/null; then
        echo "✅ $name started successfully (PID: $pid)"
    else
        echo "❌ $name failed to start"
        return 1
    fi
}

# Function to check if a topic exists
wait_for_topic() {
    local topic=$1
    local timeout=${2:-10}
    local count=0

    echo "Waiting for topic $topic..."
    while [ $count -lt $timeout ]; do
        if ros2 topic list | grep -q "$topic"; then
            echo "✅ Topic $topic is available"
            return 0
        fi
        sleep 1
        ((count++))
    done

    echo "⚠️  Topic $topic not found after ${timeout}s"
    return 1
}

# Create PID tracking file
echo "# Star Tracker PIDs - $(date)" > /tmp/star_tracker_pids.txt

# 1. Start BNO055 IMU Interface
start_process "BNO055 IMU Interface" \
    "ros2 run star_tracker bno055_interface --ros-args -p i2c_bus:=1 -p i2c_address:=0x28"

# Wait for IMU topics to be available
wait_for_topic "/imu/euler" 15
wait_for_topic "/imu/data" 5
wait_for_topic "/imu/magnetometer" 5
wait_for_topic "/imu/accelerometer" 5

# 2. Start Hardware Interface (if robot is connected)
echo "Checking for robot hardware..."
if [ -e /dev/ttyUSB0 ]; then
    echo "Robot hardware detected on /dev/ttyUSB0"
    chmod 666 /dev/ttyUSB0 2>/dev/null

    start_process "Robot Hardware Interface" \
        "ros2 launch so_100_arm hardware.launch.py"

    # Wait for joint states
    wait_for_topic "/joint_states" 10
else
    echo "⚠️  No robot hardware detected (/dev/ttyUSB0 not found)"
    echo "Starting with simulation mode..."

    start_process "Robot Simulation" \
        "ros2 launch so_100_arm moveit.launch.py use_fake_hardware:=true"

    wait_for_topic "/joint_states" 10
fi

# 3. Start GPS Interface (if GPS device is available)
echo "Checking for GPS hardware..."
if [ -e /dev/ttyAMA0 ] || [ -e /dev/ttyUSB1 ]; then
    GPS_DEVICE="/dev/ttyAMA0"
    [ -e /dev/ttyUSB1 ] && GPS_DEVICE="/dev/ttyUSB1"

    echo "GPS hardware detected on $GPS_DEVICE"
    chmod 666 $GPS_DEVICE 2>/dev/null

    start_process "GPS Interface" \
        "ros2 run star_tracker gps_interface --ros-args -p serial_port:=$GPS_DEVICE"

    wait_for_topic "/gps/fix" 10
else
    echo "⚠️  No GPS hardware detected - using fallback location"
fi

# 4. Start Star Tracker with auto-calibration
echo ""
echo "🚀 Starting Star Tracker with Automatic Calibration..."
echo "Available parameters:"
echo "  --target moon|sun|polaris|sirius"
echo "  --declination <magnetic_declination_for_your_location>"
echo "  --lat <latitude> --lon <longitude> --alt <altitude>"
echo ""

# Parse command line arguments
TARGET_OBJECT="moon"
MAGNETIC_DECLINATION="0.0"
LATITUDE="40.7128"
LONGITUDE="-74.0060"
ALTITUDE="10.0"

while [[ $# -gt 0 ]]; do
    case $1 in
        --target)
            TARGET_OBJECT="$2"
            shift 2
            ;;
        --declination)
            MAGNETIC_DECLINATION="$2"
            shift 2
            ;;
        --lat)
            LATITUDE="$2"
            shift 2
            ;;
        --lon)
            LONGITUDE="$2"
            shift 2
            ;;
        --alt)
            ALTITUDE="$2"
            shift 2
            ;;
        *)
            echo "Unknown parameter: $1"
            shift
            ;;
    esac
done

echo "Configuration:"
echo "  Target Object: $TARGET_OBJECT"
echo "  Magnetic Declination: ${MAGNETIC_DECLINATION}°"
echo "  Location: ${LATITUDE}°N, ${LONGITUDE}°E, ${ALTITUDE}m"
echo ""

# Start the main star tracker
start_process "Star Tracker Node" \
    "ros2 run star_tracker star_tracker_node --ros-args \
     -p use_imu:=true \
     -p goto_mode:=true \
     -p target_object:=$TARGET_OBJECT \
     -p magnetic_declination:=$MAGNETIC_DECLINATION \
     -p location_lat:=$LATITUDE \
     -p location_lon:=$LONGITUDE \
     -p location_alt:=$ALTITUDE"

echo ""
echo "🎯 Star Tracker System Started!"
echo ""
echo "📊 System Status:"
ros2 topic list | grep -E "(imu|joint|gps|star)" | sed 's/^/  ✅ /'
echo ""
echo "📝 Logs:"
echo "  Monitor with: ros2 topic echo /rosout"
echo "  IMU data: ros2 topic echo /imu/euler"
echo "  Joint states: ros2 topic echo /joint_states"
echo ""
echo "🛑 To stop all processes:"
echo "  $0 --stop"
echo "  or: kill \$(cat /tmp/star_tracker_pids.txt)"
echo ""
echo "Press Ctrl+C to stop or run in background with: $0 &"

# Wait for interrupt or keep running
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    # Script was executed directly, wait for interrupt
    trap 'echo ""; echo "Stopping Star Tracker System..."; kill $(cat /tmp/star_tracker_pids.txt 2>/dev/null) 2>/dev/null; rm -f /tmp/star_tracker_pids.txt; exit 0' SIGINT SIGTERM

    echo "System running... Press Ctrl+C to stop"
    while true; do
        sleep 1
    done
fi