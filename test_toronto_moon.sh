#!/bin/bash

# Test script for Toronto moon tracking with GPS/IMU emulation in Docker

echo "=============================================="
echo "Star Tracker Test: Moon Tracking from Toronto"
echo "=============================================="
echo ""
echo "Configuration:"
echo "  Location: Toronto, Canada (43.65°N, 79.38°W)"
echo "  Target: Moon"
echo "  GPS: Enabled (emulated)"
echo "  IMU: Enabled (BNO055 emulation)"
echo "  GoTo Mode: Enabled"
echo "  Test Duration: 5 minutes"
echo ""

# Create results directory
mkdir -p test_results

# Clean up any previous test containers
echo "Cleaning up previous test containers..."
docker-compose -f docker-compose.test.yml down 2>/dev/null

# Build the Docker image if needed
echo "Building Docker image..."
docker-compose -f docker-compose.test.yml build star_tracker_test

# Run the test
echo "Starting test environment..."
docker-compose -f docker-compose.test.yml up star_tracker_test

# Check if we want to run with visualization
if [ "$1" == "--viz" ]; then
    echo "Starting RViz visualization..."
    xhost +local:docker 2>/dev/null || true
    docker-compose -f docker-compose.test.yml --profile visualization up rviz_viewer &
fi

# Display results
echo ""
echo "=============================================="
echo "Test Complete!"
echo "=============================================="
echo "Results saved to ./test_results/"
echo ""

if [ -f "test_results/toronto_test_results.json" ]; then
    echo "Test Summary:"
    cat test_results/toronto_test_results.json 2>/dev/null | head -20
fi

echo ""
echo "Tracking log: test_results/toronto_moon_tracking.log"

# Cleanup
docker-compose -f docker-compose.test.yml down