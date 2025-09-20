# Star Tracker Docker Testing Commands

## Quick Test Commands

### 1. **Start the Container and Run All Tests**
```bash
# Build and start containers
docker-compose up -d so100-arm

# Run complete test suite
./run_tests.sh

# Or run specific tests
./run_tests.sh astropy
./run_tests.sh integration
./run_tests.sh mock
```

### 2. **Interactive Testing Session**
```bash
# Start interactive session
docker-compose exec so100-arm bash

# Inside container, run individual tests:
cd /ros2_ws/src/so_100_arm

# Standalone validations (no ROS2 required)
python3 star_tracker/validate_tests.py
python3 star_tracker/test_astropy_validation.py
python3 star_tracker/validate_ros2.py
python3 star_tracker/integration_tests.py

# Build the package
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select star_tracker
source install/setup.bash

# Test launch files
ros2 launch star_tracker test_star_tracker.launch.py --show-args

# Run automated tests
ros2 launch star_tracker test_star_tracker.launch.py test_duration:=30
```

### 3. **Specific Test Scenarios**
```bash
# Test GPS simulation only
./run_tests.sh gps

# Test astropy calculations
./run_tests.sh astropy

# Test package build
./run_tests.sh build

# Test ROS2 compatibility
./run_tests.sh ros2
```

## Expected Test Results

### ✅ **Syntax Validation**
- All Python files compile without errors
- Import statements resolve correctly
- No syntax errors detected

### ✅ **Astropy Calculations**
```
Sun position (Summer Solstice, NYC, Noon UTC):
  Altitude: 72.7° (expected ~72.7°)
  Error:    0.1°
  ✓ Sun altitude within expected range

Polaris position:
  Altitude: 40.7°
  Expected: ~40.7° (latitude)
  Error:    0.02°
  ✓ Polaris altitude matches latitude
```

### ✅ **ROS2 Message Compatibility**
```
✓ sensor_msgs.msg.NavSatFix - GPS position data
✓ sensor_msgs.msg.TimeReference - GPS time synchronization
✓ trajectory_msgs.msg.JointTrajectory - Robot trajectory commands
✓ control_msgs.action.FollowJointTrajectory - Trajectory execution action

ROS2 COMPATIBILITY: ✓ COMPATIBLE
```

### ✅ **Integration Tests**
```
Tests run: 8
Failures: 0
Errors: 0
Success rate: 100.0%
```

### ✅ **Mock System Test**
Expected output when running the mock system:
```
[INFO] Mock GPS initialized at 40.712800, -74.006000
[INFO] Mock IMU initialized
[INFO] Emulated SO-100 arm initialized
[INFO] GPS fix acquired
[INFO] GPS location updated: Lat=40.712800, Lon=-74.006000, Alt=10.0m
[INFO] Tracking moon: Alt=45.2°, Az=180.3°
[INFO] Moving to positions: [180.3, -44.8, 0.0, 44.8, 0.0]
```

## Troubleshooting

### **Container Won't Start**
```bash
# Check container status
docker-compose ps

# View logs
docker-compose logs so100-arm

# Rebuild if needed
docker-compose build --no-cache so100-arm
```

### **Package Build Fails**
```bash
# Clean build
docker-compose exec so100-arm bash -c "cd /ros2_ws && rm -rf build install log && colcon build"

# Check dependencies
docker-compose exec so100-arm bash -c "rosdep check --from-paths src --ignore-src"
```

### **Import Errors**
```bash
# Check Python packages
docker-compose exec so100-arm python3 -c "import astropy; print('Astropy:', astropy.__version__)"
docker-compose exec so100-arm python3 -c "import scipy; print('Scipy:', scipy.__version__)"
docker-compose exec so100-arm python3 -c "import numpy; print('Numpy:', numpy.__version__)"
```

### **ROS2 Launch Fails**
```bash
# Check ROS2 environment
docker-compose exec so100-arm bash -c "source /opt/ros/humble/setup.bash && env | grep ROS"

# List available packages
docker-compose exec so100-arm bash -c "source /opt/ros/humble/setup.bash && ros2 pkg list | grep star"
```

## Test File Locations

After running tests, look for these output files:
```
validation_results.json          # Syntax and basic validation
integration_test_results.json    # Full integration results
test_results.json               # ROS2 system test data
test_report.md                  # Generated test summary
```

## Continuous Integration

For CI/CD pipelines:
```bash
# Headless test execution
DISPLAY="" ./run_tests.sh all

# Exit code indicates success/failure
echo $?  # 0 = success, 1 = failure
```

## Performance Expectations

**Test Duration:**
- Syntax validation: ~5 seconds
- Astropy calculations: ~10 seconds  
- ROS2 compatibility: ~15 seconds
- Integration tests: ~30 seconds
- Mock system test: ~30 seconds
- Complete suite: ~2-3 minutes

**Resource Usage:**
- Memory: ~1GB peak
- CPU: Normal load
- Disk: ~100MB for test results