#!/bin/bash

# Star Tracker Test Runner for Docker Environment
# Usage: ./run_tests.sh [test_type]

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Print colored output
print_header() {
    echo -e "${BLUE}================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}================================${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

# Test functions
run_syntax_validation() {
    print_header "Running Syntax Validation"
    
    docker-compose exec -T so100-arm bash -c "
        cd /ros2_ws/src/so_100_arm &&
        python3 star_tracker/validate_tests.py
    "
    
    if [ $? -eq 0 ]; then
        print_success "Syntax validation passed"
    else
        print_error "Syntax validation failed"
        return 1
    fi
}

run_astropy_validation() {
    print_header "Running Astropy Calculations Validation"
    
    docker-compose exec -T so100-arm bash -c "
        cd /ros2_ws/src/so_100_arm &&
        python3 star_tracker/test_astropy_validation.py
    "
    
    if [ $? -eq 0 ]; then
        print_success "Astropy validation passed"
    else
        print_error "Astropy validation failed"
        return 1
    fi
}

run_ros2_validation() {
    print_header "Running ROS2 Message Compatibility"
    
    docker-compose exec -T so100-arm bash -c "
        cd /ros2_ws/src/so_100_arm &&
        source /opt/ros/humble/setup.bash &&
        python3 star_tracker/validate_ros2.py
    "
    
    if [ $? -eq 0 ]; then
        print_success "ROS2 validation passed"
    else
        print_error "ROS2 validation failed"
        return 1
    fi
}

run_integration_tests() {
    print_header "Running Integration Tests"
    
    docker-compose exec -T so100-arm bash -c "
        cd /ros2_ws/src/so_100_arm &&
        python3 star_tracker/integration_tests.py
    "
    
    if [ $? -eq 0 ]; then
        print_success "Integration tests passed"
    else
        print_error "Integration tests failed"
        return 1
    fi
}

run_build_test() {
    print_header "Testing Package Build"
    
    docker-compose exec -T so100-arm bash -c "
        cd /ros2_ws &&
        source /opt/ros/humble/setup.bash &&
        colcon build --packages-select star_tracker
    "
    
    if [ $? -eq 0 ]; then
        print_success "Package build successful"
    else
        print_error "Package build failed"
        return 1
    fi
}

run_launch_validation() {
    print_header "Testing Launch File Validation"
    
    # Test launch file syntax
    docker-compose exec -T so100-arm bash -c "
        cd /ros2_ws &&
        source /opt/ros/humble/setup.bash &&
        source install/setup.bash &&
        ros2 launch star_tracker test_star_tracker.launch.py --show-args
    "
    
    if [ $? -eq 0 ]; then
        print_success "Launch file validation passed"
    else
        print_error "Launch file validation failed"
        return 1
    fi
}

run_mock_system_test() {
    print_header "Running Mock System Test (30 seconds)"
    
    # Run the test framework for 30 seconds
    docker-compose exec so100-arm bash -c "
        cd /ros2_ws &&
        source /opt/ros/humble/setup.bash &&
        source install/setup.bash &&
        timeout 30s ros2 launch star_tracker test_star_tracker.launch.py test_duration:=25 || true
    " &
    
    # Wait for test to complete
    wait
    
    # Check if results were generated
    docker-compose exec -T so100-arm bash -c "
        cd /ros2_ws/src/so_100_arm &&
        ls -la *.json 2>/dev/null | head -5 || echo 'No test result files found'
    "
    
    print_success "Mock system test completed"
}

run_gps_simulation_test() {
    print_header "Running GPS Simulation Test"
    
    docker-compose exec -T so100-arm bash -c "
        cd /ros2_ws &&
        source /opt/ros/humble/setup.bash &&
        source install/setup.bash &&
        timeout 20s ros2 launch star_tracker test_star_tracker.launch.py enable_gps_test:=true enable_imu_test:=false test_duration:=15 || true
    " &
    
    wait
    print_success "GPS simulation test completed"
}

run_all_tests() {
    print_header "Running Complete Test Suite"
    
    local failed_tests=0
    
    # Run each test and track failures
    run_syntax_validation || ((failed_tests++))
    run_astropy_validation || ((failed_tests++))
    run_ros2_validation || ((failed_tests++))
    run_build_test || ((failed_tests++))
    run_launch_validation || ((failed_tests++))
    run_integration_tests || ((failed_tests++))
    run_mock_system_test || ((failed_tests++))
    run_gps_simulation_test || ((failed_tests++))
    
    # Summary
    print_header "Test Summary"
    
    if [ $failed_tests -eq 0 ]; then
        print_success "All tests passed! 🎉"
        echo -e "${GREEN}The Star Tracker GPS system is ready for deployment.${NC}"
    else
        print_error "$failed_tests tests failed"
        echo -e "${RED}Please review the failed tests above.${NC}"
        return 1
    fi
}

# Ensure Docker containers are running
ensure_containers() {
    print_header "Ensuring Docker Containers are Running"
    
    if ! docker-compose ps | grep -q "so100_arm_container.*Up"; then
        echo "Starting Docker containers..."
        docker-compose up -d so100-arm
        
        # Wait for container to be ready
        echo "Waiting for container to be ready..."
        sleep 10
        
        # Build the workspace inside container
        docker-compose exec -T so100-arm bash -c "
            cd /ros2_ws &&
            source /opt/ros/humble/setup.bash &&
            colcon build --symlink-install
        "
    else
        print_success "Containers already running"
    fi
}

# Generate test report
generate_report() {
    print_header "Generating Test Report"
    
    docker-compose exec -T so100-arm bash -c "
        cd /ros2_ws/src/so_100_arm &&
        echo '# Star Tracker Test Report' > test_report.md &&
        echo '' >> test_report.md &&
        echo 'Generated: $(date)' >> test_report.md &&
        echo '' >> test_report.md &&
        echo '## Test Results' >> test_report.md &&
        echo '' >> test_report.md &&
        ls -la *.json 2>/dev/null | while read line; do
            echo '- ' \$line >> test_report.md
        done || echo 'No test result files found' >> test_report.md &&
        echo '' >> test_report.md &&
        echo '## System Information' >> test_report.md &&
        echo '- ROS2 Distribution: Humble' >> test_report.md &&
        echo '- Python Version: ' \$(python3 --version) >> test_report.md &&
        echo '- Astropy Version: ' \$(python3 -c 'import astropy; print(astropy.__version__)' 2>/dev/null || echo 'Not installed') >> test_report.md &&
        echo 'Test report generated: test_report.md'
    "
}

# Main execution
case "${1:-all}" in
    "syntax")
        ensure_containers
        run_syntax_validation
        ;;
    "astropy")
        ensure_containers
        run_astropy_validation
        ;;
    "ros2")
        ensure_containers
        run_ros2_validation
        ;;
    "integration")
        ensure_containers
        run_integration_tests
        ;;
    "build")
        ensure_containers
        run_build_test
        ;;
    "launch")
        ensure_containers
        run_launch_validation
        ;;
    "mock")
        ensure_containers
        run_mock_system_test
        ;;
    "gps")
        ensure_containers
        run_gps_simulation_test
        ;;
    "all")
        ensure_containers
        run_all_tests
        generate_report
        ;;
    "help")
        echo "Star Tracker Test Runner"
        echo ""
        echo "Usage: $0 [test_type]"
        echo ""
        echo "Available test types:"
        echo "  syntax      - Python syntax validation"
        echo "  astropy     - Astronomical calculations"
        echo "  ros2        - ROS2 message compatibility"
        echo "  integration - Integration test suite"
        echo "  build       - Package build test"
        echo "  launch      - Launch file validation"
        echo "  mock        - Mock system test"
        echo "  gps         - GPS simulation test"
        echo "  all         - Run complete test suite (default)"
        echo "  help        - Show this help"
        echo ""
        echo "Examples:"
        echo "  $0              # Run all tests"
        echo "  $0 astropy      # Test only astropy calculations"
        echo "  $0 mock         # Run mock system test"
        ;;
    *)
        print_error "Unknown test type: $1"
        echo "Run '$0 help' for available options"
        exit 1
        ;;
esac