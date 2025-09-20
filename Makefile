.PHONY: help build up down shell logs clean rebuild test

help: ## Show this help message
	@echo "SO-100 Robot Arm Docker Commands"
	@echo "================================="
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-15s\033[0m %s\n", $$1, $$2}'

build: ## Build Docker image
	docker-compose build

up: ## Start containers in background
	docker-compose up -d

down: ## Stop and remove containers
	docker-compose down

shell: ## Enter the main container
	docker-compose exec so100-arm bash

logs: ## Show container logs
	docker-compose logs -f

clean: ## Clean up containers, volumes, and images
	docker-compose down -v
	docker rmi so100-arm:humble || true

rebuild: clean build ## Clean rebuild of everything

test: ## Run basic tests
	docker-compose exec so100-arm bash -c "ros2 topic list"
	docker-compose exec so100-arm bash -c "ros2 node list"

# Simulation commands
sim: ## Launch simulation with MoveIt
	docker-compose exec so100-arm bash -c "source /ros2_ws/install/setup.bash && ros2 launch so_100_arm moveit.launch.py"

gazebo: ## Launch Gazebo simulation
	docker-compose exec so100-arm bash -c "source /ros2_ws/install/setup.bash && ros2 launch so_100_arm gz.launch.py"

rviz: ## Launch RViz visualization
	docker-compose exec so100-arm bash -c "source /ros2_ws/install/setup.bash && ros2 launch so_100_arm rviz.launch.py"

# Hardware commands
hardware: ## Launch hardware interface (physical robot)
	docker-compose exec so100-arm bash -c "source /ros2_ws/install/setup.bash && ros2 launch so_100_arm hardware.launch.py"

test-servo: ## Test servo communication
	docker-compose exec so100-arm bash -c "source /ros2_ws/install/setup.bash && ros2 run so_arm_100_hardware test_servo"

# Star tracker commands
tracker: ## Launch star tracker
	docker-compose exec so100-arm bash -c "source /ros2_ws/install/setup.bash && ros2 launch star_tracker star_tracker.launch.py"

tracker-service: ## Start star tracker service
	docker-compose --profile star-tracking up -d star-tracker

# Development commands
colcon-build: ## Build ROS packages inside container
	docker-compose exec so100-arm bash -c "cd /ros2_ws && colcon build --symlink-install"

colcon-clean: ## Clean build artifacts
	docker-compose exec so100-arm bash -c "cd /ros2_ws && rm -rf build install log"

# Monitoring commands
topics: ## List ROS2 topics
	docker-compose exec so100-arm bash -c "ros2 topic list"

nodes: ## List ROS2 nodes
	docker-compose exec so100-arm bash -c "ros2 node list"

joint-states: ## Monitor joint states
	docker-compose exec so100-arm bash -c "ros2 topic echo /joint_states"

# Quick launch combinations
demo: up ## Start containers and launch demo
	@sleep 3
	@make sim

dev: up shell ## Start containers and enter shell for development

tracker-demo: up ## Start containers with star tracker
	@sleep 3
	docker-compose exec -d so100-arm bash -c "source /ros2_ws/install/setup.bash && ros2 launch so_100_arm moveit.launch.py"
	@sleep 5
	docker-compose exec so100-arm bash -c "source /ros2_ws/install/setup.bash && ros2 launch star_tracker star_tracker.launch.py"