#!/bin/bash

echo "Starting SO-100 Robot Arm on Raspberry Pi 5..."

# Pull latest image (if using registry)
# docker pull so100-arm-pi5:latest

# Start services
docker-compose up -d

# Show logs
docker-compose logs -f
