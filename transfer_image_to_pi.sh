#!/bin/bash

# Transfer Docker image to Raspberry Pi
set -e

echo "========================================"
echo "Docker Image Transfer to Raspberry Pi"
echo "========================================"

# Configuration
PI_HOST="${1:-raspberrypi.local}"
PI_USER="${2:-acidhax}"
IMAGE_NAME="${3:-so100-arm-pi5:latest}"

echo "Target: $PI_USER@$PI_HOST"
echo "Image: $IMAGE_NAME"
echo ""

# Save the ARM64 image
echo "Saving Docker image for ARM64..."
docker save "$IMAGE_NAME" | gzip > so100-arm-pi5.tar.gz

# Check file size
SIZE=$(du -h so100-arm-pi5.tar.gz | cut -f1)
echo "Image size: $SIZE"

# Transfer to Pi
echo ""
echo "Transferring image to Raspberry Pi..."
echo "This may take several minutes depending on network speed..."
scp so100-arm-pi5.tar.gz "$PI_USER@$PI_HOST:~/so100-arm/"

# Load on Pi
echo ""
echo "Loading image on Raspberry Pi..."
ssh "$PI_USER@$PI_HOST" "cd ~/so100-arm && gunzip -c so100-arm-pi5.tar.gz | docker load && rm so100-arm-pi5.tar.gz"

# Clean up local file
rm so100-arm-pi5.tar.gz

echo ""
echo "✅ Image transfer complete!"
echo ""
echo "You can now run on the Pi:"
echo "  cd ~/so100-arm"
echo "  docker-compose up"