#!/bin/bash

# Setup Docker buildx for multi-architecture builds
# Supports AMD64 and ARM64 (Raspberry Pi 5)

echo "=========================================="
echo "Setting up Docker Buildx for Multi-Arch"
echo "=========================================="

# Check if buildx is available
if ! docker buildx version > /dev/null 2>&1; then
    echo "❌ Docker buildx not found. Please update Docker."
    exit 1
fi

# Create a new builder instance for multi-arch builds
BUILDER_NAME="so100-multiarch-builder"

# Check if builder already exists
if docker buildx ls | grep -q "$BUILDER_NAME"; then
    echo "✓ Builder '$BUILDER_NAME' already exists"
    docker buildx use $BUILDER_NAME
else
    echo "Creating new buildx builder: $BUILDER_NAME"
    docker buildx create --name $BUILDER_NAME \
        --driver docker-container \
        --platform linux/amd64,linux/arm64/v8 \
        --use
fi

# Bootstrap the builder
echo "Bootstrapping builder..."
docker buildx inspect --bootstrap

# Show current builder
echo ""
echo "Current builder configuration:"
docker buildx inspect

echo ""
echo "✅ Buildx setup complete!"
echo ""
echo "Available platforms:"
echo "  - linux/amd64 (x86_64)"
echo "  - linux/arm64/v8 (ARM64 - Raspberry Pi 5)"
echo ""
echo "=========================================="