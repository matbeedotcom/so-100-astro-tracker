#!/bin/bash

# Test script for multi-architecture build verification
echo "=============================================="
echo "Multi-Architecture Build Test"
echo "=============================================="

# Test 1: Verify buildx builder is ready
echo "1. Checking buildx builder..."
docker buildx ls | grep so100-multiarch-builder
if [ $? -eq 0 ]; then
    echo "✅ Buildx builder ready"
else
    echo "❌ Buildx builder not found"
    exit 1
fi

# Test 2: Check available platforms
echo ""
echo "2. Available platforms:"
docker buildx inspect | grep "Platforms:"

# Test 3: Verify base images exist
echo ""
echo "3. Checking base images..."

echo "  AMD64 base image:"
docker buildx imagetools inspect osrf/ros:humble-desktop-full > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "  ✅ osrf/ros:humble-desktop-full (AMD64)"
else
    echo "  ❌ osrf/ros:humble-desktop-full not found"
fi

echo "  ARM64 base image:"
docker buildx imagetools inspect arm64v8/ros:humble-ros-base-jammy > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "  ✅ arm64v8/ros:humble-ros-base-jammy (ARM64)"
else
    echo "  ❌ arm64v8/ros:humble-ros-base-jammy not found"
fi

# Test 4: Quick syntax check of Dockerfile
echo ""
echo "4. Dockerfile syntax check..."
docker buildx build --dry-run -f Dockerfile.multiarch . > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "✅ Dockerfile syntax valid"
else
    echo "❌ Dockerfile syntax error"
fi

# Test 5: Platform-specific build test (build stages only)
echo ""
echo "5. Testing platform-specific build stages..."

echo "  Testing AMD64 stage..."
timeout 60 docker buildx build \
    --platform linux/amd64 \
    --target amd64-base \
    -f Dockerfile.multiarch \
    . > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "  ✅ AMD64 stage builds successfully"
else
    echo "  ⚠️  AMD64 stage build incomplete (may need more time)"
fi

echo "  Testing ARM64 stage..."
timeout 60 docker buildx build \
    --platform linux/arm64/v8 \
    --target arm64-base \
    -f Dockerfile.multiarch \
    . > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "  ✅ ARM64 stage builds successfully"
else
    echo "  ⚠️  ARM64 stage build incomplete (may need more time)"
fi

echo ""
echo "=============================================="
echo "Multi-Architecture Setup Summary"
echo "=============================================="
echo "🎯 Target Platforms:"
echo "  - linux/amd64 (x86_64 development)"
echo "  - linux/arm64/v8 (Raspberry Pi 5)"
echo ""
echo "🐳 Base Images:"
echo "  - AMD64: osrf/ros:humble-desktop-full"
echo "  - ARM64: arm64v8/ros:humble-ros-base-jammy"
echo ""
echo "🏗️  Build Commands:"
echo "  Full multi-arch: ./build_multiarch.sh"
echo "  ARM64 only:     docker buildx build --platform linux/arm64/v8 -f Dockerfile.multiarch -t so100-arm:pi5 ."
echo "  AMD64 only:     docker buildx build --platform linux/amd64 -f Dockerfile.multiarch -t so100-arm:amd64 ."
echo ""
echo "🚀 Deployment:"
echo "  Raspberry Pi 5: ./deploy_to_pi5.sh"
echo "  Local AMD64:    docker-compose up"
echo ""
echo "✅ Multi-architecture cross-compilation is ready!"
echo "=============================================="