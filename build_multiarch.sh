#!/bin/bash

# Build multi-architecture Docker images for SO-100 Robot Arm
# Supports AMD64 and ARM64 (Raspberry Pi 5)

set -e

echo "=============================================="
echo "Building Multi-Architecture Docker Images"
echo "=============================================="
echo "Platforms: linux/amd64, linux/arm64/v8"
echo "=============================================="

# Setup buildx if needed
./setup_buildx.sh

# Parse command line arguments
BUILD_PUSH=false
TAG_PREFIX="so100-arm"
REGISTRY=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --push)
            BUILD_PUSH=true
            shift
            ;;
        --registry)
            REGISTRY="$2/"
            shift 2
            ;;
        --tag)
            TAG_PREFIX="$2"
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--push] [--registry REGISTRY] [--tag TAG_PREFIX]"
            exit 1
            ;;
    esac
done

# Build configurations
TIMESTAMP=$(date +%Y%m%d-%H%M%S)
PLATFORMS="linux/amd64,linux/arm64/v8"

echo ""
echo "Build Configuration:"
echo "  Registry: ${REGISTRY:-local}"
echo "  Tag prefix: $TAG_PREFIX"
echo "  Push to registry: $BUILD_PUSH"
echo "  Timestamp: $TIMESTAMP"
echo ""

# Function to build image
build_image() {
    local dockerfile=$1
    local tag=$2
    local platforms=${3:-$PLATFORMS}
    
    echo "Building $tag for platforms: $platforms"
    
    if [ "$BUILD_PUSH" = true ]; then
        docker buildx build \
            --platform $platforms \
            -f $dockerfile \
            -t ${REGISTRY}${tag}:latest \
            -t ${REGISTRY}${tag}:${TIMESTAMP} \
            --push \
            --cache-from type=registry,ref=${REGISTRY}${tag}:buildcache \
            --cache-to type=registry,ref=${REGISTRY}${tag}:buildcache,mode=max \
            .
    else
        docker buildx build \
            --platform $platforms \
            -f $dockerfile \
            -t ${tag}:latest \
            -t ${tag}:${TIMESTAMP} \
            --load \
            .
    fi
}

# Build main multi-arch image
echo "1. Building main multi-architecture image..."
build_image "Dockerfile.multiarch" "${TAG_PREFIX}"

# Build ARM64-specific image (optimized for Raspberry Pi 5)
echo ""
echo "2. Building ARM64-optimized image for Raspberry Pi 5..."
build_image "Dockerfile.multiarch" "${TAG_PREFIX}-pi5" "linux/arm64/v8"

# Build test image
echo ""
echo "3. Building test image with all platforms..."
build_image "Dockerfile.multiarch" "${TAG_PREFIX}-test"

echo ""
echo "=============================================="
echo "✅ Multi-architecture build complete!"
echo "=============================================="
echo ""
echo "Images built:"
echo "  - ${TAG_PREFIX}:latest (multi-arch)"
echo "  - ${TAG_PREFIX}-pi5:latest (ARM64 only)"
echo "  - ${TAG_PREFIX}-test:latest (multi-arch)"
echo ""

if [ "$BUILD_PUSH" = false ]; then
    echo "To push images to a registry, run with --push flag"
    echo "Example: $0 --push --registry docker.io/yourusername"
fi

echo ""
echo "To deploy on Raspberry Pi 5:"
echo "  1. Pull the image: docker pull ${REGISTRY}${TAG_PREFIX}-pi5:latest"
echo "  2. Run: docker-compose -f docker-compose.pi5.yml up"
echo ""