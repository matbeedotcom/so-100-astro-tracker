#!/bin/bash

# Deployment script for Raspberry Pi 5
# Deploys SO-100 Robot Arm with Star Tracker to Pi 5

set -e

echo "================================================"
echo "SO-100 Robot Arm - Raspberry Pi 5 Deployment"
echo "================================================"

# Configuration
PI_HOST="${PI_HOST:-raspberrypi.local}"
PI_USER="${PI_USER:-acidhax}"
PI_PORT="${PI_PORT:-22}"
DEPLOY_DIR="/home/$PI_USER/so100-arm"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --host)
            PI_HOST="$2"
            shift 2
            ;;
        --user)
            PI_USER="$2"
            shift 2
            ;;
        --port)
            PI_PORT="$2"
            shift 2
            ;;
        --build)
            BUILD_ON_HOST=true
            shift
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--host HOST] [--user USER] [--port PORT] [--build]"
            exit 1
            ;;
    esac
done

echo "Target: $PI_USER@$PI_HOST:$PI_PORT"
echo "Deploy directory: $DEPLOY_DIR"
echo ""

# Check SSH connection
echo "Checking SSH connection..."
if ! ssh -p $PI_PORT $PI_USER@$PI_HOST "echo 'SSH connection successful'"; then
    echo "❌ Failed to connect to Raspberry Pi"
    echo "Please ensure:"
    echo "  1. Pi is powered on and connected to network"
    echo "  2. SSH is enabled (sudo raspi-config)"
    echo "  3. Correct hostname/IP: $PI_HOST"
    exit 1
fi

# Build multi-arch image if requested
if [ "$BUILD_ON_HOST" = true ]; then
    echo ""
    echo "Building ARM64 image on host..."
    ./build_multiarch.sh --tag so100-arm-pi5
fi

# Prepare deployment package
echo ""
echo "Preparing deployment package..."
# Create temp directory (Windows Git Bash compatible)
if command -v mktemp >/dev/null 2>&1; then
    TEMP_DIR=$(mktemp -d)
else
    # Fallback for Windows Git Bash
    TEMP_DIR="/tmp/so100_deploy_$$_$RANDOM"
    mkdir -p "$TEMP_DIR"
fi
cp -r docker-compose.pi5.yml $TEMP_DIR/docker-compose.yml
cp -r config $TEMP_DIR/
cp -r star_tracker $TEMP_DIR/
cp -r so_100_arm $TEMP_DIR/
mkdir -p $TEMP_DIR/data
mkdir -p $TEMP_DIR/calibration

# Create setup script for Pi
cat > $TEMP_DIR/setup_pi5.sh << 'EOF'
#!/bin/bash

echo "Setting up SO-100 Robot on Raspberry Pi 5..."

# Update system
echo "Updating system packages..."
sudo apt-get update
sudo apt-get upgrade -y

# Install Docker if not present
if ! command -v docker &> /dev/null; then
    echo "Installing Docker..."
    curl -fsSL https://get.docker.com | sh
    sudo usermod -aG docker $USER
    echo "Please log out and back in for Docker group to take effect"
fi

# Install Docker Compose
if ! command -v docker-compose &> /dev/null; then
    echo "Installing Docker Compose..."
    sudo apt-get install -y docker-compose
fi

# Enable required interfaces
echo "Enabling hardware interfaces..."
sudo raspi-config nonint do_i2c 0  # Enable I2C
sudo raspi-config nonint do_spi 0  # Enable SPI
sudo raspi-config nonint do_serial_hw 0  # Enable hardware serial
sudo raspi-config nonint do_serial_cons 1  # Disable serial console

# Install required system packages
echo "Installing system dependencies..."
sudo apt-get install -y \
    i2c-tools \
    python3-smbus \
    python3-pip \
    git \
    pigpio \
    python3-pigpio

# Enable pigpio daemon
sudo systemctl enable pigpiod
sudo systemctl start pigpiod

# Add user to required groups
sudo usermod -aG i2c,spi,gpio,dialout $USER

# Create systemd service for auto-start
sudo tee /etc/systemd/system/so100-robot.service << 'SERVICE'
[Unit]
Description=SO-100 Robot Arm with Star Tracker
After=docker.service
Requires=docker.service

[Service]
Type=simple
Restart=always
RestartSec=10
WorkingDirectory=/home/pi/so100-arm
ExecStart=/usr/bin/docker-compose up
ExecStop=/usr/bin/docker-compose down
User=pi
Group=docker

[Install]
WantedBy=multi-user.target
SERVICE

echo "Setup complete!"
echo "To enable auto-start: sudo systemctl enable so100-robot.service"
echo "To start now: docker-compose up"
EOF

chmod +x $TEMP_DIR/setup_pi5.sh

# Create run script
cat > $TEMP_DIR/run.sh << 'EOF'
#!/bin/bash

echo "Starting SO-100 Robot Arm on Raspberry Pi 5..."

# Pull latest image (if using registry)
# docker pull so100-arm-pi5:latest

# Start services
docker-compose up -d

# Show logs
docker-compose logs -f
EOF

chmod +x $TEMP_DIR/run.sh

# Transfer files to Pi
echo ""
echo "Transferring files to Raspberry Pi..."
ssh -p $PI_PORT $PI_USER@$PI_HOST "mkdir -p $DEPLOY_DIR"
scp -P $PI_PORT -r $TEMP_DIR/* $PI_USER@$PI_HOST:$DEPLOY_DIR/

# Clean up temp directory
rm -rf $TEMP_DIR

# Run setup on Pi
echo ""
echo "Running setup on Raspberry Pi..."
ssh -p $PI_PORT $PI_USER@$PI_HOST "cd $DEPLOY_DIR && ./setup_pi5.sh"

echo ""
echo "================================================"
echo "✅ Deployment complete!"
echo "================================================"
echo ""
echo "Next steps on Raspberry Pi:"
echo "  1. SSH to Pi: ssh -p $PI_PORT $PI_USER@$PI_HOST"
echo "  2. Navigate: cd $DEPLOY_DIR"
echo "  3. Start robot: ./run.sh"
echo ""
echo "Or enable auto-start:"
echo "  sudo systemctl enable so100-robot.service"
echo "  sudo systemctl start so100-robot.service"
echo ""
echo "Monitor logs:"
echo "  docker-compose logs -f"
echo ""