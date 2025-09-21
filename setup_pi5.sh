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
