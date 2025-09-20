FROM osrf/ros:humble-desktop-full

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    wget \
    curl \
    nano \
    htop \
    usbutils \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Install MoveIt2 and ros2_control packages
RUN apt-get update && apt-get install -y \
    ros-humble-moveit \
    ros-humble-moveit-configs-utils \
    ros-humble-moveit-planners \
    ros-humble-moveit-simple-controller-manager \
    ros-humble-moveit-ros-visualization \
    ros-humble-moveit-setup-assistant \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller \
    ros-humble-position-controllers \
    ros-humble-gazebo-ros2-control \
    ros-humble-gz-ros2-control \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2 \
    ros-humble-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages for star tracking calculations and testing
RUN pip3 install \
    numpy \
    astropy \
    pyephem \
    skyfield \
    opencv-python \
    scipy \
    pyserial \
    adafruit-circuitpython-bno055 \
    picamera2 || true

# Create workspace
WORKDIR /ros2_ws

# Copy the SO-100 arm package and star_tracker
COPY . /ros2_ws/src/so_100_arm/
COPY star_tracker/ /ros2_ws/src/star_tracker/

# Copy hardware config for so_arm_100_hardware
COPY config/hardware_config.yaml /ros2_ws/src/so_arm_100_hardware/config/hardware_config.yaml

# Clone hardware interface package (if needed for physical robot)
RUN cd /ros2_ws/src && \
    git clone https://github.com/matbeedotcom/so_arm_100_hardware.git || true

# Initialize rosdep and install dependencies
RUN cd /ros2_ws && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y || true

# Build the workspace (including star_tracker if present)
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /ros2_ws && \
    colcon build --symlink-install --packages-up-to star_tracker so_100_arm so_arm_100_hardware || \
    colcon build --symlink-install"

# Setup entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Create non-root user for better security
RUN useradd -m -s /bin/bash ros && \
    echo "ros:ros" | chpasswd && \
    usermod -aG dialout ros && \
    chown -R ros:ros /ros2_ws

# Switch to non-root user
USER ros

# Source ROS2 setup in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/ros/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /home/ros/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> /home/ros/.bashrc

WORKDIR /ros2_ws

ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]