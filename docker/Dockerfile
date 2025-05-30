# Use the base ROS2 Jazzy image
FROM ros:jazzy-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-web-video-server \
    ros-${ROS_DISTRO}-rosbridge-suite \
    ros-${ROS_DISTRO}-image-transport-plugins \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Copy the start.sh script
COPY start.sh /start.sh

# Give execute permission to the start.sh script
RUN chmod +x /start.sh

# Run the start.sh script
CMD ["/start.sh"]
