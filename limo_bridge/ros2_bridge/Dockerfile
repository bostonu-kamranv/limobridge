# Use the official ROS Humble base image
FROM ros:humble-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install CycloneDDS and standard message packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install Python ZeroMQ
RUN pip3 install pyzmq

# Create the application directory
WORKDIR /app

# Copy the bridge script
COPY bridge.py /app/bridge.py

# Source ROS setup
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Entrypoint: Source ROS and run the script
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && python3 bridge.py"]
