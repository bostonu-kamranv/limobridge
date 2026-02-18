# Use the official ROS Noetic base image
FROM ros:noetic-ros-base

# Set environment variables to avoid interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive

# Install Python 3 pip and ZeroMQ
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-noetic-geometry-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-nav-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install the Python ZeroMQ library
RUN pip3 install pyzmq

# Create the application directory
WORKDIR /app

# Copy the bridge script into the container
COPY bridge.py /app/bridge.py

# Source ROS setup in .bashrc so it loads if you shell in
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# The entrypoint command. 
# We use 'bash -c' to source ROS before running the script.
CMD ["bash", "-c", "source /opt/ros/noetic/setup.bash && python3 bridge.py"]
