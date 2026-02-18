# Use Melodic for Ubuntu 18.04 compatibility
FROM ros:melodic-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Install Python 2 dependencies
RUN apt-get update && apt-get install -y \
    python-pip \
    python-dev \
    build-essential \
    ros-melodic-geometry-msgs \
    ros-melodic-sensor-msgs \
    ros-melodic-nav-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install pyzmq for Python 2
RUN pip install pyzmq

WORKDIR /app
COPY bridge.py /app/bridge.py

# Source Melodic
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

# Run with 'python' (which is 2.7), not 'python3'
CMD ["bash", "-c", "source /opt/ros/melodic/setup.bash && python bridge.py"]
