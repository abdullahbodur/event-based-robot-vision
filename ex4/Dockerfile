FROM osrf/ros:noetic-desktop-full

# Set working directory
WORKDIR /catkin_ws

# Install dependencies
RUN apt-get update && apt-get install -y \
    git \
    wget \
    python3-catkin-tools \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Create workspace structure
RUN mkdir -p /catkin_ws/src

# Copy dependencies file
COPY dvs_integrator_skeleton/dependencies.yaml /catkin_ws/src/dvs_integrator/

# Clone dependencies
RUN cd /catkin_ws/src && \
    vcs-import < dvs_integrator/dependencies.yaml

# Install system dependencies
RUN cd /catkin_ws && \
    rosdep install --from-paths src --ignore-src -r -y || true

# Create data directory and download bag file
RUN mkdir -p /catkin_ws/data && \
    cd /catkin_ws/data && \
    wget -q http://rpg.ifi.uzh.ch/datasets/davis/simulation_3planes.bag -O simulation_3planes.bag || \
    (echo "Warning: Failed to download bag file. You can download it manually later." && touch simulation_3planes.bag)

# Copy package source (will be mounted as volume, but this sets up structure)
RUN mkdir -p /catkin_ws/src/dvs_integrator

# Source the workspace
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

# Default command
CMD ["/bin/bash"]

