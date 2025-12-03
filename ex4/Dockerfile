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

# Note: libcaer-dev is needed for rpg_dvs_ros driver packages but not for dvs_msgs or dvs_integrator
# If you need to build driver packages, install libcaer-dev separately:
# Add iniVation repo: wget -qO - https://www.inivation.com/Debian/archive-key.asc | apt-key add -
# Then: echo "deb https://www.inivation.com/Debian stable main" > /etc/apt/sources.list.d/inivation.list
# Then: apt-get update && apt-get install -y libcaer-dev

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Create workspace structure
RUN mkdir -p /catkin_ws/src

# Copy dvs_integrator_skeleton package to workspace
COPY dvs_integrator_skeleton /catkin_ws/src/dvs_integrator

# Copy dependencies file
COPY dvs_integrator_skeleton/dependencies.yaml /catkin_ws/src/dvs_integrator/

# Clone dependencies (rpg_dvs_ros contains: dvs_msgs, dvs_renderer, davis_ros_driver, etc.)
# This will clone rpg_dvs_ros (with all its packages) and catkin_simple
RUN cd /catkin_ws/src && \
    vcs-import < dvs_integrator/dependencies.yaml

# Note: dvs_displayer is optional and should be provided separately
# If you have dvs_displayer, copy it to catkin_ws/src/dvs_displayer on the host
# or add it to the build context and uncomment the line below:
# COPY dvs_displayer /catkin_ws/src/dvs_displayer

# Install system dependencies
RUN cd /catkin_ws && \
    rosdep install --from-paths src --ignore-src -r -y || true

# Create data directory and download bag file
RUN mkdir -p /catkin_ws/data && \
    cd /catkin_ws/data && \
    wget -q http://rpg.ifi.uzh.ch/datasets/davis/simulation_3planes.bag -O simulation_3planes.bag || \
    (echo "Warning: Failed to download bag file. You can download it manually later." && touch simulation_3planes.bag)

# Source the workspace
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

# Default command
CMD ["/bin/bash"]

