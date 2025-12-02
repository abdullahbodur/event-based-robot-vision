# dvs_integrator

Event-by-event processing in ROS C++ for the purpose of image reconstruction (using a high pass filter). Based on the paper by Scheerlinck et al., ACCV 2018.

The package reads events from a topic and outputs the reconstructed images using per-pixel temporal filters (high pass filter).

## Project Structure

```
ex4/
├── Dockerfile              # Docker image definition
├── docker-compose.yml     # Docker Compose configuration
├── Makefile               # Convenience commands for running the project
├── README.md              # This file
├── dvs_integrator_skeleton/  # Source code package (extract from abdullahbodur.zip)
│   ├── src/
│   ├── include/
│   ├── launch/
│   └── dependencies.yaml
├── catkin_ws/             # ROS workspace (created during build)
│   ├── src/              # ROS packages
│   ├── devel/            # Development files
│   └── build/            # Build files
└── data/                 # Bag files directory (created automatically)
    └── simulation_3planes.bag  # Downloaded during Docker build
```

**Note:** The `dvs_integrator_skeleton` package is not included in this repository. You need to extract `abdullahbodur.zip` and copy the `dvs_integrator_skeleton` folder to the `ex4` folder.

## Slides and videos
- [Slides](https://drive.google.com/file/d/1MtbzVMKebJq2I0FaG6CtmH0gEomAJ19h/view)
- [Direct integrator](https://youtu.be/BNwFeG4CSE4)
- [Leaky integrator](https://youtu.be/4u-I9LNW4gw)

## Input / Output
**Input**:
- Events (topic: `/dvs/events`)

**Output**:
- Time map (topic: `/event_integrator/time_map`) - Time surface visualization
- Reconstructed intensity image (topic: `/event_integrator/image`) - Brightness image reconstructed from events

**Parameters** (dynamic reconfigure):
- Cutoff frequency (alpha) - Controls the decay rate of the high pass filter (range: 0-20)

---

## Setup

### Prerequisites

**For Docker (macOS/Linux/Windows):**
- [Docker Desktop](https://www.docker.com/products/docker-desktop) installed
- For macOS: [XQuartz](https://www.xquartz.org/) installed (`brew install --cask xquartz`)

**For Native Linux:**
- ROS Noetic installed
- Standard build tools (cmake, g++, etc.)

### Initial Setup

1. **Clone the repository:**
   ```bash
   git clone https://github.com/abdullahbodur/event-based-robot-vision.git
   cd event-based-robot-vision/ex4
   ```

2. **Extract and copy the dvs_integrator_skeleton package:**
   ```bash
   # Extract abdullahbodur.zip
   unzip abdullahbodur.zip
   
   # Copy dvs_integrator_skeleton folder to ex4 folder
   cp -r abdullahbodur/dvs_integrator_skeleton ./dvs_integrator_skeleton
   ```
   
   The `dvs_integrator_skeleton` package must be extracted from `abdullahbodur.zip` and copied to the `ex4` folder.

3. **Build Docker image** (downloads dependencies and bag file):
   ```bash
   make docker-build
   ```
   This will:
   - Download ROS Noetic base image
   - Install dependencies (catkin_simple, rpg_dvs_ros)
   - Download the `simulation_3planes.bag` file

4. **Build the ROS workspace:**
   ```bash
   make build
   ```
   This compiles all ROS packages in the workspace.

5. **Setup X11 for GUI** (macOS only):
   ```bash
   # Start XQuartz
   open -a XQuartz
   
   # Configure XQuartz:
   # 1. Go to Preferences → Security
   # 2. Check "Allow connections from network clients"
   # 3. Restart XQuartz
   
   # Allow X11 connections (run in terminal)
   export PATH="/opt/X11/bin:$PATH"
   export DISPLAY=:0
   xhost +localhost
   xhost +$(ipconfig getifaddr en0)
   ```

---

## Quick Start (Docker)

All commands should be run from the project root directory (`ex4/`). Use the Makefile for easy execution:

**Terminal 1 - Start ROS Core:**
```bash
make roscore
```

**Terminal 2 - Launch Integrator Node:**
```bash
make integrator
```

**Terminal 3 - Launch Event Displayer** (optional, for event visualization):
```bash
make displayer
```

**Terminal 4 - Open Visualization:**
```bash
make visualization
```

**Terminal 5 - Play ROS Bag File:**
```bash
make bag BAG_FILE=simulation_3planes.bag
# or
make bag BAG_FILE=slider_depth.bag
```
- Press **SPACE** to play/pause
- The bag file plays at 0.1x speed (10x slower) for better visualization

**Terminal 6 - Dynamic Reconfigure** (optional):
```bash
make reconfigure
```
- Find `dvs_integrator_one` in the left panel
- Adjust **Cutoff_frequency** parameter (0-20 range)
- Watch the reconstructed image change in real-time

**Note:** After changing the configuration parameters, restart the integrator by running `make integrator` again.

### Available Makefile Commands

```bash
make help          # Show all available commands
make docker-build   # Build Docker image (downloads bag file)
make build          # Build the ROS workspace
make roscore        # Start ROS core
make integrator     # Start dvs_integrator node
make displayer      # Start dvs_displayer node
make visualization  # Start rqt visualization
make bag BAG_FILE=<filename>  # Play bag file
make reconfigure    # Start dynamic reconfigure
make status         # Check Docker container status
make clean          # Stop all Docker containers
```

**Note:** The `dvs_displayer` package should be placed at `catkin_ws/src/dvs_displayer/` (same level as `rpg_dvs_ros`). If you don't have it, you can skip it - event visualization is optional. The bag files are automatically downloaded during Docker build, or you can place your own bag files in the `data/` directory.

---

## Native Linux Setup

### Dependencies

The catkin package dependencies are:
- [catkin simple](https://github.com/catkin/catkin_simple)
- ROS messages for DVS ([rpg_dvs_ros](https://github.com/uzh-rpg/rpg_dvs_ros))

### Create a catkin workspace

```bash
cd
mkdir -p catkin_ws/src
cd catkin_ws
catkin config --init --mkdirs --extend /opt/ros/noetic --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Add packages to the catkin workspace

```bash
cd catkin_ws/src
sudo apt-get install python3-vcstool
vcs-import < dvs_integrator/dependencies.yaml
```

**Note:** If you have the `dvs_displayer` package (from Exercise 2 or course materials), place it at the same level as `rpg_dvs_ros`:
- `catkin_ws/src/dvs_displayer/` (should be at same level as `catkin_ws/src/rpg_dvs_ros/`)

### Compile

**Build dependencies first:**
```bash
cd ~/catkin_ws
catkin build catkin_simple
catkin build dvs_msgs
source devel/setup.bash
```

**Build dvs_displayer (if you have it):**
```bash
catkin build dvs_displayer
source devel/setup.bash
```

**Build this package:**
```bash
catkin build dvs_integrator
source devel/setup.bash
```

### Run

**Terminal 1 - ROS Core:**
```bash
roscore
```

**Terminal 2 - Launch Integrator:**
```bash
roslaunch dvs_integrator integrator.launch
```

**Terminal 3 - Launch Event Displayer (Optional):**
```bash
roslaunch dvs_displayer display_monocular.launch
```

**Terminal 4 - Play Bag File:**
```bash
rosbag play -r 0.1 --pause path_to_file/slider_depth.bag
```

**Terminal 5 - Dynamic Reconfigure:**
```bash
rosrun rqt_reconfigure rqt_reconfigure
```
- Find `dvs_integrator_one` in the left panel
- Adjust **Cutoff_frequency** parameter (0-20 range)

**Note:** After changing the configuration parameters, restart the integrator by running `make integrator` again.

## Troubleshooting

### X11/GUI Issues on macOS
- Make sure XQuartz is running: `open -a XQuartz`
- Check X11 permissions: `xhost`
- Try: `export DISPLAY=:0` before running Docker commands
- Allow X11 connections: `export PATH="/opt/X11/bin:$PATH" && xhost +localhost`

### No Images Appearing
- Check that bag file is playing: `rostopic hz /dvs/events`
- Verify integrator is running: `rostopic list | grep event_integrator`
- Check for errors in terminal output
- Make sure bag file is not paused (press SPACE in rosbag terminal)

### dvs_displayer Not Found
- Ensure `dvs_displayer` is in `catkin_ws/src/dvs_displayer/` (same level as `rpg_dvs_ros`)
- Build it: `catkin build dvs_displayer`
- If you don't have `dvs_displayer`, you can skip it - event visualization is optional

### Build Errors
- Make sure dependencies are built first: `catkin build catkin_simple` then `catkin build dvs_msgs`
- Clean and rebuild: `rm -rf build/ devel/` then `catkin build`
- For Docker: Make sure to source setup.bash: `source /catkin_ws/devel/setup.bash`
