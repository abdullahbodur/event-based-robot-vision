## DVS Integrator Setup Guide - Exc4

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
├── dvs_displayer/         # Optional: Event visualization package (from Exercise 2)
│   ├── src/
│   ├── include/
│   └── launch/
├── catkin_ws/             # ROS workspace (created by make setup-ws)
│   ├── src/              # ROS packages (set up by make setup-ws)
│   │   ├── dvs_integrator/  # Copied from dvs_integrator_skeleton
│   │   ├── dvs_displayer/   # Copied from dvs_displayer (if available)
│   │   ├── rpg_dvs_ros/     # Cloned from GitHub (contains dvs_msgs)
│   │   └── catkin_simple/   # Cloned from GitHub (build tool)
│   ├── devel/            # Development files (generated during build)
│   └── build/            # Build files (generated during build)
└── data/                 # Bag files directory (created automatically)
    └── simulation_3planes.bag  # Downloaded during Docker build
```

**Note:** 
- The `dvs_integrator_skeleton` package is not included in this repository. You need to extract `abdullahbodur.zip` and copy the `dvs_integrator_skeleton` folder to the `ex4` folder.
- If you have `dvs_displayer` (from Exercise 2 or course materials), place it in the root folder (`ex4/dvs_displayer/`). It will be automatically copied to `catkin_ws/src/dvs_displayer/` when you run `make setup-ws`.

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

2. **Extract and copy packages:**
   ```bash
   # Extract abdullahbodur.zip
   unzip abdullahbodur.zip
   
   # Copy dvs_integrator_skeleton folder to ex4 folder
   cp -r abdullahbodur/dvs_integrator_skeleton ./dvs_integrator_skeleton
   
   # Copy dvs_displayer folder to ex4 folder (if available from Exercise 2)
   cp -r abdullahbodur/dvs_displayer ./dvs_displayer
   ```
   
   The `dvs_integrator_skeleton` package must be extracted from `abdullahbodur.zip` and copied to the `ex4` folder. If you have `dvs_displayer` (from Exercise 2 or course materials), also copy it to the `ex4` folder - it will be automatically copied to `catkin_ws/src/` during workspace setup.

3. **Set up catkin workspace on host:**
   ```bash
   make setup-ws
   ```
   This will:
   - Create `catkin_ws/src/` directory structure
   - Copy `dvs_integrator_skeleton` from root folder to `catkin_ws/src/dvs_integrator`
   - Copy `dvs_displayer` from root folder to `catkin_ws/src/dvs_displayer` (if it exists in root)
   - Clone dependencies:
     - `catkin_simple` (build tool) from GitHub
     - `rpg_dvs_ros` (contains: `dvs_msgs`, `dvs_renderer`, `davis_ros_driver`, etc.) from GitHub
   - Show what packages are in the workspace
   
   **Note:** Place `dvs_displayer` in the root folder (`ex4/dvs_displayer/`) if you have it. The `setup-ws` target will automatically copy it to `catkin_ws/src/dvs_displayer/`.

4. **Build Docker image** (downloads bag file and sets up container):
   ```bash
   make docker-build
   ```
   This will:
   - Download ROS Noetic base image
   - Install system dependencies (git, wget, catkin-tools, vcstool, rosdep)
   - Copy `dvs_integrator_skeleton` to container's `/catkin_ws/src/dvs_integrator`
   - Install ROS package dependencies via `rosdep`
   - Download the `simulation_3planes.bag` file

5. **Build the ROS workspace:**
   ```bash
   make build
   ```
   This compiles the necessary ROS packages in the workspace:
   - `catkin_simple` (build tool)
   - `dvs_msgs` (message definitions)
   - `dvs_integrator` (main package)
   - `dvs_displayer` (if available)
   
   **Note:** The build process only builds the packages needed for the integrator. Driver packages that require `libcaer` are skipped.

### Catkin Workspace Structure

The `catkin_ws/` folder structure is set up as follows:

```
catkin_ws/
├── src/                    # Source packages (mounted from host to container)
│   ├── dvs_integrator/     # Main package (from dvs_integrator_skeleton)
│   ├── dvs_displayer/      # Optional: Event visualization (from Exercise 2)
│   ├── rpg_dvs_ros/        # DVS ROS packages (cloned by Dockerfile)
│   │   ├── dvs_msgs/       # Event message definitions
│   │   ├── dvs_renderer/   # Event visualization renderer
│   │   ├── davis_ros_driver/  # DAVIS camera driver
│   │   └── ...             # Other rpg_dvs_ros packages
│   └── catkin_simple/      # Build tool (cloned by Dockerfile)
├── devel/                  # Development files (generated during build)
└── build/                  # Build files (generated during build)
```

**How it works:**
- **Makefile (`setup-ws`)**: Sets up the workspace on the host machine by:
  - Copying packages from root folder to `catkin_ws/src/`
  - Cloning dependencies (`catkin_simple`, `rpg_dvs_ros`) from GitHub
- **Dockerfile**: Sets up the Docker image with ROS Noetic and system dependencies
- **docker-compose.yml**: Mounts `./catkin_ws/src` from host to `/catkin_ws/src` in container, so your local changes are reflected

**Required packages in `catkin_ws/src/`:**
- `dvs_integrator/` - Main package (required, copied from root `dvs_integrator_skeleton/`)
- `catkin_simple/` - Build tool (required, cloned by `setup-ws`)
- `rpg_dvs_ros/` - DVS ROS packages (required, cloned by `setup-ws`, contains `dvs_msgs`)

**Optional packages:**
- `dvs_displayer/` - Event visualization (optional, copied from root `dvs_displayer/` if it exists)

**Package locations:**
- Place `dvs_integrator_skeleton/` in the root folder (`ex4/`) - it will be copied to `catkin_ws/src/dvs_integrator/` by `make setup-ws`
- Place `dvs_displayer/` in the root folder (`ex4/`) if you have it - it will be copied to `catkin_ws/src/dvs_displayer/` by `make setup-ws`
- `rpg_dvs_ros/` and `catkin_simple/` are automatically cloned by `make setup-ws` from GitHub

6. **Setup X11 for GUI** (macOS only):
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
- If you see "libcaer" errors: This is expected - driver packages require libcaer, but `dvs_integrator` and `dvs_displayer` don't need it. The build process skips these packages automatically.

### Docker Mount Issues on macOS
- If you see "file exists" errors when running `make build`:
  - Try: `rm -rf catkin_ws && make setup-ws`
  - Or restart Docker Desktop
  - Ensure Docker Desktop has file sharing enabled for your project directory
