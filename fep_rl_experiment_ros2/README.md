# fep_rl_experiment_ros2

ROS2 port of the `fep_rl_experiment` package for online reinforcement learning with the Franka Emika Panda manipulator.

## Overview

This package is a complete port from ROS1 (rospy) to ROS2 (rclpy) while maintaining full functional compatibility with the original system. The architecture remains unchanged: a ROS-based robot control stack communicates with a remote GPU-based training stack via ZMQ.

## What's New in ROS2

### API Changes

- **Node Architecture**: All classes now inherit from `rclpy.node.Node`
- **Publishers/Subscribers**: Converted from rospy to rclpy API (`create_publisher`, `create_subscription`)
- **Services**: Service callbacks now take `(request, response)` and return `response`
- **Parameters**: Declarative parameter system (`declare_parameter` + `get_parameter`)
- **Logging**: Node-based logging (`self.get_logger().info()`)
- **Time**: Clock-based time API (`self.get_clock().now().to_msg()`)
- **Timers**: Timer-based publishing instead of `rospy.Rate()` loops for dummy publishers
- **Launch Files**: Python-based launch files instead of XML

### Build System

- **ament_cmake**: Replaces catkin
- **setup.py**: Defines entry points for executables
- **setup.cfg**: Required configuration file for ROS2 Python packages
- **package.xml**: Format 3 with updated dependencies

## Package Structure

```
fep_rl_experiment_ros2/
├── package.xml                    # ROS2 package manifest
├── setup.py                       # Python package with entry points
├── setup.cfg                      # Install configuration
├── CMakeLists.txt                 # ament_cmake build file
├── resource/
│   └── fep_rl_experiment_ros2    # Empty marker file
├── config/
│   └── robot_params.yaml          # Robot parameters
├── launch/                        # Python launch files
│   ├── bringup_sim.launch.py
│   ├── bringup_real.launch.py
│   └── cube_detection.launch.py
└── fep_rl_experiment_ros2/        # Python package
    ├── __init__.py
    ├── robot.py                   # Robot class (main ROS interface)
    ├── experiment_driver.py       # ExperimentDriver orchestration
    ├── environment.py             # PandaPickCube environment
    ├── transitions_server.py      # ZMQ server
    ├── session.py                 # Session logging
    ├── logger.py                  # CSV logger
    ├── plot.py                    # Reward plotting
    ├── robot_interface.py         # Test node
    ├── online_learning.py         # Main experiment node
    ├── dummy_image_publisher.py   # Simulation image publisher
    └── dummy_cube_publisher.py    # Simulation cube publisher
```

## Executables

The package provides four executables:

1. **robot_interface**: Test node for robot control
2. **online_learning**: Main experiment node for training
3. **dummy_image_publisher**: Publishes synthetic camera images (simulation)
4. **dummy_cube_publisher**: Publishes dummy cube poses (simulation)

## Building

### Prerequisites

- ROS2 (tested with Humble/Iron)
- Python 3.8+
- cv_bridge, tf2_ros, tf2_geometry_msgs
- numpy, opencv-python, matplotlib, zmq, onnxruntime

### Option 1: Docker Workflow (Recommended)

The easiest way to use this package is with Docker, which provides a complete ROS2 environment with all dependencies.

#### Initial Setup

```bash
# Pull the training submodule (if not already done)
git submodule update --init --recursive

# Build the ROS2 Docker image
docker compose -f docker/docker-compose.yaml build fep_rl_ros2
```

#### Run the ROS2 Container

```bash
# Option 1: Start an interactive session (removed after exit)
docker compose -f docker/docker-compose.yaml run --rm fep_rl_ros2 bash

# Option 2: Start in detached mode (keeps running in background)
docker compose -f docker/docker-compose.yaml up -d fep_rl_ros2

# Then connect to the running container
docker exec -it $(docker ps -qf "ancestor=fep_rl_ros2") bash

# Inside the container, the workspace is already built and sourced
# You can immediately run nodes:
ros2 launch fep_rl_experiment_ros2 bringup_sim.launch.py
```

#### Rebuild After Code Changes

If you modify the Python code, you can rebuild the package inside the container:

```bash
# Inside the container
cd /ros2_ws
colcon build --packages-select fep_rl_experiment_ros2
source install/setup.bash
```

Or rebuild the Docker image for a fresh environment:

```bash
# Outside the container
docker compose -f docker/docker-compose.yaml build fep_rl_ros2
```

### Option 2: Native Build

If you prefer to build natively without Docker:

```bash
cd ~/ros2_ws/src
ln -s /path/to/panda-rl-kit/fep_rl_experiment_ros2 .
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select fep_rl_experiment_ros2
source install/setup.bash
```

## Usage

### Docker Workflow

#### Running Simulation

```bash
# Start the ROS2 container
docker compose -f docker/docker-compose.yaml run --rm fep_rl_ros2 bash

# Inside the container, launch simulation
ros2 launch fep_rl_experiment_ros2 bringup_sim.launch.py
```

#### Running on Real Robot

```bash
# Start the ROS2 container with access to hardware
docker compose -f docker/docker-compose.yaml run --rm fep_rl_ros2 bash

# Inside the container, launch with real robot
ros2 launch fep_rl_experiment_ros2 bringup_real.launch.py \
  robot_ip:=172.16.1.11 \
  sessionId:=session_0 \
  markerSize:=0.042 \
  cubeSize:=0.05
```

#### Two-Container Training Setup

For online training, run both the ROS2 container (robot side) and the training container (GPU side):

```bash
# Terminal 1: Start ROS2 container
docker compose -f docker/docker-compose.yaml up -d fep_rl_ros2
docker exec -it $(docker ps -qf "ancestor=fep_rl_ros2") bash
# Inside: Launch the experiment
ros2 launch fep_rl_experiment_ros2 bringup_real.launch.py sessionId:=my_session

# Terminal 2: Start training container
docker compose -f docker/docker-compose.yaml up -d safe_learning
docker exec -it $(docker ps -qf "ancestor=safe_learning") bash
# Inside: Run training (connects to port 5559 on host)
python train_brax.py +experiment=franka_online
```

The containers communicate via the ZMQ server on port 5559 (exposed through `network_mode: host`).

### Native Workflow

#### Simulation

```bash
ros2 launch fep_rl_experiment_ros2 bringup_sim.launch.py
```

#### Real Robot

```bash
ros2 launch fep_rl_experiment_ros2 bringup_real.launch.py \
  robot_ip:=172.16.1.11 \
  sessionId:=session_0 \
  markerSize:=0.042 \
  cubeSize:=0.05
```

#### Online Learning with Parameters

```bash
ros2 run fep_rl_experiment_ros2 online_learning \
  --ros-args \
  -p session_id:=my_session \
  -p trajectory_length:=250 \
  -p dt:=0.05
```

## Key Migration Details

### Robot Class (robot.py)

- Inherits from `Node` instead of standalone class
- TF2 listener now requires node reference: `TransformListener(buffer, self)`
- Timestamps converted: `stamp.sec + stamp.nanosec / 1e9`
- Service callbacks return response object with success/message fields

### ExperimentDriver (experiment_driver.py)

- Parameters declared at initialization
- Publishers require message objects (not primitives)
- Logger accessed via `self.get_logger()`

### Environment (environment.py)

- Service calls pass `(request, response)` to callbacks
- Logger accessed through `robot.get_logger()`

### Launch Files

All launch files converted to Python:
- `DeclareLaunchArgument` for parameters
- `IncludeLaunchDescription` for nested launches
- `Node` for individual nodes
- `remappings` and `parameters` as lists/dicts

## Testing Checklist

- [ ] Package builds without errors
- [ ] All executables discoverable: `ros2 pkg executables fep_rl_experiment_ros2`
- [ ] Dummy publishers work: `ros2 topic echo /camera/color/image_raw`
- [ ] Robot node initializes with all publishers/subscribers
- [ ] Parameters correctly set and retrieved
- [ ] Service calls work (reset_controller, start_controller)
- [ ] TF lookups work (aruco_cube_frame → panda_link0)
- [ ] Launch files start all nodes without errors
- [ ] ZMQ server listens on port 5559
- [ ] Full simulation pipeline works end-to-end

## Docker Architecture

The project uses Docker Compose with three services:

1. **fep_rl** (ROS1 Noetic) - Original ROS1 implementation
2. **fep_rl_ros2** (ROS2 Humble) - New ROS2 implementation (this package)
3. **safe_learning** (CUDA + JAX) - GPU-accelerated training stack

### Container Features

- **Network Mode**: `host` for direct hardware access and inter-container communication
- **Device Access**: Full `/dev` access for robot controllers, cameras, and sensors
- **X11 Forwarding**: GUI support (RViz, Gazebo) through mounted X11 socket
- **Volume Mounts**: Live code editing via bind mounts to `/code`
- **Port Exposure**: ZMQ server on 5559, Franka controller on UDP 20210-20230 and 33300-33400

### Environment Variables

- `DISPLAY`: X11 display for GUI applications
- `ROS_DOMAIN_ID`: ROS2 DDS domain (default: 0)
- `NVIDIA_VISIBLE_DEVICES`: GPU access for training container

## Docker Troubleshooting

### Permission Issues with Devices

If you encounter permission errors accessing `/dev` devices:

```bash
# Add your user to dialout and video groups
sudo usermod -aG dialout $USER
sudo usermod -aG video $USER
# Log out and back in for changes to take effect
```

### X11 Display Issues

If GUI applications fail to start:

```bash
# Allow X server connections
xhost +local:docker

# Or run container with --privileged flag (less secure)
docker compose run --rm --privileged fep_rl_ros2 bash
```

### Container Networking

The containers use `network_mode: host` to:
- Access Franka robot at its IP (e.g., 172.16.1.11)
- Bind ZMQ server to port 5559 accessible from other containers
- Use ROS2 DDS multicast without port mapping

### Rebuilding After Dependency Changes

If you modify `package.xml` or install new dependencies:

```bash
# Rebuild the image from scratch
docker compose -f docker/docker-compose.yaml build --no-cache fep_rl_ros2
```

### Cleaning Up

```bash
# Remove stopped containers
docker compose -f docker/docker-compose.yaml down

# Remove images
docker rmi fep_rl_ros2 fep_rl safe_learning

# Clean all build artifacts
docker system prune -a
```

## Troubleshooting

### CUDA Version Mismatch (safe_learning container)

If you see this error when starting the training container:
```
nvidia-container-cli: requirement error: unsatisfied condition: cuda>=12.x
```

**Solution:** Your NVIDIA driver doesn't support the CUDA version in the Dockerfile. Fix it by:

1. Check your driver version:
   ```bash
   nvidia-smi --query-gpu=driver_version --format=csv,noheader
   ```

2. Update `docker/Dockerfile.safe_learning` base image based on your driver:
   - Driver 535.x: `nvidia/cuda:12.2.2-cudnn8-devel-ubuntu22.04`
   - Driver 545.x: `nvidia/cuda:12.3.2-cudnn8-devel-ubuntu22.04`
   - Driver 555.x+: `nvidia/cuda:12.5.1-cudnn-devel-ubuntu22.04`

3. Rebuild:
   ```bash
   docker compose -f docker/docker-compose.yaml build safe_learning
   ```

See main README for complete CUDA compatibility table.

## Known Limitations

1. **ArUco ROS2**: Requires `aruco_ros` ROS2 package - may need porting or alternative
2. **Franka ROS2**: Assumes Franka controllers are available in ROS2 (e.g., `franka_ros2`)
3. **Threading**: ZMQ server runs in separate thread - may need `MultiThreadedExecutor` for better concurrency
4. **Docker Size**: ROS2 Humble desktop-full image is ~6GB; consider using ros-base for production

## Compatibility

The ZMQ protocol and message formats remain unchanged, ensuring compatibility with the existing training stack (`external/safe-learning`).

## Quick Reference

### Docker Commands

```bash
# Build the image
docker compose -f docker/docker-compose.yaml build fep_rl_ros2

# Start interactive session (one-off)
docker compose -f docker/docker-compose.yaml run --rm fep_rl_ros2 bash

# Start persistent container (background)
docker compose -f docker/docker-compose.yaml up -d fep_rl_ros2

# Connect to running container
docker exec -it $(docker ps -qf "ancestor=fep_rl_ros2") bash

# Run a single command
docker compose -f docker/docker-compose.yaml run --rm fep_rl_ros2 \
  ros2 launch fep_rl_experiment_ros2 bringup_sim.launch.py

# List available executables
docker compose -f docker/docker-compose.yaml run --rm fep_rl_ros2 \
  ros2 pkg executables fep_rl_experiment_ros2

# Rebuild package inside container
docker compose -f docker/docker-compose.yaml run --rm fep_rl_ros2 bash -c \
  "cd /ros2_ws && colcon build --packages-select fep_rl_experiment_ros2"
```

### ROS2 Commands

```bash
# List topics
ros2 topic list

# Echo a topic
ros2 topic echo /instant_reward

# Check node info
ros2 node info /online_learning

# Call a service
ros2 service call /reset_controller std_srvs/srv/Empty

# View transform tree
ros2 run tf2_tools view_frames
```

### Useful Development Commands

```bash
# Watch reward topic in real-time
ros2 topic echo /instant_reward --no-arr

# Monitor image publication rate
ros2 topic hz /camera/color/image_raw

# Check parameter values
ros2 param list /online_learning
ros2 param get /online_learning trajectory_length

# Record a bag for debugging
ros2 bag record -a -o my_experiment

# Check system health
ros2 doctor
```

## References

- Original ROS1 package: `fep_rl_experiment`
- Training stack: `external/safe-learning` (Git submodule)
- ROS2 Migration Guide: https://docs.ros.org/en/humble/How-To-Guides/Migrating-from-ROS1.html
- ROS2 Humble Documentation: https://docs.ros.org/en/humble/index.html
- Docker Compose Documentation: https://docs.docker.com/compose/
