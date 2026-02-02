# Docker Guide for fep_rl_experiment_ros2

This guide provides detailed information about using the ROS2 package with Docker.

## Overview

The `fep_rl_ros2` Docker image provides a complete ROS2 Humble environment with all dependencies pre-installed and the workspace pre-built.

## Image Details

- **Base Image**: `osrf/ros:humble-desktop-full`
- **ROS Distribution**: ROS2 Humble Hawksbill (LTS)
- **Python Version**: 3.10
- **Workspace**: `/ros2_ws`
- **Build System**: colcon
- **Size**: ~6-7 GB (includes GUI tools)

## Building the Image

### First Time Build

```bash
# From repository root
docker compose -f docker/docker-compose.yaml build fep_rl_ros2
```

This will:
1. Install system dependencies (RealSense SDK, build tools)
2. Install Python dependencies (numpy, opencv, onnxruntime, zmq, matplotlib)
3. Create ROS2 workspace at `/ros2_ws`
4. Build the `fep_rl_experiment_ros2` package
5. Configure shell environment

Build time: ~15-30 minutes depending on your system.

### Rebuild After Changes

**IMPORTANT:** `docker compose up` does NOT automatically rebuild images when the Dockerfile changes. You must explicitly rebuild.

If you modify code (Python files):
```bash
# Rebuild inside container (faster)
docker compose -f docker/docker-compose.yaml run --rm fep_rl_ros2 bash
cd /ros2_ws
colcon build --packages-select fep_rl_experiment_ros2
```

If you modify dependencies (`package.xml`, `setup.py`) or Dockerfile:
```bash
# Option 1: Explicit rebuild (recommended)
docker compose -f docker/docker-compose.yaml build fep_rl_ros2
docker compose -f docker/docker-compose.yaml up -d fep_rl_ros2

# Option 2: Build + start in one command
docker compose -f docker/docker-compose.yaml up -d --build fep_rl_ros2

# Option 3: Force complete rebuild (if cached layers cause issues)
docker rmi fep_rl_ros2
docker compose -f docker/docker-compose.yaml build --no-cache fep_rl_ros2
```

## Running Containers

### Interactive Shell

**Option 1: One-off interactive session (removed after exit)**
```bash
docker compose -f docker/docker-compose.yaml run --rm fep_rl_ros2 bash
```

**Option 2: Persistent container (keeps running)**
```bash
# Start in detached mode
docker compose -f docker/docker-compose.yaml up -d fep_rl_ros2

# Connect to the running container
docker exec -it $(docker ps -qf "ancestor=fep_rl_ros2") bash

# Or manually find and connect
docker ps  # Find container ID
docker exec -it <container_id> bash
```

Both options give you a bash shell with:
- ROS2 environment sourced
- Workspace built and sourced
- Code mounted at `/code`
- Hardware devices accessible

### Single Command

```bash
docker compose -f docker/docker-compose.yaml run --rm fep_rl_ros2 \
  ros2 launch fep_rl_experiment_ros2 bringup_sim.launch.py
```

### Background Service

```bash
# Start detached
docker compose -f docker/docker-compose.yaml up -d fep_rl_ros2

# Connect to running container
docker exec -it $(docker ps -qf "ancestor=fep_rl_ros2") bash

# View logs
docker compose -f docker/docker-compose.yaml logs -f fep_rl_ros2

# Stop
docker compose -f docker/docker-compose.yaml down
```

## Volume Mounts

The compose file mounts:

| Host Path | Container Path | Purpose |
|-----------|---------------|---------|
| `..` (repo root) | `/code` | Live code editing |
| `/tmp/.X11-unix` | `/tmp/.X11-unix` | X11 display (GUI) |
| `/dev` | `/dev` | Hardware devices |
| `~/.Xauthority` | `~/.Xauthority` | X11 authentication |

Any changes you make to files under `/code` in the container are immediately reflected on the host and vice versa.

## Network Configuration

- **Mode**: `host` - Container shares host's network namespace
- **Benefits**:
  - Direct access to Franka robot IP
  - ZMQ server on 5559 accessible from other containers
  - ROS2 DDS multicast works without configuration
- **Trade-offs**:
  - Less network isolation
  - Port conflicts possible

### Exposed Ports

| Port/Range | Protocol | Purpose |
|------------|----------|---------|
| 5559 | TCP | ZMQ transitions server |
| 20210-20230 | UDP | Franka controller communication |
| 33300-33400 | UDP | Franka controller data |

## Device Access

The container has access to:
- `/dev/dri` - GPU/graphics (via devices mapping)
- All `/dev` devices (via volume mount)
- USB devices (for cameras, sensors)

For specific devices, add to compose file:
```yaml
devices:
  - /dev/video0:/dev/video0  # Specific camera
```

## Environment Variables

### Set at Runtime

```bash
# Custom ROS domain
docker compose run --rm -e ROS_DOMAIN_ID=42 fep_rl_ros2 bash

# Different display
docker compose run --rm -e DISPLAY=:1 fep_rl_ros2 bash
```

### Default Variables

- `DISPLAY`: Inherited from host for X11
- `ROS_DOMAIN_ID`: 0 (default)
- `NVIDIA_VISIBLE_DEVICES`: all
- `NVIDIA_DRIVER_CAPABILITIES`: all

## Multi-Container Workflows

### ROS2 + Training Stack

```bash
# Terminal 1: Robot control (ROS2)
docker compose -f docker/docker-compose.yaml up -d fep_rl_ros2
docker exec -it $(docker ps -qf "ancestor=fep_rl_ros2") bash
ros2 launch fep_rl_experiment_ros2 bringup_real.launch.py sessionId:=exp_001

# Terminal 2: Policy training (GPU)
docker compose -f docker/docker-compose.yaml up -d safe_learning
docker exec -it $(docker ps -qf "ancestor=safe_learning") bash
python train_brax.py +experiment=franka_online
```

Both containers communicate via ZMQ on port 5559 through the host network.

## Troubleshooting

### X11 Permission Denied

```bash
# On host
xhost +local:docker
```

Or run with display explicitly:
```bash
docker compose run --rm -e DISPLAY=$DISPLAY fep_rl_ros2 bash
```

### Device Permission Issues

Add your user to required groups:
```bash
sudo usermod -aG dialout,video,plugdev $USER
# Log out and back in
```

### ROS2 DDS Issues

If nodes can't discover each other:
```bash
# Check ROS_DOMAIN_ID matches
echo $ROS_DOMAIN_ID

# Use Fast-DDS (default in Humble)
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Or Cyclone DDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### Container Build Fails

Clear Docker cache and rebuild:
```bash
docker system prune -a
docker compose -f docker/docker-compose.yaml build --no-cache fep_rl_ros2
```

### Package Not Found

If `ros2 pkg list` doesn't show your package:
```bash
# Rebuild workspace
cd /ros2_ws
colcon build --symlink-install --packages-select fep_rl_experiment_ros2
source install/setup.bash
```

### ZMQ Port Already in Use

Check what's using port 5559:
```bash
sudo netstat -tlnp | grep 5559
# Or with ss
sudo ss -tlnp | grep 5559
```

### CUDA Version Error (safe_learning container)

If you see:
```
nvidia-container-cli: requirement error: unsatisfied condition: cuda>=12.x
please update your driver to a newer version, or use an earlier cuda container
```

Your NVIDIA driver is too old for the CUDA version in `Dockerfile.safe_learning`.

**Quick fix:**

1. Check your driver:
   ```bash
   nvidia-smi --query-gpu=driver_version --format=csv,noheader
   ```

2. Edit `docker/Dockerfile.safe_learning` line 2:
   - Driver 535.x → Use `nvidia/cuda:12.2.2-cudnn8-devel-ubuntu22.04`
   - Driver 545.x → Use `nvidia/cuda:12.3.2-cudnn8-devel-ubuntu22.04`
   - Driver 555.x+ → Use `nvidia/cuda:12.5.1-cudnn-devel-ubuntu22.04`

3. **IMPORTANT**: Remove old image and rebuild:
   ```bash
   # Remove the old cached image
   docker rmi safe_learning

   # Rebuild with new CUDA version
   docker compose -f docker/docker-compose.yaml build safe_learning

   # Now start it
   docker compose -f docker/docker-compose.yaml up -d safe_learning
   ```

**Why remove the old image?** Docker Compose doesn't automatically rebuild when you run `up` - it reuses cached images. You must explicitly build or remove the old image first.

See the CUDA compatibility table in the main README for more details.

## Performance Tips

### Faster Builds

Use more parallel jobs:
```bash
colcon build --parallel-workers 8 --packages-select fep_rl_experiment_ros2
```

### Smaller Image

For production without GUI tools:
```dockerfile
FROM osrf/ros:humble-ros-base  # Instead of desktop-full
```

### Layer Caching

Order Dockerfile commands from least to most frequently changed:
1. System packages
2. Python dependencies
3. Code copy
4. Build

## Development Workflow

Recommended workflow for iterative development:

```bash
# 1. Start container once
docker compose -f docker/docker-compose.yaml run --rm fep_rl_ros2 bash

# 2. Make changes on host in your editor

# 3. Rebuild in container
cd /ros2_ws
colcon build --packages-select fep_rl_experiment_ros2

# 4. Test immediately
ros2 launch fep_rl_experiment_ros2 bringup_sim.launch.py

# 5. Repeat steps 2-4
```

No need to rebuild the Docker image for Python changes!

## Cleanup

Remove containers:
```bash
docker compose -f docker/docker-compose.yaml down
```

Remove images:
```bash
docker rmi fep_rl_ros2
```

Remove all unused Docker resources:
```bash
docker system prune -a --volumes
```

## Advanced: Custom Build Arguments

Modify `docker/Dockerfile.ros2` to add build arguments:

```dockerfile
ARG MAKE_JOBS=6
ARG ROS_DISTRO=humble
```

Then build with:
```bash
docker compose build --build-arg MAKE_JOBS=12 fep_rl_ros2
```
