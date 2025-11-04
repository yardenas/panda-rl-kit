# Franka Emika Panda Online RL

This repository hosts the ROS packages, launch files, and training utilities we use to run online reinforcement-learning experiments on a real Franka Emika Panda manipulator. The goal is to make it straightforward to reproduce the hardware setup, bring up the robot, and iterate on policies either directly on hardware or in simulation.

## Repository Highlights
- `fep_rl_experiment`: ROS package for robot bring-up, cube detection, experiment logging, and the online learning node.
- `scripts/train_training.bash`: helper script that creates an SSH reverse tunnel and launches online learning with a custom session identifier.
- `docker/`: Dockerfile and compose configuration for a fully reproducible runtime environment with ROS Noetic and Intel RealSense support.
- `setup/`: step-by-step hardware and software guides for preparing a lab workstation without containers.

## Before You Start

1. Review the hardware checklist in `setup/README.hardware.md`.
2. Prepare your Ubuntu 20.04 workstation following `setup/README.software.md` **or** use the Docker workflow below.
3. Verify that your Franka Emika Panda has an active FCI license and that you can ping the robot control cabinet from your workstation.

## Docker Workflow

The Docker environment mirrors the system dependencies described in the manual setup guides while isolating ROS and Python packages.

1. Build the image (run from the repository root):
   ```bash
   docker compose -f docker/docker-compose.yaml build
   ```
2. Allow container access to the X server if you plan to run RViz from the container (Linux host):
   ```bash
   xhost +local:docker
   ```
3. Launch the container:
   ```bash
   docker compose -f docker/docker-compose.yaml run --rm fep_rl bash
   ```
4. Inside the container, the workspace at `/catkin_ws` is already built during the image build. Activate it with:
   ```bash
   source /catkin_ws/devel/setup.bash
   ```
   For development against the mounted repository at `/code`, rebuild as needed from `/catkin_ws` (the workspace links to `/code/fep_rl_experiment`):
   ```bash
   cd /catkin_ws
   catkin build
   ```
5. Bring up the simulation or hardware launch files as described in the next section. The repository is mounted at `/code`, so edits persist back to the host.

> **Tip:** If you need GPU acceleration inside Docker, ensure the NVIDIA Container Toolkit is installed and that the compose file’s `NVIDIA_*` environment variables match your driver capability.
> **Ports:** The default compose file maps UDP ranges `20210-20230` and `33300-33400` for streaming and teleoperation utilities. Adjust these if they conflict with services already running on your host.

## Running Experiments

With your workspace sourced (`source devel/setup.bash`), you can start the main launch files:

- **Simulation:** `roslaunch fep_rl_experiment bringup_sim.launch`
- **Real robot (default IP 172.16.1.11):**
  ```bash
  roslaunch fep_rl_experiment bringup_real.launch \
    robot_ip:=172.16.1.11 \
    sessionId:=session_0 \
    markerSize:=0.042 \
    cubeSize:=0.05
  ```
- **Training helper script:** `./scripts/train_training.bash <username> <host> [session_id]`

Experiment metrics are logged under `experiment_sessions/` and reward telemetry is published on `/instant_reward` and `/episode_reward`.

## Manual Installation

If you prefer to work natively, follow the detailed instructions in `setup/README.software.md`. They cover ROS Noetic installation, RealSense kernel modules, Python dependencies, catkin workspace creation, and sanity checks for both simulation and real hardware bring-up.

## Project Layout

- `fep_rl_experiment/launch`: ROS launch files for simulation, real-robot operation, and cube detection.
- `fep_rl_experiment/nodes`: Python entry points for the robot interface, dummy publishers, and online learning orchestration.
- `fep_rl_experiment/src/fep_rl_experiment`: Core environment, robot abstractions, logging utilities, and the ONNX-based transition server.
- `setup/`: Documentation for hardware and software preparation.
- `docker/`: Container recipes for reproducible builds.

## License

This project is released under the MIT License; see `LICENSE` for details.
