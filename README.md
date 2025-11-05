# Franka Emika Panda Online RL

This repository hosts the ROS packages, launch files, and training utilities we use to run online reinforcement-learning experiments on a real Franka Emika Panda manipulator. The goal is to make it straightforward to reproduce the hardware setup, bring up the robot, and iterate on policies either directly on hardware or in simulation.

## Repository Highlights
- `fep_rl_experiment`: ROS package for robot bring-up, cube detection, experiment logging, and the online learning node.
- `scripts/remote_training.bash`: helper script that creates an SSH reverse tunnel and launches online learning with a custom session identifier. Run this script on the workstation that has direct network access to the robot controller.
- `docker/`: Dockerfile and compose configuration for a fully reproducible runtime environment with ROS Noetic and Intel RealSense support.
- `setup/`: step-by-step hardware and software guides for preparing a lab workstation without containers.
- `external/safe-learning`: Git submodule with the Brax/JAX training stack (policy optimisation and safety-critical RL).

## Before You Start

1. Review the hardware checklist in `setup/README.hardware.md`.
2. Prepare your Ubuntu 20.04 workstation following `setup/README.software.md` **or** use the Docker workflow below.
3. Verify that your Franka Emika Panda has an active FCI license and that you can ping the robot control cabinet from your workstation.

## GPU Requirements 

The CUDA-enabled training stack (`safe_learning`) has been validated with the following NVIDIA components. Make sure your GPU workstation matches these versions (or newer) before launching the trainer:

- NVIDIA driver `555.42.06`
- CUDA runtime `12.5`
- cuDNN `9.10.2` for CUDA 12

You can confirm the driver and CUDA versions with `nvidia-smi`; cuDNN is reported by the installed `nvidia-cudnn-cu12` package inside the training environment. These requirements are needed only for policy training, the robot host is not required to have them.

## Docker Workflow

The Docker environment mirrors the system dependencies described in the manual setup guides while isolating ROS and Python packages.

1. Make sure the `safe-learning` submodule is available (only needed the first time or after cleaning the checkout):
   ```bash
   git submodule update --init --recursive
   ```
2. Build the image (run from the repository root):
   ```bash
   docker compose -f docker/docker-compose.yaml build
   ```
3. Allow container access to the X server if you plan to run RViz from the container (Linux host):
   ```bash
   xhost +local:docker
   ```
4. Launch the container:
   ```bash
   docker compose -f docker/docker-compose.yaml run --rm fep_rl bash
   ```
5. Inside the container, the workspace at `/catkin_ws` is already built during the image build. Activate it with:
   ```bash
   source /catkin_ws/devel/setup.bash
   ```
   For development against the mounted repository at `/code`, rebuild as needed from `/catkin_ws` (the workspace links to `/code/fep_rl_experiment`):
   ```bash
   cd /catkin_ws
   catkin build
   ```
6. Bring up the simulation or hardware launch files as described in the next section. The repository is mounted at `/code`, so edits persist back to the host.

> **Tip:** If you need GPU acceleration inside Docker, ensure the NVIDIA Container Toolkit is installed and that the compose file’s `NVIDIA_*` environment variables match your driver capability.
> **Ports:** The default compose file maps UDP ranges `20210-20230` and `33300-33400` for streaming and teleoperation utilities. Adjust these if they conflict with services already running on your host.

## Safe-Learning Trainer

The Brax/JAX trainer lives in the `external/safe-learning` submodule. Pull it with:

```bash
git submodule update --init --recursive
```

The docker compose file now exposes two services:

- `fep_rl`: robot-side ROS / sampling stack (unchanged).
- `safe_learning`: CUDA-enabled training environment. It installs dependencies from the submodule so it can evolve independently of ROS packages.

Build both images after checking out the submodule:

```bash
docker compose -f docker/docker-compose.yaml build
```

To run the trainer on the same machine, launch both services and point `train_brax.py` at the exposed transition server endpoint (`tcp://host.docker.internal:5559`). When offloading training to a remote GPU machine, use `./scripts/remote_training.bash` to open a reverse tunnel (`remote:5555 -> local:5559`) and connect the trainer via `tcp://localhost:5555`. Run this helper on the workstation that talks to the robot so the tunnel originates from the host that can reach the transition server. The training service can be started with:

```bash
docker compose -f docker/docker-compose.yaml run --rm safe_learning bash
```

### Remote Trainer via Reverse SSH

When the robot and GPU trainer live on different networks you need a reverse SSH tunnel so the remote machine can reach the transition server that runs next to the robot.

1. **Pick tunnel details:** choose an open port on the remote GPU host (default `5555`) and make sure inbound connections to that port are allowed by its firewall.
2. **Start the tunnel from the robot workstation:**
   ```bash
   ./scripts/remote_training.bash <gpu_user> <gpu_host> [session_id]
   ```
   The helper runs `ssh -R 5555:localhost:5559 <gpu_user>@<gpu_host>` in the background, launches `bringup_real.launch`, and cleans up the tunnel when you press `Ctrl+C`. Override the port by editing the script or by running the raw SSH command yourself.
3. **Manually launching (optional):** if you prefer to manage the ROS bring-up separately, first create the tunnel:
   ```bash
   ssh -R <remote_port>:localhost:5559 <gpu_user>@<gpu_host> -N
   ```
   Then, in another terminal on the robot workstation, start `roslaunch fep_rl_experiment bringup_real.launch`.
4. **Run the trainer on the remote GPU:** point your training job at `tcp://localhost:<remote_port>`; for example:
   ```bash
   docker compose -f docker/docker-compose.yaml run --rm safe_learning \
     python train_brax.py --transition-endpoint tcp://localhost:5555
   ```

Keep the SSH session open while training—closing it tears down the tunnel and the trainer will stop receiving transitions.

For in-depth guidance—including native virtual environments, remote trainer instructions, and pointers to the `safe-learning` and `madrona_mjx` installation docs—see `setup/README.safe_learning.md`.

### Pretraining a Prior Policy

1. **Train the prior in simulation:** on the training machine run:
   ```bash
   python train_brax.py +experiment=franka_sim_to_real
   ```
   The job logs to Weights & Biases; when it finishes, open the run in the W&B UI and copy the Run ID (the short hash shown under the run name, or the final path element of the run URL).
2. **Record the W&B run id:** the `franka_sim_to_real` job uploads its replay buffer to W&B.
   You'll reuse this identifier when launching online training.

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
- **Training helper script:** `./scripts/remote_training.bash <username> <host> [session_id]`
- **Online training with the pretrained prior:** run from the training machine, using the W&B ids from pretraining:
  ```bash
  python train_brax.py +experiment=franka_online \
    training.wandb_id=<wandb_run_id> \
    agent.replay_buffer.wandb_ids='[<wandb_run_id>]'
  ```
  Replace `<wandb_run_id>` with the Run ID from the simulation job (for example `username/project/abc123`). The online run resumes logging to the existing W&B project while seeding the policy and replay buffer with the simulated prior.

Experiment metrics are logged under `experiment_sessions/` and reward telemetry is published on `/instant_reward` and `/episode_reward`.

## Manual Installation

If you prefer to work natively, follow the detailed instructions in `setup/README.software.md`. They cover ROS Noetic installation, RealSense kernel modules, Python dependencies, catkin workspace creation, and sanity checks for both simulation and real hardware bring-up.

That document also lists the Python packages that mirror the Docker image.

## Project Layout

- `fep_rl_experiment/launch`: ROS launch files for simulation, real-robot operation, and cube detection.
- `fep_rl_experiment/nodes`: Python entry points for the robot interface, dummy publishers, and online learning orchestration.
- `fep_rl_experiment/src/fep_rl_experiment`: Core environment, robot abstractions, logging utilities, and the ONNX-based transition server.
- `setup/`: Documentation for hardware and software preparation.
- `docker/`: Container recipes for reproducible builds.

## License

This project is released under the MIT License; see `LICENSE` for details.
