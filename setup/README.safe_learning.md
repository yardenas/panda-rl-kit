# Safe-Learning Integration Guide

This guide explains how to bring the `safe-learning` research code into the Franka Emika Panda online RL workspace. The training stack lives in a separate Git submodule and uses its own Python environment so that robotics dependencies do not leak into the GPU training image.

## 1. Check Out the Submodule

```bash
git submodule update --init --recursive
```

The submodule is placed under `external/safe-learning`. After cloning you can work on it like any other repository (create branches, push commits, etc.).

## 2. Build the Docker Images

The compose file now ships two services:

- `fep_rl`: ROS Noetic container (unchanged) used for robot bring-up and data collection.
- `safe_learning`: CUDA/JAX container used for Brax training and policy optimisation.

You can build both images in one command:

```bash
docker compose -f docker/docker-compose.yaml build
```

### Optional: Local Virtual Environments

If you prefer to work outside Docker, keep the package sets isolated:

```bash
# ROS / robot stack
python3 -m venv .venv-fep
source .venv-fep/bin/activate
pip install \
  numpy==1.21.6 \
  onnxruntime>=1.16.0 \
  pyzmq>=25.1.0 \
  opencv-python>=4.7.0 \
  matplotlib>=3.5.0

# Training stack
cd external/safe-learning
python3 -m venv .venv-safe
source .venv-safe/bin/activate
pip install -r requirements.txt  # or follow the repo-specific instructions
```

## 3. Remote-First Training Workflow

Policy optimisation is typically executed on a GPU server while the robot-side transition server stays on the lab workstation.

1. On the robot workstation (or simulation host) launch the ROS container:
   ```bash
   docker compose -f docker/docker-compose.yaml up fep_rl
   ```
   The transition server binds to `tcp://*:5559` (also exposed on the host).

2. To make the ZMQ endpoint reachable on a remote trainer, start the reverse SSH tunnel helper from the repository root:
   ```bash
   ./scripts/train_training.bash <gpu_user> <gpu_host> [session_id]
   ```
   This command forwards the lab machine’s port `5559` to `localhost:5555` on the remote machine and launches the bring-up launch file with the given session id.

3. On the remote GPU server (inside the `safe_learning` container) point training jobs to `tcp://localhost:5555`, e.g.:
   ```bash
   docker compose -f docker/docker-compose.yaml run --rm safe_learning \
     python train_brax.py --transition-endpoint tcp://localhost:5555
   ```
   Adjust the CLI flag/environment variable to match the argument expected by `train_brax.py`. When running on the same machine you can skip SSH tunnelling and use `tcp://host.docker.internal:5559`.

## 4. `mjx_madrona` Reference

The `safe-learning` repository depends on [`madrona_mjx`](https://github.com/shacklettbp/madrona_mjx) for Brax acceleration. Build instructions and CUDA compatibility notes live in those upstream repositories—follow their docs to install the extension on your target machine. Any helper scripts or wheel caches we maintain will be shipped inside `safe-learning` so they stay versioned with the training code.
