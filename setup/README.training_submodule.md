# Training Submodule Integration Guide

This guide explains how to bring the `training-submodule` research code into the Franka Emika Panda online RL workspace. The training stack lives in a separate Git submodule and uses its own Python environment so that robotics dependencies do not leak into the GPU training image.

## 1. Check Out the Submodule

```bash
git submodule update --init --recursive
```

The submodule is placed under `external/training-submodule`. After cloning you can work on it like any other repository (create branches, push commits, etc.).

## 2. Build the Docker Images

The compose file now ships two services:

- `fep_rl`: ROS Noetic container (unchanged) used for robot bring-up and data collection.
- `training_submodule`: CUDA/JAX container used for Brax training and policy optimisation.

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
python -m pip install --upgrade pip
# Install the same Python packages listed in setup/README.software.md.

# Training stack (uses uv sync)
cd external/training-submodule
uv python install 3.11.6  # installs the interpreter if it's missing
uv sync --no-dev
source .venv/bin/activate
```

> **Note:** Install [`uv`](https://example.com/anonymous/uv) if it is not already available (`curl -LsSf https://astral.sh/uv/install.sh | sh`).

## 3. Remote-First Training Workflow

Policy optimisation is typically executed on a GPU server while the robot-side transition server stays on the lab workstation.

1. On the robot workstation (or simulation host) launch the ROS container:
   ```bash
   docker compose -f docker/docker-compose.yaml up fep_rl
   ```
   The transition server binds to `tcp://*:5559` (also exposed on the host).

2. To make the ZMQ endpoint reachable on a remote trainer, start the reverse SSH tunnel helper from the repository root:
   ```bash
   ./scripts/remote_training.bash <gpu_user> <gpu_host> [session_id]
   ```
   This command forwards the lab machine’s port `5559` to `localhost:5555` on the remote machine and launches the bring-up launch file with the given session id.

3. On the remote GPU server (inside the `training_submodule` container) point training jobs to `tcp://localhost:5555`, e.g.:
   ```bash
   docker compose -f docker/docker-compose.yaml run --rm training_submodule \
     python train_brax.py --transition-endpoint tcp://localhost:5555
   ```
   Adjust the CLI flag/environment variable to match the argument expected by `train_brax.py`. When running on the same machine you can skip SSH tunnelling and use `tcp://host.docker.internal:5559`.

## 4. Accelerator Reference

The `training-submodule` repository depends on [`accelerator_repo`](https://example.com/anonymous/accelerator-repo) for Brax acceleration. Build instructions and CUDA compatibility notes live in those upstream repositories—follow their docs to install the extension on your target machine. Any helper scripts or wheel caches we maintain will be shipped inside `training-submodule` so they stay versioned with the training code.
