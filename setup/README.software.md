# Software Setup

This document explains how to prepare an Ubuntu 20.04 workstation to reproduce the reinforcement-learning experiments on a real Franka Emika Panda arm. The instructions mirror the tooling bundled in the Docker image and emphasize reproducibility outside containers.

## 1. Base System

- Install **Ubuntu 20.04 LTS** with a kernel supported by the Franka Control Interface (FCI). Enable secure boot only if your kernel is properly signed; otherwise disable it to simplify RealSense and Franka driver installation.
- Create an administrative user with sudo privileges. All commands below assume you work from this account and have network access for package downloads.

Update the system:

```bash
sudo apt update
sudo apt upgrade
```

## 2. Install ROS Noetic Desktop Full

ROS Noetic is the target middleware. If ROS is already installed, skip to the next section.

```bash
sudo apt install curl gnupg lsb-release
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source /opt/ros/noetic/setup.bash
```

Initialize `rosdep` (run once per machine):

```bash
sudo rosdep init
rosdep update
```

## 3. System Dependencies

Install the additional tools and ROS packages used by the experiment launcher and auxiliary scripts:

```bash
sudo apt install \
    python3-pip python3-venv python3-rosdep python3-wstool python3-catkin-tools \
    build-essential cmake ccache git curl \
    assimp-utils libassimp-dev libboost-all-dev libccd-dev libeigen3-dev \
    liboctomap-dev libtinyxml-dev liburdfdom-dev libbenchmark-dev \
    libsasl2-dev libgmp3-dev libsnmp-dev \
    ros-noetic-libfranka ros-noetic-franka-ros \
    ros-noetic-moveit ros-noetic-rosparam-shortcuts \
    ros-noetic-realsense2-camera ros-noetic-graph-msgs \
    ros-noetic-joy ros-noetic-joy-teleop \
    ros-noetic-plotjuggler ros-noetic-code-coverage ros-noetic-spacenav-node \
    python3-tk psutils apt-transport-https
```

### RealSense Firmware and Kernel Modules

Intel’s upstream repository provides the official `librealsense` kernel modules. Add the repository and install the packages:

```bash
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp >/dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | \
  sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt update
sudo apt install librealsense2-dkms librealsense2-utils
```

Reboot if prompted so the DKMS modules load correctly.

## 4. Python Environment

Create a virtual environment (recommended) and install the Python dependencies shared with the Docker image:

```bash
python3 -m venv ~/fep_env
source ~/fep_env/bin/activate
python -m pip install --upgrade pip
pip install \
  numpy==1.21.6 \
  onnxruntime>=1.16.0 \
  pyzmq>=25.1.0 \
  opencv-python>=4.7.0 \
  matplotlib>=3.5.0
```

When you launch ROS, ensure the virtual environment is active **before** you source your workspace’s `setup.bash` so that the correct interpreter is used.

## 5. Catkin Workspace

Set up a catkin workspace (adapt paths to your preference):

```bash
mkdir -p ~/fep_ws/src
cd ~/fep_ws
catkin config --init --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release
cd src
git clone <repository-url>
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin build
```

Replace `<repository-url>` with the HTTPS or SSH clone URL of this repository.

Source the workspace when the build completes:

```bash
source ~/fep_ws/devel/setup.bash
```

Add the source command to `~/.bashrc` if you intend to work with this workspace regularly.

## 6. Sanity Checks

1. **Simulation Bring-up:**
   ```bash
   roslaunch fep_rl_experiment bringup_sim.launch
   ```
   RViz should display the Panda model, simulated depth images, and dummy cube detections.

2. **Real Robot Bring-up:**
   ```bash
   roslaunch fep_rl_experiment bringup_real.launch robot_ip:=172.16.1.11 sessionId:=session_0
   ```
   Adjust the `robot_ip`, `markerSize`, `cubeSize`, and RealSense serial parameters as needed for your hardware.

3. **Training Session Proxy (optional):**
   ```bash
   ./scripts/remote_training.bash <username> <remote-host> session_1
   ```
   This opens an SSH reverse tunnel and launches the online learning node.

Experiment summaries and logs are stored under `experiment_sessions/` in the workspace by default. Monitor the `/instant_reward` and `/episode_reward` topics to verify policy health while training.

You are now ready to proceed with the Docker-based workflow or manual experimentation described in the root README.
