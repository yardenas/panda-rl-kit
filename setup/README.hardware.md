# Hardware Setup

This guide documents the physical setup that was used to collect data and run the reinforcement-learning experiments showcased in this repository. Following these steps will get your Franka Emika Panda workspace ready for the software stack described in `setup/README.software.md`.

## Bill of Materials

- Franka Emika Panda 7-DoF arm with a valid FCI (Franka Control Interface) license and control cabinet.
- External control workstation (tested with Ubuntu 20.04) with at least 4 CPU cores, 16 GB RAM, and a dedicated NVIDIA GPU if you plan to train policies locally.
- Intel RealSense depth camera (D435/D455 family tested) with USB 3.0 cable and a rigid mount looking at the manipulation workspace.
- Printed AprilTag or ArUco marker board. The default launch files expect single ArUco markers with IDs `571 581 591 601 611 621` at 42 mm edge length.
- Cubic grasp target (50 mm edge length) with matte surface finish for reliable pose estimation.
- Stable workbench that can anchor the Panda base and safely accommodate the camera and marker target.
- Physical E-stop button within reach of the operator and clear line of sight to the robot.

## Installation and Wiring

1. **Mount the robot base** on the workbench following Franka Emika’s official mechanical instructions. Double-check torque values on all anchor bolts.
2. **Connect power and emergency-stop loop** according to the Panda user manual. Verify that the control cabinet boots correctly and that the E-stop triggers as expected.
3. **Network topology:** connect the external control workstation directly to the Panda control cabinet via Ethernet. The default ROS launch files assume the cabinet uses IP `172.16.1.11`; set a static IP for the workstation on the same subnet (e.g. `172.16.1.2/24`) and confirm connectivity with `ping 172.16.1.11`.
4. **RealSense camera:** mount the camera rigidly to a tripod or frame so it observes the workspace from above at roughly a 45° angle. Secure the USB 3.0 cable to avoid tugging on the device during robot motion.
5. **Marker board placement:** place the ArUco board flat on the table inside the camera view and clamp it to prevent movement. Keep glossy surfaces away from the markers to reduce reflections.

## Geometric Calibration

The launch files ship with a static transform from the Panda base frame (`panda_link0`) to the camera frame (`camera_link`). Once your hardware is installed:

1. Launch only the camera and the static transform publisher:
   ```bash
   roslaunch fep_rl_experiment bringup_real.launch robot_ip:=<your_robot_ip> sessionId:=calib_test
   ```
2. Inspect the transform in RViz. If the camera pose differs from the default (~0.9 m above and slightly offset from the base), edit the `args` of the `static_transform_publisher` in `fep_rl_experiment/launch/bringup_real.launch`.
3. Update the ArUco marker size (`markerSize`) and cube size (`cubeSize`) arguments if you use different targets.

## Operational Safety Checklist

- Keep the Panda’s workspace clear and familiarize yourself with the arm’s collision and joint limits before running experiments.
- Verify that the robot is in gravity-compensation mode and that the brakes disengage cleanly before enabling autonomous control.
- Always start experiments at low speed and monitor the `instant_reward` and `episode_reward` topics to ensure the policy behaves as expected.
- Maintain a clear path to the E-stop and be ready to disable power at the control cabinet if the robot moves unexpectedly.

Proceed to `setup/README.software.md` once the physical installation is complete.
