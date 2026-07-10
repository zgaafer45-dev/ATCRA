# ATCRA

ATCRA is a ROS 2 workspace for an automated tool-changing robotic arm (ATCRA). It contains the robot's description/configuration package, the hardware interface package, and two MoveIt packages — one for simulation and one for driving the real hardware.

## Package Overview

### `sabry`
The robot's description package. It holds the URDF/Xacro files, meshes, and general robot configuration used by the other packages. This package doesn't run any nodes on its own — it's the shared source of truth for the robot's kinematic and visual description, and is included by both the simulation and hardware MoveIt packages.

### `sabry_hardware`
Responsible for the physical robot interface. This package handles:
- **Tool changing** — the logic and control needed to switch end-effector tools on the arm.
- **Hardware interface** — the `ros2_control` hardware interface that talks to the real robot/controllers.
- **Actions and services** — the custom actions and services used to command tool changes and other hardware-level behaviors from the rest of the stack.

### `sabry_moveit`
The MoveIt configuration package used for **simulated results**. It brings up MoveIt with a simulated robot (no physical hardware required), and is used to test motion planning and pipeline logic before running on the real arm. Workpiece coordinates can be published manually here for testing.

### `sabrydemo_moveit`
The MoveIt configuration package used for the **hardware connection**. It brings up MoveIt configured to talk to the real robot through `sabry_hardware`, and is the entry point for running the full system on physical hardware.

## Prerequisites
- ROS 2 Humble
- MoveIt 2
- A working `colcon` workspace with this repo cloned into `src/`

Build the workspace before running anything:
```bash
colcon build
source install/setup.bash
```

## Running on Real Hardware

Run the following in order, each in its own sourced terminal:

1. **Launch the hardware MoveIt demo**
   ```bash
   ros2 launch sabrydemo_moveit demo.launch.py
   ```
2. **Bring up the rest of the system**
   ```bash
   ros2 launch sabry_bringup sabry_bringup.launch.xml
   ```
3. **Start the camera / inspection node**
   ```bash
   ros2 run sabry_hardware inspection_v3.py
   ```
   This node handles vision-based inspection and provides the detected workpiece data to the rest of the pipeline.

## Running in Simulation

To test with simulated results instead of real hardware:

1. **Launch the simulated MoveIt demo**
   ```bash
   ros2 launch sabry_moveit demo.launch.py
   ```
2. **Bring up the rest of the system** (same as hardware)
   ```bash
   ros2 launch sabry_bringup sabry_bringup.launch.xml
   ```
3. **Publish workpiece coordinates manually** — since there's no camera running in simulation, publish the coordinates by hand to drive the pipeline:
   ```bash
   ros2 topic pub /workpiece_coordinates geometry_msgs/msg/Point "{x: 0.1, y: 0.1, z: 0.35}" --once
   ```

## Summary

| Mode | MoveIt Launch | Bringup | Vision Input |
|---|---|---|---|
| Hardware | `sabrydemo_moveit demo.launch.py` | `sabry_bringup.launch.xml` | `inspection_v3.py` camera node |
| Simulation | `sabry_moveit demo.launch.py` | `sabry_bringup.launch.xml` | Manual `ros2 topic pub /workpiece_coordinates` |
