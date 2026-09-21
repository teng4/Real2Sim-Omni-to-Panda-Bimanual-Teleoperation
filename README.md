# Real2Sim-Omni-to-Panda-Bimanual-Teleoperation
Real2Sim Omni-to-Panda Bimanual Teleoperation.

A Real2Sim bimanual teleoperation system that uses two 6-DOF Phantom Omni haptic devices as physical leader robots to teleoperate two simulated 7-DOF Franka Emika Panda robots.

This repository provides a demonstration of the Real2Sim teleoperation framework described in our paper:

> Teng Li, Sunny Zhang, Cate Balasubramanian, Thomas Looi, and Dale J. Podolsky.  
> **“A Configurable Real2Sim Bimanual Teleoperation Framework for Surgical Robotic Tool Design and Evaluation.”**  
> *The International Journal of Medical Robotics and Computer Assisted Surgery*, 2026, e70235.  
> DOI: https://doi.org/10.1002/rcs.70235

---

## System Configuration

### Physical Leader Robots

- Two 6-DOF Phantom Omni haptic devices
- Right leader: `omni11`
- Left leader: `omni12`

### Simulated Follower Robots

- Two 7-DOF Franka Emika Panda robots
- Right follower: `panda1`
- Left follower: `panda2`

### Software Environment

- Ubuntu 20.04.6 LTS
- ROS 1
- C++
- RViz for visualization

---

## Installation and Usage

1. Clone or download the entire repository. The project is organized as a standard ROS `catkin_ws` workspace.

2. Unzip `panda1.zip` and `panda2.zip`. These packages contain the URDF files used to visualize the two Panda robots in RViz.

3. Build the workspace:

   ```bash
   catkin_make
   ```

4. Make sure that the two physical leader robots are publishing their real-time Cartesian poses and button states to the required ROS topics.

5. Launch the bimanual Real2Sim teleoperation system:

   ```bash
   roslaunch real2simpkg omni_panda_biteleop_v1.launch
   ```

6. An example of the system running in RViz is shown below:

<p align="center">
  <img src="https://github.com/teng4/Real2Sim-Omni-to-Panda-Bimanual-Teleoperation/blob/47cb066351eb1458e4b4867dbe08e369a2febc42/images/real2sim_omni2panda_bitele_v3ok_image1.jpg" width="60%">
</p>

### Demo Video

[Watch the demo video on YouTube](https://youtu.be/JBzOnDclYlc)

---

## Implementation Details

### 1. Robot Naming Convention

The following naming convention is used throughout the project:

| Robot/Tool | Side |
|---|---|
| `omni11`, `cleft11`, `panda1` | Right (R) |
| `omni12`, `cleft12`, `panda2` | Left (L) |

---

### 2. Leader Robot Pose Input

The framework assumes that the physical Phantom Omni devices publish their real-time 6D Cartesian poses and button states in the following form:

```text
[x, y, z, roll, pitch, yaw, btnState]
```

A customized ROS message type is used:

```text
teng4pkg_msgs/Vector7DOF
```

with

```text
Vector7DOF.msg = {a1, a2, a3, a4, a5, a6, a7}
```

where `a7` represents the button state.

The ROS topics used in the current implementation are:

```text
Right Omni (omni11):
/ooomni11/omni11_vector7dof

Left Omni (omni12):
/ooomni12/omni12_vector7dof
```

The leader data are published at **1000 Hz**.

---

### 3. RViz Visualization

Visualization of the simulated robots in RViz is optional and is independent of the core Real2Sim teleoperation framework.

The simulated Panda joint states are published to:

```text
Right Panda (panda1):
/panda1/joint_states

Left Panda (panda2):
/panda2/joint_states
```

The joint states are published at **1000 Hz** in the current implementation.

Other visualization or simulation platforms, such as Unity, may also be used with the Real2Sim framework.

---

### 4. Customizing the Follower Kinematics

The kinematic parameters of the two simulated followers can be independently configured.

The Denavit–Hartenberg (DH) parameters for the right and left followers can be specified using:

```cpp
paramR
paramL
```

This allows the framework to accommodate different follower kinematic configurations.

---

### 5. Customizing the Initial Follower Configuration

The initial joint configurations of the two simulated followers can be customized using:

```cpp
q0_initCleft11
q0_initCleft12
```

These parameters allow the user to initialize the simulated followers at desired poses.

The initial pose of each physical leader robot is recorded as its home pose when the teleoperation program starts. Therefore, the physical leader should preferably be placed in an intuitive initial configuration before launching the system.

For example, if the end effector (EE) of a simulated follower is initialized in a vertically downward orientation, the corresponding Omni stylus can also be manually placed in a vertically downward orientation immediately before launching the teleoperation program. This provides a more intuitive initial leader–follower pose correspondence.

---

## Citation

If you find this project helpful for your research, please consider citing the following paper.

```text
Teng Li, Sunny Zhang, Cate Balasubramanian, Thomas Looi, and Dale J. Podolsky.
2026. "A Configurable Real2Sim Bimanual Teleoperation Framework for Surgical
Robotic Tool Design and Evaluation."
The International Journal of Medical Robotics and Computer Assisted Surgery:
e70235.
https://doi.org/10.1002/rcs.70235
```

---

## License

This project is distributed under the BSD 3-Clause License. See the `LICENSE` file for details.

---

## Author

**Teng Li**

Created: 2025-07-10 17:04. 
Last updated: September 2026
