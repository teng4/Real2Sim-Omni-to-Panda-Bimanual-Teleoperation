# Real2Sim-Omni-to-Panda-Bimanual-Teleoperation
Real2Sim Omni-to-Panda Bimanual Teleoperation System.

## Programming Environment and Robots

(1) ROS;

(2) C++;

(3) Real robots: Two Phantom Omni haptic devices;

(4) Sim robots: Two 7DOF Franka Emika Panda robots;

(5) Ubuntu 20.04.6 LTS



## How to use:

1. Download the whole project, which is a typical `catkin_ws`.
2. Unzip the `panda1.zip` and `panda2.zip`, which are the CAD model files for visualization.
3. `catkin_make`
4. `roslaunch real2simpkg omni_panda_v1.launch`
5. A screenshot when this project running is illustrated below


# About:
1. This script is the demo code for `real2sim Omni-to-Panda bimanual teleoperation`.
1. All the tools/robots named with `omni11` or `cleft11` are on the righ-hand side (R).
     All the tools/robots named with `omni12` or `cleft12` are on the left-hand side (L).
1. Assume the real omni devices are publishing their real-time 6D pose data + button state [x,y,z,roll,pitch,yaw,btnState] to ROS topic via
      customized msg type [teng4pkg_msgs/Vector7DOF, Vector7DOF.msg={a1,a2,a3,a4,a5,a6,a7=btn}.]
       Omni11(R) is publishing data to `/ooomni11/omni11_vector7dof`, 1000 Hz.
       Omni12(L) is publishing data to `/ooomni11/omni11_vector7dof`, 1000 Hz.
1. (optional) For visualizing the sim tool or sim panda in rviz, the following ROS topics are used:
       panda1(R) joint states are publishing to `/panda1/joint_states`, 1000 Hz.
       panda2(L) joint states are publishing to `/panda2/joint_states`, 1000 Hz.
       Note, visualization is not mandatory, and it is independent of the real2sim framework (i.e., this script).
       You can also visualize the sim tool or sim panda in other visualization platforms, e.g., Unity.
1. The DH table for the two sim tools can be specified as different ones, using "paramR" and "paramL" to customize in the main function.
1. The initial pose of the sim tools can be customized via variables `q0_initCleft11` and `q0_initCleft12` in joint space.
      On the other side, the initial pose of the real robots (i.e., omni devices in this case) matters a lot, 
      since its pose will be recorded as its own home pose at the moment you run this script. 
      For instance, if the sim tool EE is customized to init with a vertical straight down pose, in order to make the real robot have the same home pose 
      (which can provide a more intuitive teleoperation), you need to manually put the omni stylus to be in a vertical straight down pose right before you run this script.

------
Created on 2025-07-10 17:04.
