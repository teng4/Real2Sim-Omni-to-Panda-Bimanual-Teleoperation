# Real2Sim-Omni-to-Panda-Bimanual-Teleoperation
Real2Sim Omni-to-Panda Bimanual Teleoperation System.

## Programming Environment and Robots

(1) Real robots: Two 6-DOF Phantom Omni haptic devices;

(2) Sim robots: Two 7-DOF Franka Emika Panda robots;

(3) Ubuntu 20.04.6 LTS

(4) ROS;

(5) C++;



## How to use:

1. Download the whole project, which is a typical `catkin_ws`.
2. Unzip the `panda1.zip` and `panda2.zip`, which are the URDF files for visualization in RVIZ.
3. `catkin_make`
4. Assume the physical master robots have been publishing their Cartesian pose msg to ROS topics.
5. `roslaunch real2simpkg omni_panda_biteleop_v1.launch`
6. A screenshot when running this project is illustrated below
<img src="https://github.com/teng4/Real2Sim-Omni-to-Panda-Bimanual-Teleoperation/blob/47cb066351eb1458e4b4867dbe08e369a2febc42/images/real2sim_omni2panda_bitele_v3ok_image1.jpg" width="50%" height="50%">

[[watch the demo video](https://youtu.be/JBzOnDclYlc)]


# About:
1. This code project is the demo code for the paper `Teng Li, Sunny Zhang, Cate Balasubramanian, Thomas Looi, and Dale J Podolsky. “A General Real2Sim Bimanual Teleoperation Framework for Evaluating Surgical Robotic Tool Design”. 2025, pp.1-10. [Under Review]`.
1. All the tools/robots named with `omni11` or `cleft11` are on the righ-hand side (R).
     All the tools/robots named with `omni12` or `cleft12` are on the left-hand side (L).
1. Assume the real omni devices are publishing their real-time 6D pose data + button state [x,y,z,roll,pitch,yaw,btnState] to ROS topic via
      customized msg type [teng4pkg_msgs/Vector7DOF, Vector7DOF.msg={a1,a2,a3,a4,a5,a6,a7=btn}.]
       Omni11(R) is publishing data to `/ooomni11/omni11_vector7dof`, 1000 Hz.
       Omni12(L) is publishing data to `/ooomni12/omni12_vector7dof`, 1000 Hz.
1. (optional) For visualizing the sim tool or sim panda in rviz, the following ROS topics are used:
       panda1(R) joint states are publishing to `/panda1/joint_states`, 1000 Hz.
       panda2(L) joint states are publishing to `/panda2/joint_states`, 1000 Hz.
       Note, visualization is not mandatory, and it is independent of the real2sim framework (i.e., this script).
       You can also visualize the sim tool or sim panda in other visualization platforms, e.g., Unity.
1. The DH table for the two sim tools can be specified as different ones, using "paramR" and "paramL" to customize in the main function.
1. The initial pose of the sim tools can be customized via variables "q0_initCleft11" and "q0_initCleft12" in joint space. 
       This adjustable initial joint configuration of sim tools allow user to set the sim tools with any initial pose the user expected. 
       On the other hand, the initial pose of the real robots (i.e., omni devices in this case) may matter, 
       because its pose will be recorded as its own home pose at the moment you run this script.
       For instance, if the sim tool EE is initialized with a vertical straight down pose, in order to make the real robot have the same home pose 
       (which can provide a more intuitive teleoperation), you need to manually put the omni stylus to be in a vertical straight down pose right before you run this script.


------
Created on 2025-07-10 17:04.
