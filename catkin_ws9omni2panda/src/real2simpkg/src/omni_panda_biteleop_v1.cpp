// Copyright (c) 2026 Teng Li
// Creator: Teng Li (teng4@ualberta.ca)

// About (20260317.1229):
// (1) This script is the demo code for real2sim Omni-to-Panda bimanual teleoperation.
// (2) A demo video for this script can be found at: https://youtu.be/JBzOnDclYlc
// (3) All the tools/robots named with "omni11" or "cleft11" are on the righ-hand side (R).
//     All the tools/robots named with "omni12" or "cleft12" are on the left-hand side (L).
// (4) Assume the real omni devices are publishing their real-time 6D pose data + button state [x,y,z,roll,pitch,yaw,btnState] to ROS topic with
//      customized msg type [teng4pkg_msgs/Vector7DOF, Vector7DOF.msg={a1,a2,a3,a4,a5,a6,a7=btn}.]
//       Omni11(R) is publishing data to "/ooomni11/omni11_vector7dof", 1000 Hz.
//       Omni12(L) is publishing data to "/ooomni12/omni12_vector7dof", 1000 Hz.
// (5) (optional) For visualizing the sim tool or sim panda in rviz, the following ROS topics are used:
//       panda1(R) joint states are publishing to "/panda1/joint_states", 1000 Hz.
//       panda2(L) joint states are publishing to "/panda2/joint_states", 1000 Hz.
//       Note, visualization is not mandatory, and it is independent of the real2sim framework (i.e., this script).
//       You can also visualize the sim tool or sim panda in other visualization platforms, e.g., Unity.
// (6) The DH table for the two sim tools can be specified as different ones, using "paramR" and "paramL" to customize in the main function.
// (7) The initial pose of the sim tools can be customized via variables "q0_initCleft11" and "q0_initCleft12" in joint space. 
//      This adjustable initial joint configuration of sim tools allow user to set the sim tools with any initial pose the user expected. 
//      On the other hand, the initial pose of the real robots (i.e., omni devices in this case) may matter, 
//      because its pose will be recorded as its own home pose at the moment you run this script.
//      For instance, if the sim tool EE is initialized with a vertical straight down pose, in order to make the real robot have the same home pose 
//      (which can provide a more intuitive teleoperation), you need to manually put the omni stylus to be in a vertical straight down pose right before you run this script.


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <sstream>

#include <algorithm>
#include <pthread.h>

#include "teng4pkg_msgs/Vector7DOF.h" 

#include <tf/transform_broadcaster.h> 
#include <Eigen/Dense> 

#include <random>

#include <iostream>
#include <fstream>
using namespace std;
const char *path1="/home/teng/catkin_ws9omni2panda/src/real2simpkg/src/data/data_output1.csv"; //output data to csv file.
const char *path2="/home/teng/catkin_ws9omni2panda/src/real2simpkg/src/data/data_output2.csv"; //output data to csv file.
ofstream myfile1(path1);
ofstream myfile2(path2);

double teng4_timer0_begin;
double teng4_timer0_now;
double teng4_timer0_elapsed;
double teng4_timer0_elapsed2;
double teng4_timer1_begin;
double teng4_timer1_now; 
double teng4_timer1_elapsed; 
double teng4_timer1_elapsed2;
double teng4_timer2_begin;
double teng4_timer2_now;
double teng4_timer2_elapsed;
double teng4_timer2_elapsed2;

const unsigned long interval = 10000000;

int flag_once1 = 0;
int flag_once2 = 0;

int flag_toFreezeOmni11 = 1;
int flag_toFreezeOmni12 = 1;

int flag_getOmni11PoseInit0 = 1;
int flag_getOmni12PoseInit0 = 1;

// set xyz limits for cleft tool (optional)
double x_min = -0.02;
double x_max = 0.02;
double y_min = -0.02;
double y_max = 0.02;
double z_min = 0.31;
double z_max = 0.35;


//joint limits for 7dof franka fr3 (Left-hand side).
//---------------------------------------
// % (https://frankarobotics.github.io/docs/control_parameters.html#limits-for-franka-research-3)
double q1_max =  2.7437; //% 157.2024
double q1_min = -2.7437; //%-157.2024
double q2_max =  1.7837; //% 102.1985
double q2_min = -1.7837; //%-102.1985
double q3_max =  2.9007; //% 166.1979
double q3_min = -2.9007; //%-166.1979
double q4_max = -0.1518; //%-8.6975
double q4_min = -3.0421; //%-174.2995
double q5_max =  2.8065; //% 160.8006
double q5_min = -2.8065; //%-160.8006
double q6_max =  4.5169; //% 258.7993
double q6_min =  0.5445; //% 31.1976
double q7_max =  3.0159; //% 172.7983
double q7_min = -3.0159; //%-172.7983
//---------------------------------------


//joint limits for 7dof franka fr3 (Right-hand side).
//---------------------------------------
// % (https://frankarobotics.github.io/docs/control_parameters.html#limits-for-franka-research-3)
double Rq1_max =  2.7437; //% 157.2024
double Rq1_min = -2.7437; //%-157.2024
double Rq2_max =  1.7837; //% 102.1985
double Rq2_min = -1.7837; //%-102.1985
double Rq3_max =  2.9007; //% 166.1979
double Rq3_min = -2.9007; //%-166.1979
double Rq4_max = -0.1518; //%-8.6975
double Rq4_min = -3.0421; //%-174.2995
double Rq5_max =  2.8065; //% 160.8006
double Rq5_min = -2.8065; //%-160.8006
double Rq6_max =  4.5169; //% 258.7993
double Rq6_min =  0.5445; //% 31.1976
double Rq7_max =  3.0159; //% 172.7983
double Rq7_min = -3.0159; //%-172.7983
//---------------------------------------

int msg_received_counter1 = 0;
int msg_received_counter2 = 0;

int omni11button0123 = 0;
int omni12button0123 = 0;

// omni related parameters.
Eigen::Matrix4d omni11_T_EEinit0 = Eigen::Matrix4d::Identity();
Eigen::Matrix3d omni11_R_EEinit0 = Eigen::Matrix3d::Identity();
Eigen::Vector3d omni11_t_EEinit0 = Eigen::Vector3d::Zero();

Eigen::Matrix4d omni11_T_EEnow_freezed = Eigen::Matrix4d::Identity();
Eigen::Matrix3d omni11_R_EEnow_freezed = Eigen::Matrix3d::Identity();
Eigen::Vector3d omni11_t_EEnow_freezed = Eigen::Vector3d::Zero();

Eigen::Matrix4d omni12_T_EEinit0 = Eigen::Matrix4d::Identity();
Eigen::Matrix3d omni12_R_EEinit0 = Eigen::Matrix3d::Identity();
Eigen::Vector3d omni12_t_EEinit0 = Eigen::Vector3d::Zero();

Eigen::Matrix4d omni12_T_EEnow_freezed = Eigen::Matrix4d::Identity();
Eigen::Matrix3d omni12_R_EEnow_freezed = Eigen::Matrix3d::Identity();
Eigen::Vector3d omni12_t_EEnow_freezed = Eigen::Vector3d::Zero();

// cleft tool 11 (R) related parameter.
Eigen::Matrix4d cleft11_T_EEinit0 = Eigen::Matrix4d::Identity();
Eigen::Matrix3d cleft11_R_EEinit0 = Eigen::Matrix3d::Identity();
Eigen::Vector3d cleft11_t_EEinit0 = Eigen::Vector3d::Zero();

Eigen::Matrix4d cleft11_T_EEnow = Eigen::Matrix4d::Identity();
Eigen::Matrix3d cleft11_R_EEnow = Eigen::Matrix3d::Identity();
Eigen::Vector3d cleft11_t_EEnow = Eigen::Vector3d::Zero();

Eigen::Matrix4d cleft11_T_EEnow_freezed = Eigen::Matrix4d::Identity();
Eigen::Matrix3d cleft11_R_EEnow_freezed = Eigen::Matrix3d::Identity();
Eigen::Vector3d cleft11_t_EEnow_freezed = Eigen::Vector3d::Zero();

Eigen::Matrix4d cleft11_T_EEdes = Eigen::Matrix4d::Identity();
Eigen::Matrix3d cleft11_R_EEdes = Eigen::Matrix3d::Identity();
Eigen::Vector3d cleft11_t_EEdes = Eigen::Vector3d::Zero();

// cleft tool 12 (L) related parameter.
Eigen::Matrix4d cleft12_T_EEinit0 = Eigen::Matrix4d::Identity();
Eigen::Matrix3d cleft12_R_EEinit0 = Eigen::Matrix3d::Identity();
Eigen::Vector3d cleft12_t_EEinit0 = Eigen::Vector3d::Zero();

Eigen::Matrix4d cleft12_T_EEnow = Eigen::Matrix4d::Identity();
Eigen::Matrix3d cleft12_R_EEnow = Eigen::Matrix3d::Identity();
Eigen::Vector3d cleft12_t_EEnow = Eigen::Vector3d::Zero();

Eigen::Matrix4d cleft12_T_EEnow_freezed = Eigen::Matrix4d::Identity();
Eigen::Matrix3d cleft12_R_EEnow_freezed = Eigen::Matrix3d::Identity();
Eigen::Vector3d cleft12_t_EEnow_freezed = Eigen::Vector3d::Zero();

Eigen::Matrix4d cleft12_T_EEdes = Eigen::Matrix4d::Identity();
Eigen::Matrix3d cleft12_R_EEdes = Eigen::Matrix3d::Identity();
Eigen::Vector3d cleft12_t_EEdes = Eigen::Vector3d::Zero();


// set scaling factor of position from Robot A to Tool B.
//--------------------------------------------------------
const double scale_pos_AtoB1 = 0.0005; //0.0005 for omni-panda teleop.
const double scale_pos_AtoB2 = 0.0005; //0.0005 for omni-panda teleop.
//--------------------------------------------------------

// set initial pose for the sim tools, in joint space (ensure numberOfElements = numberOfJoints). 
//--------------------------------------------------------
const std::vector<double> q0_initCleft11 = {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}; //Panda actual start position.
const std::vector<double> q0_initCleft12 = {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}; //Panda actual start position.
//--------------------------------------------------------


// transformation matrix from real robot A1 (R) base to sim tool B1 base.
const Eigen::Matrix4d T_mapRAB1 = (Eigen::Matrix4d() << //for omni -> 7dof panda actual init pose.
                                          -1.0,  0.0,  0.0, 0.0,
                                           0.0,  0.0,  1.0, 0.0,
                                           0.0,  1.0,  0.0, 0.0,
                                           0.0,  0.0,  0.0, 1.0).finished();
//
// transformation matrix from real robot A2 (L) base to sim tool B2 base.
const Eigen::Matrix4d T_mapRAB2 = (Eigen::Matrix4d() << // for omni -> 7dof panda FCI doc DH pose.
                                          1.0,  0.0, 0.0, 0.0,
                                          0.0,  0.0, 1.0, 0.0,
                                          0.0, -1.0, 0.0, 0.0,
                                          0.0,  0.0, 0.0, 1.0).finished();
//


std::vector<double> q0_seed_cleft11forsave(q0_initCleft11.size());
std::vector<double> q0_seed_cleft12forsave(q0_initCleft12.size());
std::vector<double> qik_forCheck_cleft11forsave(q0_initCleft11.size());
std::vector<double> qik_forCheck_cleft12forsave(q0_initCleft12.size());
std::vector<double> qik_checked_cleft11forsave(q0_initCleft11.size());
std::vector<double> qik_checked_cleft12forsave(q0_initCleft12.size());
std::vector<double> x_des_cleft11forsave(6); //Cartesian 6d pose
std::vector<double> x_des_cleft12forsave(6); //Cartesian 6d pose
std::vector<double> x_des_old_cleft11forsave(6); //Cartesian 6d pose
std::vector<double> x_des_old_cleft12forsave(6); //Cartesian 6d pose

std::vector<double> cleft11_q_now(q0_initCleft11.size());
std::vector<double> cleft12_q_now(q0_initCleft12.size());

//store final joint command.
std::vector<double> cleft11_q_cmd(q0_initCleft11.size());
std::vector<double> cleft12_q_cmd(q0_initCleft12.size());


struct DHParameter {
    double a, alpha, d, theta;
    bool is_prismatic;
};


struct ToolState {  
  // TODO: Define your parameters in this struct if needed
};


class Teng4ROS {

private:
    std::vector<DHParameter> dh_params;
    // Compute a single DH transformation matrix, modified convention.
    Eigen::Matrix4d dhTransform(double theta, double alpha, double a, double d) {
        Eigen::Matrix4d T;
        T << cos(theta), -sin(theta), 0, a,
             sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d,
             sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),  cos(alpha)*d,
             0, 0, 0, 1;
        return T;
    }

    //robot ID for init.
    std::string robot_name;
    std::vector<double> q0_init;

public:
  ros::NodeHandle nh;

  ros::Subscriber sub_omni11_pose6dof_btn;
  ros::Subscriber sub_omni12_pose6dof_btn;
 
  ros::Publisher pub_panda1_joint_states;
  ros::Publisher pub_panda2_joint_states;


  // Constructor default
  Teng4ROS() {}
  
  // // Constructor explicit 1
  explicit Teng4ROS(const std::vector<DHParameter> &params,
                  const std::vector<double> &q0,
                  const std::string& name)
    : dh_params(params), q0_init(q0), robot_name(name)
  {
    if (robot_name == "robot11R") 
    {
      ROS_INFO_STREAM("\nteng4note, an object created for " << robot_name << " \n");

      //[teng4pkg_msgs/Vector7DOF, Vector7DOF.msg={a1,a2,a3,a4,a5,a6,a7=btn}.]
      sub_omni11_pose6dof_btn = nh.subscribe("/ooomni11/omni11_vector7dof", 1, &Teng4ROS::omni11Pose6dofCallback, this);  

    }
    else if (robot_name == "robot12L") 
    {
      ROS_INFO_STREAM("\nteng4note, an object created for " << robot_name << " \n");

      //[teng4pkg_msgs/Vector7DOF, Vector7DOF.msg={a1,a2,a3,a4,a5,a6,a7=btn}.]
      sub_omni12_pose6dof_btn = nh.subscribe("/ooomni12/omni12_vector7dof", 1, &Teng4ROS::omni12Pose6dofCallback, this);

    }
    else 
    {
        ROS_FATAL_STREAM("\nInvalid robot_name: " << robot_name);
        throw std::runtime_error("Invalid robot_name");
    }
  }



  ToolState *state;

  void init(ToolState *s)
  { 
    state = s; 

    if (robot_name == "robot11R") 
    {
      ROS_INFO_STREAM("\n(in init) teng4note, init started for: " << robot_name << " \n");

      pub_panda1_joint_states = nh.advertise<sensor_msgs::JointState>("/panda1/joint_states", 1); 
      pub_panda2_joint_states = nh.advertise<sensor_msgs::JointState>("/panda2/joint_states", 1); 

      // cleft11 (panda1) related initialization.
      cleft11_T_EEinit0 = forwardKinematics(q0_initCleft11);
      cleft11_R_EEinit0 = cleft11_T_EEinit0.block<3, 3>(0, 0);
      cleft11_t_EEinit0 = cleft11_T_EEinit0.block<3, 1>(0, 3); 
      ROS_INFO_STREAM("(In init) - cleft11_T_EEinit0 = \n" << cleft11_T_EEinit0 << " ");    
      ROS_INFO_STREAM("(In init) - cleft11_R_EEinit0 = \n" << cleft11_R_EEinit0 << " ");
      ROS_INFO_STREAM("(In init) - cleft11_t_EEinit0 = \n" << cleft11_t_EEinit0 << " ");

      cleft11_T_EEnow = cleft11_T_EEinit0;
      cleft11_R_EEnow = cleft11_R_EEinit0;
      cleft11_t_EEnow = cleft11_t_EEinit0;

      cleft11_T_EEnow_freezed = cleft11_T_EEnow;

      cleft11_T_EEdes = cleft11_T_EEinit0;
      cleft11_R_EEdes = cleft11_T_EEdes.block<3, 3>(0, 0);
      cleft11_t_EEdes = cleft11_T_EEdes.block<3, 1>(0, 3);
      
      for (int i = 0; i < q0_initCleft11.size(); i++)
      {
        cleft11_q_now[i] = q0_initCleft11[i];
        ROS_INFO_STREAM("(In init) - cleft11_q_now[i] = q" << i << " = " << cleft11_q_now[i] << " ");
        cleft11_q_cmd[i] = q0_initCleft11[i];
      }

      for (int i = 0; i < q0_initCleft11.size(); i++)
      {
        q0_seed_cleft11forsave[i] = q0_initCleft11[i];
        qik_forCheck_cleft11forsave[i] = q0_initCleft11[i];
        qik_checked_cleft11forsave[i] = q0_initCleft11[i];        
      }

      // init variables relevant to saving data.
      x_des_cleft11forsave[0] = cleft11_t_EEinit0[0];
      x_des_cleft11forsave[1] = cleft11_t_EEinit0[1];
      x_des_cleft11forsave[2] = cleft11_t_EEinit0[2];
      Eigen::Vector3d cleft11_rpy_EEdes_initforsave = fcn_so3_to_rpy_cpp(cleft11_R_EEdes);
      x_des_cleft11forsave[3] = cleft11_rpy_EEdes_initforsave[0];
      x_des_cleft11forsave[4] = cleft11_rpy_EEdes_initforsave[1];
      x_des_cleft11forsave[5] = cleft11_rpy_EEdes_initforsave[2];
      x_des_old_cleft11forsave = x_des_cleft11forsave;

      if (myfile1.is_open())
      {
        ROS_WARN_STREAM("ATTENTION! Save Data Output file1 opened!");
        ROS_WARN_STREAM("Save Data Output file to: \n" << path1);

        if (flag_once1 == 0)
        {
          myfile1 << "timer1_elapsedAll_ms" << "," 
                << "\n"; 

          flag_once1 = 1;
        }
      }
      else
      {
        ROS_ERROR("ERROR! Unable to open file1 !\n");
      }
      //
      if (myfile2.is_open())
      {
        ROS_WARN_STREAM("ATTENTION! Save Data Output file2 opened!");
        ROS_WARN_STREAM("Save Data Output file to: \n" << path2);

        if (flag_once2 == 0)
        {
          myfile2<< "timer2_elapsedAll_ms" << ","
                << "\n"; 

          flag_once2 = 1;
        }
      }
      else
      {
        ROS_ERROR("ERROR! Unable to open file2 !\n");
      }
    

      // initialization time-related variables.
      teng4_timer0_begin = ros::Time::now().toSec();
      teng4_timer0_now = teng4_timer0_begin;
      teng4_timer0_elapsed = 0.0;
      teng4_timer0_elapsed2 = 0.0;
      teng4_timer1_begin = ros::Time::now().toSec(); 
      teng4_timer1_now = teng4_timer1_begin; 
      teng4_timer1_elapsed = 0.0;
      teng4_timer1_elapsed2 = 0.0;
      teng4_timer2_begin = ros::Time::now().toSec();
      teng4_timer2_now = teng4_timer2_begin; 
      teng4_timer2_elapsed = 0.0;
      teng4_timer2_elapsed2 = 0.0;
      
      ROS_INFO_STREAM("\n(in init) teng4note, init DONE for: " << robot_name << " \n");
    }
    else if (robot_name == "robot12L") 
    {
      ROS_INFO_STREAM("\n(in init) teng4note, init started for: " << robot_name << " \n");

      // cleft12 (panda2) related initialization.
      cleft12_T_EEinit0 = forwardKinematics(q0_initCleft12);
      cleft12_R_EEinit0 = cleft12_T_EEinit0.block<3, 3>(0, 0);
      cleft12_t_EEinit0 = cleft12_T_EEinit0.block<3, 1>(0, 3);
      ROS_INFO_STREAM("(In init) - cleft12_T_EEinit0 = \n" << cleft12_T_EEinit0 << " ");
      ROS_INFO_STREAM("(In init) - cleft12_R_EEinit0 = \n" << cleft12_R_EEinit0 << " ");
      ROS_INFO_STREAM("(In init) - cleft12_t_EEinit0 = \n" << cleft12_t_EEinit0 << " ");

      cleft12_T_EEnow = cleft12_T_EEinit0;
      cleft12_R_EEnow = cleft12_R_EEinit0;
      cleft12_t_EEnow = cleft12_t_EEinit0;

      cleft12_T_EEnow_freezed = cleft12_T_EEnow; 

      cleft12_T_EEdes = cleft12_T_EEinit0;                 
      cleft12_R_EEdes = cleft12_T_EEdes.block<3, 3>(0, 0); 
      cleft12_t_EEdes = cleft12_T_EEdes.block<3, 1>(0, 3); 

      for (int i = 0; i < q0_initCleft12.size(); i++)
      {
        cleft12_q_now[i] = q0_initCleft12[i];
        ROS_INFO_STREAM("(In init) - cleft12_q_now[i] = q" << i << " = " << cleft12_q_now[i] << " ");
        cleft12_q_cmd[i] = q0_initCleft12[i];
      }

      for (int i = 0; i < q0_initCleft12.size(); i++)
      {
        q0_seed_cleft12forsave[i] = q0_initCleft12[i];
        qik_forCheck_cleft12forsave[i] = q0_initCleft12[i];
        qik_checked_cleft12forsave[i] = q0_initCleft12[i];
        
      }

      // init variables relevant to saving data.
      x_des_cleft12forsave[0] = cleft12_t_EEinit0[0];
      x_des_cleft12forsave[1] = cleft12_t_EEinit0[1];
      x_des_cleft12forsave[2] = cleft12_t_EEinit0[2];
      Eigen::Vector3d cleft12_rpy_EEdes_initforsave = fcn_so3_to_rpy_cpp(cleft12_R_EEdes);
      x_des_cleft12forsave[3] = cleft12_rpy_EEdes_initforsave[0];
      x_des_cleft12forsave[4] = cleft12_rpy_EEdes_initforsave[1];
      x_des_cleft12forsave[5] = cleft12_rpy_EEdes_initforsave[2];
      x_des_old_cleft12forsave = x_des_cleft12forsave;
 
      ROS_INFO_STREAM("\n(in init) teng4note, init DONE for: " << robot_name << " \n");
    }
    else 
    {
        ROS_FATAL_STREAM("\n(in init) Invalid robot_name: " << robot_name);
        throw std::runtime_error("Invalid robot_name");
    }
   

  } //end of init().



  //////////////////////////////////////////////////
  // ROS Publish msgs.
  //////////////////////////////////////////////////

  void publish_cleft_msgs() {
    //get ros time
    teng4_timer0_now = ros::Time::now().toSec();
    teng4_timer0_elapsed = teng4_timer0_now - teng4_timer0_begin;

    if (int(teng4_timer0_elapsed * 1000) % (1 * 1000) == 0)
    {
      ROS_INFO_STREAM("----------------------------" << "");  
      ROS_INFO_STREAM("(In pub) - Total timer0 elapsed (s):" << teng4_timer0_elapsed << " ");
    }

    //publish msg for panda1 (for rviz) - JointState msg
    sensor_msgs::JointState panda1_rivz;
    panda1_rivz.header.stamp = ros::Time::now();
    panda1_rivz.name.resize(7);
    panda1_rivz.position.resize(7);
    panda1_rivz.name[0] = "panda1/joint1";
    panda1_rivz.name[1] = "panda1/joint2";
    panda1_rivz.name[2] = "panda1/joint3";    
    panda1_rivz.name[3] = "panda1/joint4";    
    panda1_rivz.name[4] = "panda1/joint5";    
    panda1_rivz.name[5] = "panda1/joint6";
    panda1_rivz.name[6] = "panda1/joint7";
    panda1_rivz.position[0] = cleft11_q_now[0];
    panda1_rivz.position[1] = cleft11_q_now[1];
    panda1_rivz.position[2] = cleft11_q_now[2];
    panda1_rivz.position[3] = cleft11_q_now[3];
    panda1_rivz.position[4] = cleft11_q_now[4];
    panda1_rivz.position[5] = cleft11_q_now[5];
    panda1_rivz.position[6] = cleft11_q_now[6];
    pub_panda1_joint_states.publish(panda1_rivz); 

    //publish msg for panda2 (for rviz) - JointState msg
    sensor_msgs::JointState panda2_rivz;
    panda2_rivz.header.stamp = ros::Time::now();
    panda2_rivz.name.resize(7);
    panda2_rivz.position.resize(7);
    panda2_rivz.name[0] = "panda2/joint1";
    panda2_rivz.name[1] = "panda2/joint2";
    panda2_rivz.name[2] = "panda2/joint3";    
    panda2_rivz.name[3] = "panda2/joint4";    
    panda2_rivz.name[4] = "panda2/joint5";    
    panda2_rivz.name[5] = "panda2/joint6";
    panda2_rivz.name[6] = "panda2/joint7";
    panda2_rivz.position[0] = cleft12_q_now[0];
    panda2_rivz.position[1] = cleft12_q_now[1];
    panda2_rivz.position[2] = cleft12_q_now[2];
    panda2_rivz.position[3] = cleft12_q_now[3];
    panda2_rivz.position[4] = cleft12_q_now[4];
    panda2_rivz.position[5] = cleft12_q_now[5];
    panda2_rivz.position[6] = cleft12_q_now[6];
    pub_panda2_joint_states.publish(panda2_rivz);    

  }


  //////////////////////////////////////////////////
  // ROS Subscribe msg callback.
  //////////////////////////////////////////////////

  // receiving ROS data about omni11 (R)
  //[teng4pkg_msgs/Vector7DOF, Vector7DOF.msg={a1,a2,a3,a4,a5,a6,a7=btn}.]
  void omni11Pose6dofCallback(const teng4pkg_msgs::Vector7DOF &latest_msg) 
  {
    msg_received_counter1++;

    if (1)
    {
      //get ros time
      teng4_timer1_now = ros::Time::now().toSec();
      teng4_timer1_elapsed = teng4_timer1_now - teng4_timer1_begin;

      if (int(teng4_timer1_elapsed * 1000) % (1 * 1000) == 0) 
      {
        ROS_INFO_STREAM("----------------------------" << "");
        ROS_INFO_STREAM("teng4. omni11 (panda1 R) Total timer1 elapsed (s): " << teng4_timer1_elapsed << " ");
      }

      // handle newly received ROS msg data -- msg position part
      Eigen::Vector3d omni11_t_EEnow = {latest_msg.a1, latest_msg.a2, latest_msg.a3};     

      // handle newly received ROS msg data -- msg rotation part
      Eigen::Matrix3d omni11_R_EEnow;
      std::vector<double> tempt02070937 = {latest_msg.a4, latest_msg.a5, latest_msg.a6}; //rpy
      omni11_R_EEnow = fcn_rpy_to_so3_cpp(tempt02070937);

      // handle newly received ROS msg data -- msg omni buttonState part
      omni11button0123 = latest_msg.a7; //buttonState{0,1,2,3}

      // handle newly received ROS msg data -- msg to a form of T matrix 4x4.
      Eigen::Matrix4d omni11_T_EEnow = Eigen::Matrix4d::Identity();
      omni11_T_EEnow.block<3, 3>(0, 0) = omni11_R_EEnow;
      omni11_T_EEnow.block<3, 1>(0, 3) = omni11_t_EEnow;
      // immediately convert omni msg into cleft tool frame.
      omni11_T_EEnow = T_mapRAB1.transpose() * omni11_T_EEnow;
      omni11_R_EEnow = omni11_T_EEnow.block<3, 3>(0, 0);
      omni11_t_EEnow = omni11_T_EEnow.block<3, 1>(0, 3);

      // Capture the very 1st pose as the HOME POSE for omni.
      if (flag_getOmni11PoseInit0 == 1)
      {        
        omni11_T_EEinit0 = omni11_T_EEnow;
        omni11_R_EEinit0 = omni11_T_EEnow.block<3, 3>(0, 0);
        omni11_t_EEinit0 = omni11_T_EEnow.block<3, 1>(0, 3);

        flag_getOmni11PoseInit0 = 0;
      }


      // -----------------------------------------------------------
      // ------ When Omni11 (R) button = 1. buttonState{0,1,2,3}.
      // ------ (3D position mapping only.)
      // ------ When Omni11 (R) button = 2. buttonState{0,1,2,3}.
      // ------ (full 6D pose mapping: 3D position & 3D orientation)
      // -----------------------------------------------------------
      if (omni11button0123 == 1 || omni11button0123 == 2)
      {
        if (flag_toFreezeOmni11) 
        {
          //freeze a set of pose for omni.
          omni11_T_EEnow_freezed = omni11_T_EEnow;
          omni11_R_EEnow_freezed = omni11_T_EEnow_freezed.block<3, 3>(0, 0);
          omni11_t_EEnow_freezed = omni11_T_EEnow_freezed.block<3, 1>(0, 3);
          
          //freeze a set of pose for cleft tool as well.
          cleft11_T_EEnow_freezed = cleft11_T_EEnow;
          cleft11_R_EEnow_freezed = cleft11_T_EEnow_freezed.block<3, 3>(0, 0);
          cleft11_t_EEnow_freezed = cleft11_T_EEnow_freezed.block<3, 1>(0, 3);

          flag_toFreezeOmni11 = 0;
        }

        //scale down omni relative-position
        Eigen::Vector3d cleft11_t_des_delta = (omni11_t_EEnow - omni11_t_EEnow_freezed) * scale_pos_AtoB1;

        // calculate the desired cleft position xyz
        Eigen::Vector3d cleft11_t_EEdes_forCheck = cleft11_t_EEnow_freezed + cleft11_t_des_delta;
        // check xyz limits for cleft tool, if needed.
        //Eigen::Vector3d cleft11_t_EEdes_checked = fcn_check_xyz_limit_for_cleft_tool(cleft11_t_EEdes_forCheck);

        //----------------------------------------------
        //cleft11_t_EEdes = cleft11_t_EEdes_checked;        
        cleft11_t_EEdes = cleft11_t_EEdes_forCheck; //use the one for NO check cleft xyz limit
        //----------------------------------------------

        // calculate the desired cleft orientation matrix R
        Eigen::Matrix3d cleft11_R_EEdes_prepare1 = (omni11_R_EEnow*omni11_R_EEinit0.inverse()); 
        Eigen::Matrix3d cleft11_R_EEdes_prepare2 =  cleft11_R_EEdes_prepare1 * cleft11_R_EEinit0;

        if (omni11button0123 == 1) {
          cleft11_R_EEdes = cleft11_R_EEnow; //control only 3d position.
        }else if (omni11button0123 == 2) {
          cleft11_R_EEdes = cleft11_R_EEdes_prepare2; //control full 6d pose.
        }

        // Now we can compose the final desired 6d pose for cleft tool's IK solver. 
        cleft11_T_EEdes.block<3, 3>(0, 0) = cleft11_R_EEdes;
        cleft11_T_EEdes.block<3, 1>(0, 3) = cleft11_t_EEdes;

        //for params to save data.
        q0_seed_cleft11forsave = cleft11_q_now;
        x_des_cleft11forsave[0] = cleft11_t_EEdes[0];
        x_des_cleft11forsave[1] = cleft11_t_EEdes[1];
        x_des_cleft11forsave[2] = cleft11_t_EEdes[2];
        Eigen::Vector3d cleft11_rpy_EEdes_forsave = fcn_so3_to_rpy_cpp(cleft11_R_EEdes);
        x_des_cleft11forsave[3] = cleft11_rpy_EEdes_forsave[0];
        x_des_cleft11forsave[4] = cleft11_rpy_EEdes_forsave[1];
        x_des_cleft11forsave[5] = cleft11_rpy_EEdes_forsave[2];

        teng4_timer1_now = ros::Time::now().toSec();

        // calculate IK solution via IK solver.        
        std::vector<double> q_ik_forCheck = solveIK(cleft11_T_EEdes, cleft11_q_now);

        teng4_timer1_elapsed2 = ros::Time::now().toSec() - teng4_timer1_now;
        ROS_INFO_STREAM("teng4 cleft11 (panda1 R) IK solver time cost is (ms): " << teng4_timer1_elapsed2 * 1000.0 << "\n");
        
        //--------------------------------------
        // checking IK solution for joint limits.
        std::vector<double> q_ik_checked;
        q_ik_checked = fcn_check_joint_limits_for_cleft_tool_11R(q_ik_forCheck); //comment out this line if you do not want to check joint limit.
        q_ik_checked = q_ik_forCheck; //use this one to overwrite if want "NO joint limits checking".
        //--------------------------------------

        qik_forCheck_cleft11forsave = q_ik_forCheck; 
        qik_checked_cleft11forsave = q_ik_checked;   

        cleft11_q_now = q_ik_checked;
        //update the cleft tool current pose T_EEnow  
        cleft11_T_EEnow = forwardKinematics(cleft11_q_now);
        cleft11_R_EEnow = cleft11_T_EEnow.block<3, 3>(0, 0);
        cleft11_t_EEnow = cleft11_T_EEnow.block<3, 1>(0, 3);

        //prepare q cmd for other visualization platforms, e.g., Unity.
        for (int i = 0; i < q0_initCleft11.size(); i++)
        {
          cleft11_q_cmd[i] = cleft11_q_now[i];
        }
        
      }      
      // -----------------------------------------------------------
      // ------ When Omni11 (R) button = 0 or 3. buttonState{0,1,2,3}.
      // ------ (none pose mapping)
      // -----------------------------------------------------------
      else
      {
        flag_toFreezeOmni11 = 1;    
      }

      
      // save data
      static unsigned long prevNanos11 = ros::Time::now().toNSec(); // get nanosec from rostime (1sec=1e-9nsec=1e-6microsec=1e-3millisec)
      const unsigned long interval11 = 1000000 * 10;                // in nanosec, 10ms, 10000microsec=10ms=10000000nanosec
      if (ros::Time::now().toNSec() - prevNanos11 >= interval11)
      { // save data every interval11=10ms
        myfile1<< teng4_timer1_elapsed * 1000.0 << ","               
               << "\n";         
        prevNanos11 += interval11;
      }
      teng4_timer1_elapsed2 = 0; 
    }
  }



  // receiving ROS data about omni12 (L)
  //[teng4pkg_msgs/Vector7DOF, Vector7DOF.msg={a1,a2,a3,a4,a5,a6,a7=btn}.]
  void omni12Pose6dofCallback(const teng4pkg_msgs::Vector7DOF& latest_msg) 
  {
    msg_received_counter2++;

    if (1)
    {
      //get ros time
      teng4_timer2_now = ros::Time::now().toSec();
      teng4_timer2_elapsed = teng4_timer2_now - teng4_timer2_begin;

      if (int(teng4_timer2_elapsed * 1000) % (1 * 1000) == 0)
      {
        ROS_INFO_STREAM("----------------------------" << "");
        ROS_INFO_STREAM("teng4. omni12 (panda2 L) Total timer2 elapsed (s): " << teng4_timer2_elapsed << " ");
      }

      // handle newly received ROS msg data -- msg position part
      Eigen::Vector3d omni12_t_EEnow = {latest_msg.a1, latest_msg.a2, latest_msg.a3};     

      // handle newly received ROS msg data -- msg rotation part
      Eigen::Matrix3d omni12_R_EEnow;
      std::vector<double> tempt02070937 = {latest_msg.a4, latest_msg.a5, latest_msg.a6};
      omni12_R_EEnow = fcn_rpy_to_so3_cpp(tempt02070937);

      // handle newly received ROS msg data -- msg omni buttonState part
      omni12button0123 = latest_msg.a7; //buttonState{0,1,2,3}

      // handle newly received ROS msg data -- msg to a form of T matrix 4x4.
      Eigen::Matrix4d omni12_T_EEnow = Eigen::Matrix4d::Identity();
      omni12_T_EEnow.block<3, 3>(0, 0) = omni12_R_EEnow;
      omni12_T_EEnow.block<3, 1>(0, 3) = omni12_t_EEnow;
      // immediately convert omni msg into cleft tool frame.      
      omni12_T_EEnow = T_mapRAB2.transpose() * omni12_T_EEnow;
      omni12_R_EEnow = omni12_T_EEnow.block<3, 3>(0, 0);
      omni12_t_EEnow = omni12_T_EEnow.block<3, 1>(0, 3);

      // Capture the very 1st pose as the HOME POSE for omni.
      if (flag_getOmni12PoseInit0 == 1)
      {        
        omni12_T_EEinit0 = omni12_T_EEnow;
        omni12_R_EEinit0 = omni12_T_EEnow.block<3, 3>(0, 0);
        omni12_t_EEinit0 = omni12_T_EEnow.block<3, 1>(0, 3);

        flag_getOmni12PoseInit0 = 0;
      }

      
      // -----------------------------------------------------------
      // ------ When Omni12 (L) button = 1. buttonState{0,1,2,3}.
      // ------ (3D position mapping only.)
      // ------ When Omni12 (L) button = 2. buttonState{0,1,2,3}.
      // ------ (full 6D pose mapping: 3D position & 3D orientation)
      // -----------------------------------------------------------
      if (omni12button0123 == 1 || omni12button0123 == 2)
      {
        if (flag_toFreezeOmni12) 
        {
          //freeze a set of pose for omni.
          omni12_T_EEnow_freezed = omni12_T_EEnow;
          omni12_R_EEnow_freezed = omni12_T_EEnow_freezed.block<3, 3>(0, 0);
          omni12_t_EEnow_freezed = omni12_T_EEnow_freezed.block<3, 1>(0, 3);
          
          //freeze a set of pose for cleft tool as well.
          cleft12_T_EEnow_freezed = cleft12_T_EEnow;
          cleft12_R_EEnow_freezed = cleft12_T_EEnow_freezed.block<3, 3>(0, 0);
          cleft12_t_EEnow_freezed = cleft12_T_EEnow_freezed.block<3, 1>(0, 3);

          flag_toFreezeOmni12 = 0;
        }

        //scale down omni relative-position
        Eigen::Vector3d cleft12_t_des_delta = (omni12_t_EEnow - omni12_t_EEnow_freezed) * scale_pos_AtoB2;

        // calculate the desired cleft position xyz
        Eigen::Vector3d cleft12_t_EEdes_forCheck = cleft12_t_EEnow_freezed + cleft12_t_des_delta;
        // check xyz limits for cleft tool if needed.
        //Eigen::Vector3d cleft12_t_EEdes_checked = fcn_check_xyz_limit_for_cleft_tool(cleft12_t_EEdes_forCheck);

        //----------------------------------------------
        //cleft12_t_EEdes = cleft12_t_EEdes_checked;        
        cleft12_t_EEdes = cleft12_t_EEdes_forCheck;  //use the one for NO check cleft xyz limit
        //----------------------------------------------

        // calculate the desired cleft orientation matrix R
        Eigen::Matrix3d cleft12_R_EEdes_prepare1 = (omni12_R_EEnow*omni12_R_EEinit0.inverse());
        Eigen::Matrix3d cleft12_R_EEdes_prepare2 =  cleft12_R_EEdes_prepare1 * cleft12_R_EEinit0;

        if (omni12button0123 == 1) {
          cleft12_R_EEdes = cleft12_R_EEnow; //control only 3d position.
        }else if (omni12button0123 == 2) {
          cleft12_R_EEdes = cleft12_R_EEdes_prepare2; //control full 6d pose.
        }

        // Now we can compose the final desired 6d pose for cleft tool's IK solver. 
        cleft12_T_EEdes.block<3, 3>(0, 0) = cleft12_R_EEdes;
        cleft12_T_EEdes.block<3, 1>(0, 3) = cleft12_t_EEdes;

        //for params to save data.
        q0_seed_cleft12forsave = cleft12_q_now; 
        x_des_cleft12forsave[0] = cleft12_t_EEdes[0];
        x_des_cleft12forsave[1] = cleft12_t_EEdes[1];
        x_des_cleft12forsave[2] = cleft12_t_EEdes[2];
        Eigen::Vector3d cleft12_rpy_EEdes_forsave = fcn_so3_to_rpy_cpp(cleft12_R_EEdes);
        x_des_cleft12forsave[3] = cleft12_rpy_EEdes_forsave[0];
        x_des_cleft12forsave[4] = cleft12_rpy_EEdes_forsave[1];
        x_des_cleft12forsave[5] = cleft12_rpy_EEdes_forsave[2];

        teng4_timer2_now = ros::Time::now().toSec(); //unit,sec

        // calculate IK solution via IK solver.        
        std::vector<double> q_ik_forCheck = solveIK(cleft12_T_EEdes, cleft12_q_now);

        teng4_timer2_elapsed2 = ros::Time::now().toSec() - teng4_timer2_now;
        ROS_INFO_STREAM("teng4 cleft12 (panda2 L) IK solver time cost is (ms): " << teng4_timer2_elapsed2 * 1000.0 << "\n");

        //--------------------------------------
        // checking IK solution for joint limits.
        std::vector<double> q_ik_checked;
        q_ik_checked = fcn_check_joint_limits_for_cleft_tool_12L(q_ik_forCheck); //comment out this line if you do not want to check joint limit.
        q_ik_checked = q_ik_forCheck; //use this one to overwrite if want "NO joint limits checking".
        //--------------------------------------

        qik_forCheck_cleft12forsave = q_ik_forCheck; 
        qik_checked_cleft12forsave = q_ik_checked;   

        cleft12_q_now = q_ik_checked;
        //update the cleft tool current pose T_EEnow  
        cleft12_T_EEnow = forwardKinematics(cleft12_q_now);
        cleft12_R_EEnow = cleft12_T_EEnow.block<3, 3>(0, 0);
        cleft12_t_EEnow = cleft12_T_EEnow.block<3, 1>(0, 3);
    
        //prepare q cmd for other visualization platforms, e.g., Unity.
        for (int i = 0; i < q0_initCleft12.size(); i++)
        {
          cleft12_q_cmd[i] = cleft12_q_now[i];
        }
        
      }      
      // -----------------------------------------------------------
      // ------ When Omni12 (L) button = 0 or 3. buttonState{0,1,2,3}.
      // ------ (none pose mapping)
      // -----------------------------------------------------------
      else
      {    
        flag_toFreezeOmni12 = 1;    
      }


      // save data
      static unsigned long prevNanos12 = ros::Time::now().toNSec(); // get nanosec from rostime (1sec=1e-9nsec=1e-6microsec=1e-3millisec)
      const unsigned long interval12 = 1000000 * 10;                // in nanosec, 10ms, 10000microsec=10ms=10000000nanosec
      if (ros::Time::now().toNSec() - prevNanos12 >= interval12)
      { // save data every interval11=10ms
        myfile2<< teng4_timer2_elapsed * 1000.0 << ","               
               << "\n"; 
        prevNanos12 += interval12;
      }
      teng4_timer2_elapsed2 = 0; 
    }
  }




  // ///////////////////////////////////////////////////////////////
  // //////////// teng4 customized fcn() starting from here below.
  // ///////////////////////////////////////////////////////////////

  // fcn (so3 -> rpy) cpp
  Eigen::Vector3d fcn_so3_to_rpy_cpp (const Eigen::Matrix3d &R)
  { 
    double R11, R12, R13;
    double R21, R22, R23;
    double R31, R32, R33;

    R11 = R(0,0);
    R12 = R(0,1);
    R13 = R(0,2);
    R21 = R(1,0);
    R22 = R(1,1);
    R23 = R(1,2);
    R31 = R(2,0);
    R32 = R(2,1);
    R33 = R(2,2);

    double sy = sqrt(R32 * R32 + R33 * R33);
    double eps = 2.2204e-16;
    double arad, brad, grad;

    if (sy > eps)
    {
      //% non singular
      arad = atan2(R32, R33);
      brad = atan2(-R31, sy);
      grad = atan2(R21, R11);
    }
    else
    {
      //% singular
      arad = atan2(-R23, R22);
      brad = atan2(-R31, sy);
      grad = 0;
    }

    Eigen::Vector3d rpy;
    rpy[0] = arad;
    rpy[1] = brad;
    rpy[2] = grad;

    return rpy;
  }

  
  // fcn (rpy -> so3) cpp
  Eigen::Matrix3d fcn_rpy_to_so3_cpp (const std::vector<double> &rpy)
  {
    double arad = rpy[0];
    double brad = rpy[1];
    double grad = rpy[2];

    Eigen::Matrix3d Rz;
    Rz << cos(grad), -sin(grad), 0,
         sin(grad), cos(grad), 0,
         0, 0, 1;

    Eigen::Matrix3d Ry;
    Ry << cos(brad), 0, sin(brad),
       0, 1, 0,
       -sin(brad), 0, cos(brad);

    Eigen::Matrix3d Rx;
    Rx << 1, 0, 0,
       0, cos(arad), -sin(arad),
       0, sin(arad), cos(arad);

    Eigen::Matrix3d R = Rz * Ry * Rx;

    return R; 
  }


  // fcn pose error(T0, T1)
  Eigen::VectorXd get_T_error(Eigen::Matrix4d &current_pose, Eigen::Matrix4d target_pose) 
  {
    Eigen::Matrix4d d_T = current_pose.inverse() * target_pose;

    Eigen::Vector3d position_error = d_T.block<3, 1>(0, 3);

    Eigen::Matrix3d rotation_error_matrix = d_T.block<3, 3>(0, 0) - Eigen::Matrix3d::Identity();

    Eigen::Vector3d rotation_error = {rotation_error_matrix(2, 1) - rotation_error_matrix(1, 2), rotation_error_matrix(0, 2) - rotation_error_matrix(2, 0), rotation_error_matrix(1, 0) - rotation_error_matrix(0, 1)};
    rotation_error = 0.5 * rotation_error;

    Eigen::VectorXd error(6);
    error << position_error, rotation_error;

    return error.transpose();
  }


  // fcn T=FK(q), Compute forward kinematics
  Eigen::Matrix4d forwardKinematics(const std::vector<double> &joint_angles)
  {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (size_t i = 0; i < dh_params.size(); ++i)
    {
      double theta = dh_params[i].is_prismatic ? dh_params[i].theta : joint_angles[i] + dh_params[i].theta;      
      double d = dh_params[i].is_prismatic ? joint_angles[i]+dh_params[i].d : dh_params[i].d;
      T *= dhTransform(theta, dh_params[i].alpha, dh_params[i].a, d);
    }
    return T;
  }

  
  // fcn J (note, body Jacobian)
  Eigen::MatrixXd getJacobianHJmethod(const std::vector<double> &joint_angles) 
  {
    size_t n = joint_angles.size(); 
    Eigen::MatrixXd J(6, n);
    std::vector<Eigen::Matrix4d> T_ia_ib(n, Eigen::Matrix4d::Identity());

    // Compute all transformations
    Eigen::Vector3d zero3d = Eigen::Vector3d(0, 0, 0);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_0_EE = Eigen::Matrix4d::Identity();
    for (size_t i = 0; i < n; ++i)
    {
      double theta = dh_params[i].is_prismatic ? dh_params[i].theta : joint_angles[i] + dh_params[i].theta;
      double d = dh_params[i].is_prismatic ? joint_angles[i]+dh_params[i].d : dh_params[i].d; 
      T = dhTransform(theta, dh_params[i].alpha, dh_params[i].a, d);
      T_0_EE *= T;
      T_ia_ib[i] = T;      
    }

    for (size_t i = 0; i < n; ++i)
    {
      Eigen::Matrix4d T_i_EE = Eigen::Matrix4d::Identity();

      if (i < n - 1) // for link-1,2,3,...,n-1.
      {
        for (size_t j = i + 1; j < n; ++j)
        {
          T_i_EE *= T_ia_ib[j];
        }
      }
      else // for last link-n.
      {
        T_i_EE = Eigen::Matrix4d::Identity();
      }

      Eigen::Vector3d J_ix_all = Eigen::Vector3d(0, 0, 0);
      Eigen::Vector3d J_iy_all = Eigen::Vector3d(0, 0, 0);
      Eigen::Vector3d J_iz_all = Eigen::Vector3d(0, 0, 0);

      J_ix_all = T_i_EE.block<3, 1>(0, 3).cross(T_i_EE.block<3, 1>(0, 0));
      J(0, i) = dh_params[i].is_prismatic ? T_i_EE(2, 0) : J_ix_all(2);
      
      J_iy_all = T_i_EE.block<3, 1>(0, 3).cross(T_i_EE.block<3, 1>(0, 1));
      J(1, i) = dh_params[i].is_prismatic ? T_i_EE(2, 1) : J_iy_all(2);
      
      J_iz_all = T_i_EE.block<3, 1>(0, 3).cross(T_i_EE.block<3, 1>(0, 2));
      J(2, i) = dh_params[i].is_prismatic ? T_i_EE(2, 2) : J_iz_all(2);
      
      J(3, i) = dh_params[i].is_prismatic ? 0 : T_i_EE(2, 0);
      J(4, i) = dh_params[i].is_prismatic ? 0 : T_i_EE(2, 1);
      J(5, i) = dh_params[i].is_prismatic ? 0 : T_i_EE(2, 2);
    }

    return J;
  }


  // IKsolver, Solve inverse kinematics
  std::vector<double> solveIK(const Eigen::Matrix4d &T1,
                              std::vector<double> &q0,
                              double tolerance = 1e-10,
                              int max_iterations = 500,
                              bool swapped = false) {
    std::vector<double> qk = q0;
    std::vector<double> qk_new = q0; 
    int rej_count = 0;
    int max_rej = 100;

    double dmp_fact = 0.1;
    Eigen::VectorXd err_new(6);
    Eigen::VectorXd err_now(6);
    for (int iter = 0; iter < max_iterations; ++iter)
    {   
      Eigen::Matrix4d T0 = forwardKinematics(qk);
      // Compute pose error
      err_now << get_T_error(T0, T1); //T0 is current pose, T1 is target pose.

      if (err_now.norm() < tolerance)
      { 
        // wrap angles for revolute joints
        for (int i = 0; i < qk.size(); i++) 
        {
            if (qk[i] > M_PI)
            {
              qk[i] = qk[i] - 2 * M_PI;
            }
            if (qk[i] < -M_PI)
            {
              qk[i] = qk[i] + 2 * M_PI;
            }
        }

        return qk;
      }



      // Compute Jacobian
      Eigen::MatrixXd J = getJacobianHJmethod(qk);

      Eigen::MatrixXd Inn = Eigen::MatrixXd::Identity(qk.size(), qk.size());

      //do the damped inverse Gauss-Newton with Levenberg-Marquadt
      Eigen::VectorXd delta_qk = (J.transpose() * J + dmp_fact * Inn).inverse() * J.transpose() * err_now;

      for (size_t i = 0; i < qk.size(); ++i)
      {
        qk_new[i] = qk[i] + delta_qk[i];
      }

      Eigen::Matrix4d T0_new = forwardKinematics(qk_new);

      err_new << get_T_error(T0_new, T1);

      if (err_new.norm() < err_now.norm()) //good update
      { 
        qk = qk_new;
        err_now = err_new;
        dmp_fact = dmp_fact * 0.5; 
        rej_count = 0;        
      }
      else //bad update
      { 
        dmp_fact = dmp_fact * 2.0;
        rej_count = rej_count + 1;

        if (rej_count > max_rej)
        { 
          break;
        }
      }
    }

    return qk;
  }

  
  // fcn check cleft xyz limits
  Eigen::Vector3d fcn_check_xyz_limit_for_cleft_tool(const Eigen::Vector3d &xyz)
  {
    //check xyz limits for cleft tool.
    double x = xyz[0];
    double y = xyz[1];
    double z = xyz[2];

    // check cleft tool x limit.
    if (x < x_min) {
      x = x_min;
      ROS_WARN_STREAM("warning, cleft tool x_min is reached.");
    }
    else if (x > x_max) {
      x = x_max;
      ROS_WARN_STREAM("warning, cleft tool x_max is reached.");
    }
    // check cleft tool y limit.
    if (y < y_min) {
      y = y_min;
      ROS_WARN_STREAM("warning, cleft tool y_min is reached.");
    }
    else if (y > y_max) {
      y = y_max;
      ROS_WARN_STREAM("warning, cleft tool y_max is reached.");
    }
    // check cleft tool z limit.
    if (z < z_min) {
      z = z_min;
      ROS_WARN_STREAM("warning, cleft tool z_min is reached.");
    }
    else if (z > z_max) {
      z = z_max;
      ROS_WARN_STREAM("warning, cleft tool z_max is reached.");
    }

    Eigen::Vector3d xyz_new = {x,y,z};
    return xyz_new;
  }


  // fcn check joint limits of "Right" hand side tool, if needed.
  std::vector<double> fcn_check_joint_limits_for_cleft_tool_11R(const std::vector<double> &q)
  {
    // Define joint limits (assuming these are globally available)
    std::vector<double> q_min = {Rq1_min, Rq2_min, Rq3_min, Rq4_min, Rq5_min, Rq6_min, Rq7_min};
    std::vector<double> q_max = {Rq1_max, Rq2_max, Rq3_max, Rq4_max, Rq5_max, Rq6_max, Rq7_max};

    std::vector<double> q_new = q;

    for (size_t i = 0; i < q.size(); ++i)
    {
      if (q[i] < q_min[i])
      {
        q_new[i] = q_min[i];
        ROS_WARN_STREAM("Warning: sim 11R tool Joint (" << i + 1 << ") reached min limit.");
      }
      else if (q[i] > q_max[i])
      {
        q_new[i] = q_max[i];
        ROS_WARN_STREAM("Warning: sim 11R tool Joint (" << i + 1 << ") reached max limit.");
      }
    }

    return q_new;   

  }


  // fcn check joint limits of "Left" hand side tool, if needed.
  std::vector<double> fcn_check_joint_limits_for_cleft_tool_12L(const std::vector<double> &q)  
  {
    // Define joint limits (assuming these are globally available)
    std::vector<double> q_min = {q1_min, q2_min, q3_min, q4_min, q5_min, q6_min, q7_min};
    std::vector<double> q_max = {q1_max, q2_max, q3_max, q4_max, q5_max, q6_max, q7_max};

    std::vector<double> q_new = q;

    for (size_t i = 0; i < q.size(); ++i) {
        if (q[i] < q_min[i]) {
            q_new[i] = q_min[i];
            ROS_WARN_STREAM("Warning: sim 12L tool Joint (" << i+1 << ") reached min limit.");
        } 
        else if (q[i] > q_max[i]) {
            q_new[i] = q_max[i];
            ROS_WARN_STREAM("Warning: sim 12L tool Joint (" << i+1 << ") reached max limit.");
        }
    }

    return q_new;    
  }
};



//ROS publish configuration and init
void *ros_publish(void *ptr) {
  Teng4ROS *teng_ros = (Teng4ROS *) ptr;
  int publish_rate2;
  ros::param::param(std::string("~publish_rate"), publish_rate2, 1000);
  ros::Rate loop_rate(1000); //publish to ROS at 1000hz
  ros::AsyncSpinner spinner(2);
  spinner.start();
  while (ros::ok()) {
    teng_ros->publish_cleft_msgs();
    loop_rate.sleep();
  };
  return NULL;
}



// main fcn
int main(int argc, char** argv) 
{
  ros::init(argc, argv, "real2sim_kernel_node");

  ToolState state;

  // R - 7dof panda DH table.
  std::vector<DHParameter> paramsR = {// Franka Research 3 kinematic chain.
      {0, 0, 0.333, 0, false},             // Joint 1
      {0, -M_PI_2, 0, 0, false},           // Joint 2
      {0, M_PI_2, 0.316, 0, false},        // Joint 3
      {0.0825, M_PI_2, 0, 0, false},       // Joint 4
      {-0.0825, -M_PI_2, 0.384, 0, false}, // Joint 5
      {0, M_PI_2, 0, 0, false},            // Joint 6
      {0.088, M_PI_2, 0, 0, false}         // Joint 7
      //{ 0, 0, 0.107, 0, false} // flange, EE
  };

  // L - 7dof panda DH table.
  std::vector<DHParameter> paramsL = {// Franka Research 3 kinematic chain.
      {0, 0, 0.333, 0, false},             // Joint 1
      {0, -M_PI_2, 0, 0, false},           // Joint 2
      {0, M_PI_2, 0.316, 0, false},        // Joint 3
      {0.0825, M_PI_2, 0, 0, false},       // Joint 4
      {-0.0825, -M_PI_2, 0.384, 0, false}, // Joint 5
      {0, M_PI_2, 0, 0, false},            // Joint 6
      {0.088, M_PI_2, 0, 0, false}         // Joint 7
      //{ 0, 0, 0.107, 0, false} // flange, EE
  };


  //initialize robots based on ID name.
  Teng4ROS cleft11R_solver_node(paramsR, q0_initCleft11, "robot11R");
  Teng4ROS cleft12L_solver_node(paramsL, q0_initCleft12, "robot12L");
  cleft11R_solver_node.init(&state);
  cleft12L_solver_node.init(&state);

  // Loop and publish
  pthread_t publish_thread;
  pthread_create(&publish_thread, NULL, ros_publish, (void*) &cleft11R_solver_node);
  pthread_join(publish_thread, NULL);

  ROS_INFO("Ending Session....");
  return 0;
}
