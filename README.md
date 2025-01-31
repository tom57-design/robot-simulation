<img src="./assets/logo.png" style="zoom:30%;" />

# OpenLoong Dynamics Control

## Motion Control Framework for Humanoid Robots Based on MPC and WBC

Welcome to the 🐉 OpenLoong open-source project repository!

The OpenLoong open-source project is jointly operated by Humanoid Robot (Shanghai) Co., Ltd., the Shanghai Humanoid Robot Manufacturing Innovation Center, and the OpenAtom Foundation. This repository provides a humanoid robot control framework based on Model Predictive Control (MPC) and Whole-Body Control (WBC), which can be deployed on the Mujoco simulation platform. Built on the "Qinglong" humanoid robot model from the Shanghai Humanoid Robot Innovation Center, the project offers three motion examples: walking, jumping, and blind stepping over obstacles. On the physical prototype, the robot successfully demonstrates walking and blind stepping over obstacles.


## Project Highlights

- **Easy Deployment** Provides a comprehensive solution for setting up the code execution environment, allowing users to configure their required workspace with ease. The repository includes essential dependencies, eliminating the need for extensive third-party library installations and simplifying the entire deployment process.


- **Scalability** The control framework follows a layered and modular design, enhancing system maintainability and scalability. Each functional module has clearly defined logical and functional boundaries, creating a developer-friendly environment for secondary development, making it easier to customize and extend system functionalities.


- **Ease of Understanding** The code structure is clean and well-organized, following a modular design principle that encapsulates functions efficiently. A data bus is used for inter-module communication, reducing redundancy and lowering overall complexity. The algorithm implementations follow a "Read-Compute-Write" logic, improving code readability and comprehension.


  <center><img src="./assets/行走.gif" alt="行走" style="zoom:50%;" /><img src="./assets/踩障碍.gif" alt="踩障碍" style="zoom:50%;" /></center>



## Environment Setup

**Recommended Environment**

- Operating System：Ubuntu 22.04.4 LTS
- Compiler：g++ 11.4.0



**Dependency Installation**

This repository is designed for simulation testing of the "Qinglong" humanoid robot using Mujoco. The Mujoco physics engine, Pinocchio dynamics library, Eigen, Quill logging tool, GLFW graphics library, and JsonCpp parsing library are all included in the repository. However, the simulation interface requires OpenGL support, so you need to install the following dependencies:

```Bash
# Update & Install Dependencies
sudo apt-get update
sudo apt install git cmake gcc-11 g++-11
sudo apt install libglu1-mesa-dev freeglut3-dev
```

## User Guide

**Code Retrieval & Compilation**

```Bash
# Clone
git clone https://atomgit.com/openloong/openloong-dyn-control.git

# Build
cd openloong-dyn-control
mkdir build
cd build
cmake ..
make

# mujoco simulation
./walk_mpc_wbc #or ./walk_wbc or ./jump_mpc
```

**Simulation Result**

<img src="./assets/demo.png" alt="demo" style="zoom:50%;" />

## **Code Explanation**

Refer to the API of this code[Documentation](https://www.openloong.org.cn/pages/api/html/index.html) and [Wiki](https://www.openloong.org.cn/pages/wiki/html/index.html)

**Main Prefix and Suffix Meanings**

| Prefix/Suffix    | Meaning                    |
| ---------------- | -------------------------- |
| *_L, _W*         | In body frame, world frame |
| *fe_*            | Foot end                   |
| *_L, _l, _R, _r* | Left side, Right side      |
| *swing,* *sw*    | Swing Leg                  |
| *stance,* *st*   | Stance Leg                 |
| *eul, rpy*       | Euler Angles(orientation)  |
| *omega*          | Angular velocity           |
| *pos*            | Position                   |
| *vel*            | Linear velocity            |
| *tor**, tau*     | Torque                     |
| *base*           | *BaseLink*                 |
| *_des*           | Desired value              |
| *_cur*           | Current actual value       |
| *_rot*           | Transformation matrix      |

## Development Guide

**Key Control Parameter Descriptions**

- MPC Weights

```C++
//MPC.h
void    set_weight(double u_weight, Eigen::MatrixXd L_diag, Eigen::MatrixXd K_diag);
//*u_weight* ：Minimum weight for system inputs
//*L_diag* ：Weights for system state and desired error, in the order of eul, pos, omega, vel
//*K_diag* ：Weights for system inputs, in the order of fl, tl, fr, tr  
```

- WBC Priority

```C++
//WBC_QP.cpp
std::vector<std::string taskOrder;
taskOrder.emplace_back("RedundantJoints");
taskOrder.emplace_back("static_Contact");
taskOrder.emplace_back("Roll_Pitch_Yaw_Pz");
taskOrder.emplace_back("PxPy");
taskOrder.emplace_back("SwingLeg");
taskOrder.emplace_back("HandTrack");
//Add priority & adjust the order
```

- WBC Weights

```C++
//PriorityTasks.h
Eigen::MatrixXd Kp; // Position error weights in one of the WBC priorities
Eigen::MatrixXd Kd; // Velocity error weights in one of the WBC priorities
//WBC_QP.h
Eigen::MatrixXd Q1; // External contact force & desired error weights, in the order of fl, tl, fr, tr
Eigen::MatrixXd Q2; //Joint acceleration & desired error weights
```

- Swing Leg Trajectories

```C++
//FootPlacement.h
double kp_vx;      // Swing leg x position foot placement gain 
double kp_vy;      // Swing leg y position foot placement gain 
double kp_wz;      // Swing leg z position foot placement gain 
double stepHeight; // Foot lift height
//FootPlacement.cpp
double    FootPlacement::Trajectory(double phase, double des1, double des2);        // Swing leg z direction trajectory
//phase：swing phase of reaching the highest point 
//des1：highest position of trajectory 
//des2：final position of trajectory
```

- Gait Control

```C++
//GaitScheduler.h
double tSwing;      // Single gait period time 
double FzThrehold;  // Threshold value of foot contact force 
//GaitScheduler.cpp
DataBus::LegState legState=DataBus::RS; // Initial swing leg
```

- Joint Parameters

```json
//JointCtrConfig.json
   "Joint-ankle-l-pitch" : {
      "PVT_LPF_Fc" : 20,
      "kd" : 5.0,
      "kp" : 50.0,
      "maxPos" : 0.61087,
      "maxSpeed" : 48.8,
      "maxTorque" : 58.5,
      "minPos" : -0.43644
   }
```

**Model Replacement Instructions**

For model replacement, please refer to [Tutorial](https://atomgit.com/openloong/openloong-dyn-control/blob/master/Tutorial.md) documentation。

## Reference

[1] D. Kim, J. D. Carlo, B. Katz, G. Bledt, S. Kim, Highly dynamic quadruped locomotion via whole-body impulse control and model predictive control. arXiv:1909.06586 (2019).

[2] Kim D, Jorgensen S J, Lee J, et al. Dynamic locomotion for passive-ankle biped robots and humanoids using whole-body locomotion control. arXiv:1901.08100 (2020).

[3] Di Carlo J, Wensing P M, Katz B, et al. Dynamic locomotion in the mit  cheetah 3 through convex model-predictive control[C]//2018 IEEE/RSJ  international conference on intelligent robots and systems (IROS). IEEE, 2018: 1-9.

[4] 卞泽坤, 王兴兴. 四足机器人控制算法: 建模、控制与实践[M]. 机械工业出版社, 2023
