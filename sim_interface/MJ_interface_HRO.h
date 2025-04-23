#pragma once

#include <mujoco/mujoco.h>
#include "data_bus.h"
#include <string>
#include <vector>

class MJ_interface_HRO
{
public:
    int jointNum{0};
    std::vector<double> motor_pos;
    std::vector<double> motor_pos_Old;
    std::vector<double> motor_vel;
    double rpy[3]{0};        // roll,pitch and yaw of baselink
    double baseQuat[4]{0};   // in quat, mujoco order is [w,x,y,z], here we rearrange to [x,y,z,w]
    double f3d[3][2]{0};     // 3D foot-end contact force, L for 1st col, R for 2nd col
    double basePos[3]{0};    // position of baselink, in world frame
    double baseAcc[3]{0};    // acceleration of baselink, in body frame
    double baseAngVel[3]{0}; // angular velocity of baselink, in body frame
    double baseLinVel[3]{0}; // linear velocity of baselink, in body frame

    // For HRO-c2-v1
    const std::vector<std::string> JointName = {"torso_pan_joint", "torso_tilt_joint",
                                                "right_arm_joint_1", "right_arm_joint_2", "right_arm_joint_3", "right_arm_joint_4", "right_arm_joint_5", "right_arm_joint_6", "right_arm_joint_7",
                                                "left_arm_joint_1", "left_arm_joint_2", "left_arm_joint_3", "left_arm_joint_4", "left_arm_joint_5", "left_arm_joint_6", "left_arm_joint_7",
                                                "right_leg_joint_1", "right_leg_joint_2", "right_leg_joint_3", "right_leg_joint_4", "right_leg_joint_5", "right_leg_joint_6", "right_leg_joint_7",
                                                "left_leg_joint_1", "left_leg_joint_2", "left_leg_joint_3", "left_leg_joint_4", "left_leg_joint_5", "left_leg_joint_6", "left_leg_joint_7"};

    const std::string baseName = "torso";

    const std::string orientationSensorName = "baselink-quat"; // in quat, mujoco order is [w,x,y,z], here we rearrange to [x,y,z,w]
    const std::string velSensorName = "baselink-velocity";
    const std::string gyroSensorName = "baselink-gyro";
    const std::string accSensorName = "baselink-baseAcc";

    MJ_interface_HRO(mjModel *mj_modelIn, mjData *mj_dataIn);
    void updateSensorValues();
    void setMotorsTorque(std::vector<double> &tauIn);
    void dataBusWrite(DataBus &busIn);

private:
    mjModel *mj_model;
    mjData *mj_data;
    std::vector<int> jntId_qpos, jntId_qvel, jntId_dctl;

    int orientataionSensorId;
    int velSensorId;
    int gyroSensorId;
    int accSensorId;
    int baseBodyId;

    double timeStep{0.001}; // second
    bool isIni{false};
};
