#pragma once
#include <fstream>
#include "json/json.h"
#include <string>
#include "LPF_fst.h"
#include <vector>
#include <cmath>
#include "data_bus.h"

class PVT_Ctr_HRO
{
public:
    int jointNum;
    std::vector<double> motor_pos_cur;
    std::vector<double> motor_pos_des_old;
    std::vector<double> motor_vel;
    std::vector<double> motor_tor_out; // final tau output
    PVT_Ctr_HRO(double timeStepIn, const char *jsonPath);
    void calMotorsPVT();
    void calMotorsPVT(double deltaP_Lim);
    void enablePV();          // enable PV control item
    void disablePV();         // disable PV control item
    void enablePV(int jtId);  // enable PV control item
    void disablePV(int jtId); // disable PV control item
    void setJointPD(double kp, double kd, const char *jointName);
    void dataBusRead(DataBus &busIn);
    void dataBusWrite(DataBus &busIn);

    std::vector<double> motor_pos_des; // P des
    std::vector<double> motor_vel_des; // V des
    std::vector<double> motor_tor_des; // T des

    std::vector<double> pvt_Kp;
    std::vector<double> pvt_Kd;
    std::vector<double> maxTor;
    std::vector<double> maxVel;
    std::vector<double> maxPos;
    std::vector<double> minPos;

private:
    std::vector<LPF_Fst> tau_out_lpf;
    std::vector<int> PV_enable;
    double sign(double in);

    // For HRO-c2-v1
    const std::vector<std::string> motorName = {
        "left_leg_joint_1",
        "left_leg_joint_2",
        "left_leg_joint_3",
        "left_leg_joint_4",
        "left_leg_joint_5",
        "left_leg_joint_6",
        "left_leg_joint_7",
        "right_leg_joint_1",
        "right_leg_joint_2",
        "right_leg_joint_3",
        "right_leg_joint_4",
        "right_leg_joint_5",
        "right_leg_joint_6",
        "right_leg_joint_7",
        "torso_pan_joint",
        "torso_tilt_joint",
        "left_arm_joint_1",
        "left_arm_joint_2",
        "left_arm_joint_3",
        "left_arm_joint_4",
        "left_arm_joint_5",
        "left_arm_joint_6",
        "left_arm_joint_7",
        "right_arm_joint_1",
        "right_arm_joint_2",
        "right_arm_joint_3",
        "right_arm_joint_4",
        "right_arm_joint_5",
        "right_arm_joint_6",
        "right_arm_joint_7"};
};
