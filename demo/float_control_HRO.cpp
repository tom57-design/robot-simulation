#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <cstdio>
#include <iostream>
#include "useful_math.h"
#include "GLFW_callbacks.h"
#include "MJ_interface_HRO.h"
#include "PVT_ctrl_HRO.h"
#include "pino_kin_dyn_HRO.h"
#include "data_logger.h"

// MuJoCo load and compile model
char error[1000] = "Could not load binary model";
mjModel *mj_model = mj_loadXML("../models/HRO-c2-v1/mjcf/scene.xml", 0, error, 1000);
mjData *mj_data = mj_makeData(mj_model);

//************************
// main function
int main(int argc, const char **argv)
{
    // ini classes
    UIctr uiController(mj_model, mj_data);                                              // UI control for Mujoco
    MJ_interface_HRO mj_interface(mj_model, mj_data);                                   // data interface for Mujoco
    Pin_KinDyn_HRO kinDynSolver("../models/HRO-c2-v1/urdf/HRO-c2-v1.urdf");             // kinematics and dynamics solver
    DataBus RobotState(kinDynSolver.model_nv);                                          // data bus
    PVT_Ctr_HRO pvtCtr(mj_model->opt.timestep, "../common/joint_ctrl_config_HRO.json"); // PVT joint control
    DataLogger logger("../record/datalog.log");                                         // data logger

    // variables ini
    double stand_legLength = 1.1; //-0.95; // desired baselink height
    double foot_height = 0.07;    // distance between the foot ankel joint and the bottom
    double xv_des = 0.7;          // desired velocity in x direction

    RobotState.width_hips = 0.229;
    // mju_copy(mj_data->qpos, mj_model->key_qpos, mj_model->nq*1); // set ini pos in Mujoco
    int model_nv = kinDynSolver.model_nv;

    // ini position and posture for foot-end and hand
    std::vector<double> motors_pos_des(model_nv - 6, 0);
    std::vector<double> motors_pos_cur(model_nv - 6, 0);
    std::vector<double> motors_vel_des(model_nv - 6, 0);
    std::vector<double> motors_vel_cur(model_nv - 6, 0);
    std::vector<double> motors_tau_des(model_nv - 6, 0);
    std::vector<double> motors_tau_cur(model_nv - 6, 0);

    Eigen::Vector3d fe_l_pos_L_des = {-0.0, 0.15, -stand_legLength};
    Eigen::Vector3d fe_r_pos_L_des = {-0.0, -0.15, -stand_legLength};
    Eigen::Vector3d fe_l_eul_L_des = {M_PI, -0.00, -0.000};
    Eigen::Vector3d fe_r_eul_L_des = {M_PI, -0.00, 0.000};

    Eigen::Matrix3d fe_l_rot_des = eul2Rot(fe_l_eul_L_des(0), fe_l_eul_L_des(1), fe_l_eul_L_des(2));
    Eigen::Matrix3d fe_r_rot_des = eul2Rot(fe_r_eul_L_des(0), fe_r_eul_L_des(1), fe_r_eul_L_des(2));

    Eigen::Vector3d hd_l_pos_L_des = {-0.0, 0.4, -0.3};
    Eigen::Vector3d hd_r_pos_L_des = {-0.0, -0.4, -0.3};
    Eigen::Vector3d hd_l_eul_L_des = {M_PI, 0, M_PI / 2};
    Eigen::Vector3d hd_r_eul_L_des = {M_PI, 0, M_PI / 2};

    Eigen::Matrix3d hd_l_rot_des = eul2Rot(hd_l_eul_L_des(0), hd_l_eul_L_des(1), hd_l_eul_L_des(2));
    Eigen::Matrix3d hd_r_rot_des = eul2Rot(hd_r_eul_L_des(0), hd_r_eul_L_des(1), hd_r_eul_L_des(2));

    auto resLeg = kinDynSolver.computeInK_Leg(fe_l_rot_des, fe_l_pos_L_des, fe_r_rot_des, fe_r_pos_L_des);
    auto resHand = kinDynSolver.computeInK_Hand(hd_l_rot_des, hd_l_pos_L_des, hd_r_rot_des, hd_r_pos_L_des);

    // register variable name for data logger
    logger.addIterm("simTime", 1);
    logger.addIterm("motors_pos_cur", model_nv - 6);
    logger.addIterm("motors_pos_des", model_nv - 6);
    logger.addIterm("motors_tau_cur", model_nv - 6);
    logger.addIterm("motors_vel_des", model_nv - 6);
    logger.addIterm("motors_vel_cur", model_nv - 6);
    logger.finishItermAdding();

    /// ----------------- sim Loop ---------------
    double simEndTime = 20;
    mjtNum simstart = mj_data->time;
    double simTime = mj_data->time;
    double startSteppingTime = 3;
    double startWalkingTime = 5;

    // init UI: GLFW
    uiController.iniGLFW();
    uiController.enableTracking(); // enable viewpoint tracking of the body 1 of the robot
    uiController.createWindow("Demo", false);

    while (!glfwWindowShouldClose(uiController.window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        simstart = mj_data->time;
        while (mj_data->time - simstart < 1.0 / 60.0)
        {
            mj_step(mj_model, mj_data);

            simTime = mj_data->time;
            // printf("-------------%.3f s------------\n", simTime);
            mj_interface.updateSensorValues();
            mj_interface.dataBusWrite(RobotState);

            fe_l_pos_L_des = {-0.0, 0.15, -stand_legLength + 0.15};
            fe_r_pos_L_des = {-0.0, -0.15, -stand_legLength + 0.15};
            fe_l_eul_L_des = {M_PI, -0.00, -0.000};
            fe_r_eul_L_des = {M_PI, -0.00, 0.000};

            fe_l_rot_des = eul2Rot(fe_l_eul_L_des(0), fe_l_eul_L_des(1), fe_l_eul_L_des(2));
            fe_r_rot_des = eul2Rot(fe_r_eul_L_des(0), fe_r_eul_L_des(1), fe_r_eul_L_des(2));

            hd_l_pos_L_des = {-0.0, 0.4, -0.3};
            hd_r_pos_L_des = {-0.0, -0.4, -0.3};
            hd_l_eul_L_des = {M_PI, 0, M_PI / 2};
            hd_r_eul_L_des = {M_PI, 0, M_PI / 2};

            hd_l_rot_des = eul2Rot(hd_l_eul_L_des(0), hd_l_eul_L_des(1), hd_l_eul_L_des(2));
            hd_r_rot_des = eul2Rot(hd_r_eul_L_des(0), hd_r_eul_L_des(1), hd_r_eul_L_des(2));

            resLeg = kinDynSolver.computeInK_Leg(fe_l_rot_des, fe_l_pos_L_des, fe_r_rot_des, fe_r_pos_L_des);
            resHand = kinDynSolver.computeInK_Hand(hd_l_rot_des, hd_l_pos_L_des, hd_r_rot_des, hd_r_pos_L_des);

            // Enter here functions to send actuator commands, like:
            // arm-l: 0-6, arm-r: 7-13, head: 14,15 waist: 16-18, leg-l: 19-24, leg-r: 25-30
            // get the final joint command
            RobotState.motors_pos_des = eigen2std(resLeg.jointPosRes + resHand.jointPosRes);
            RobotState.motors_vel_des = motors_vel_des;
            RobotState.motors_tor_des = motors_tau_des;
            //            Eigen::VectorXd tmp=resLeg.jointPosRes+resHand.jointPosRes;
            //            std::cout<<tmp.transpose()<<std::endl;
            //            std::cout<<resHand.itr<<std::endl;
            //            std::cout<<resHand.err.transpose()<<std::endl;

            pvtCtr.dataBusRead(RobotState);
            if (simTime <= 3)
            {
                pvtCtr.calMotorsPVT(100.0 / 1000.0 / 180.0 * 3.1415); // limit velocity
            }
            else
            {
                pvtCtr.setJointPD(500, 100, "right_arm_joint_1");
                pvtCtr.setJointPD(500, 100, "right_arm_joint_2");
                pvtCtr.setJointPD(500, 100, "right_arm_joint_3");
                pvtCtr.setJointPD(500, 100, "right_arm_joint_4");
                pvtCtr.setJointPD(500, 100, "right_arm_joint_5");
                pvtCtr.setJointPD(500, 100, "right_arm_joint_6");
                pvtCtr.setJointPD(500, 100, "right_arm_joint_7");

                pvtCtr.setJointPD(500, 100, "left_arm_joint_1");
                pvtCtr.setJointPD(500, 100, "left_arm_joint_2");
                pvtCtr.setJointPD(500, 100, "left_arm_joint_3");
                pvtCtr.setJointPD(500, 100, "left_arm_joint_4");
                pvtCtr.setJointPD(500, 100, "left_arm_joint_5");
                pvtCtr.setJointPD(500, 100, "left_arm_joint_6");
                pvtCtr.setJointPD(500, 100, "left_arm_joint_7");

                pvtCtr.setJointPD(500, 100, "right_leg_joint_1");
                pvtCtr.setJointPD(500, 100, "right_leg_joint_2");
                pvtCtr.setJointPD(500, 100, "right_leg_joint_3");
                pvtCtr.setJointPD(500, 100, "right_leg_joint_4");
                pvtCtr.setJointPD(500, 100, "right_leg_joint_5");
                pvtCtr.setJointPD(500, 100, "right_leg_joint_6");
                pvtCtr.setJointPD(500, 100, "right_leg_joint_7");

                pvtCtr.setJointPD(500, 100, "left_leg_joint_1");
                pvtCtr.setJointPD(500, 100, "left_leg_joint_2");
                pvtCtr.setJointPD(500, 100, "left_leg_joint_3");
                pvtCtr.setJointPD(500, 100, "left_leg_joint_4");
                pvtCtr.setJointPD(500, 100, "left_leg_joint_5");
                pvtCtr.setJointPD(500, 100, "left_leg_joint_6");
                pvtCtr.setJointPD(500, 100, "left_leg_joint_7");

                pvtCtr.calMotorsPVT();
            }
            pvtCtr.dataBusWrite(RobotState);

            mj_interface.setMotorsTorque(RobotState.motors_tor_out);

            logger.startNewLine();
            logger.recItermData("simTime", simTime);
            logger.recItermData("motors_pos_cur", RobotState.motors_pos_cur);
            logger.recItermData("motors_pos_des", RobotState.motors_pos_des);
            logger.recItermData("motors_tau_cur", RobotState.motors_tor_out);
            logger.recItermData("motors_vel_cur", RobotState.motors_vel_cur);
            logger.recItermData("motors_vel_des", RobotState.motors_vel_des);
            logger.finishLine();
        }

        if (mj_data->time >= simEndTime)
        {
            break;
        }

        uiController.updateScene();
    }

    //    // free visualization storage
    uiController.Close();

    // free MuJoCo model and data, deactivate
    mj_deleteData(mj_data);
    mj_deleteModel(mj_model);

    return 0;
}