#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <cstdio>
#include <iostream>
#include "useful_math.h"
#include "GLFW_callbacks.h"
#include "MJ_interface_G1.h"
#include "PVT_ctrl_G1.h"
#include "pino_kin_dyn_G1.h"
#include "data_logger.h"
#include "wbc_priority_G1.h"
#include "gait_scheduler.h"
#include "foot_placement.h"
#include "joystick_interpreter.h"

// MuJoCo load and compile model
char error[1000] = "Could not load binary model";
mjModel *mj_model = mj_loadXML("../models/unitree_g1/g1_23dof_feet_fixed.xml", 0, error, 1000);
mjData *mj_data = mj_makeData(mj_model);

//************************
// main function
int main(int argc, const char **argv)
{
    // ini classes
    UIctr uiController(mj_model, mj_data);                                                    // UI control for Mujoco
    MJ_interface_G1 mj_interface(mj_model, mj_data);                                          // data interface for Mujoco
    Pin_KinDyn_G1 kinDynSolver("../models/unitree_g1/g1_23dof.urdf");                         // kinematics and dynamics solver
    DataBus RobotState(kinDynSolver.model_nv);                                                // data bus
    WBC_priority_G1 WBC_solv(kinDynSolver.model_nv, 6 + 12, 22, 0.7, mj_model->opt.timestep); // WBC solver
    GaitScheduler gaitScheduler(0.4, mj_model->opt.timestep);                                 // gait scheduler
    PVT_Ctr_G1 pvtCtr(mj_model->opt.timestep, "../common/joint_ctrl_config_G1.json");         // PVT joint control
    FootPlacement footPlacement;                                                              // foot-placement planner
    JoyStickInterpreter jsInterp(mj_model->opt.timestep);                                     // desired baselink velocity generator
    DataLogger logger("../record/datalog.log");                                               // data logger

    // variables ini
    double stand_legLength = 0.69; // desired baselink height
    double foot_height = 0.07;     // distance between the foot ankel joint and the bottom

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

    Eigen::Vector3d fe_l_pos_L_des = {0.04, 0.18, -0.72};     // Tuned
    Eigen::Vector3d fe_r_pos_L_des = {0.04, -0.18, -0.72};    // Tuned
    Eigen::Vector3d fe_l_eul_L_des = {-0.000, -0.08, -0.000}; // Tuned
    Eigen::Vector3d fe_r_eul_L_des = {0.000, -0.08, 0.000};   // Tuned

    Eigen::Matrix3d fe_l_rot_des = eul2Rot(fe_l_eul_L_des(0), fe_l_eul_L_des(1), fe_l_eul_L_des(2));
    Eigen::Matrix3d fe_r_rot_des = eul2Rot(fe_r_eul_L_des(0), fe_r_eul_L_des(1), fe_r_eul_L_des(2));

    Eigen::Vector3d hd_l_pos_L_des = {-0.0, 0.2, 0.03};  // Tuned
    Eigen::Vector3d hd_r_pos_L_des = {-0.0, -0.2, 0.03}; // Tuned
    Eigen::Vector3d hd_l_eul_L_des = {-0, 0, 0};         // Tuned
    Eigen::Vector3d hd_r_eul_L_des = {0, 0, 0};          // Tuned

    Eigen::Matrix3d hd_l_rot_des = eul2Rot(hd_l_eul_L_des(0), hd_l_eul_L_des(1), hd_l_eul_L_des(2));
    Eigen::Matrix3d hd_r_rot_des = eul2Rot(hd_r_eul_L_des(0), hd_r_eul_L_des(1), hd_r_eul_L_des(2));

    auto resLeg = kinDynSolver.computeInK_Leg(fe_l_rot_des, fe_l_pos_L_des, fe_r_rot_des, fe_r_pos_L_des);
    auto resHand = kinDynSolver.computeInK_Hand(hd_l_rot_des, hd_l_pos_L_des, hd_r_rot_des, hd_r_pos_L_des);

    Eigen::VectorXd qIniDes = Eigen::VectorXd::Zero(mj_model->nq, 1);
    qIniDes.block(7, 0, mj_model->nq - 7, 1) = resLeg.jointPosRes + resHand.jointPosRes;
    WBC_solv.setQini(qIniDes, RobotState.q);

    // register variable name for data logger
    logger.addIterm("simTime", 1);
    logger.addIterm("motors_pos_cur", model_nv - 6);
    logger.addIterm("motors_vel_cur", model_nv - 6);
    logger.addIterm("motors_tor_cur", model_nv - 6);
    logger.addIterm("rpy", 3);
    logger.addIterm("fL", 3);
    logger.addIterm("fR", 3);
    logger.addIterm("basePos", 3);
    logger.addIterm("baseLinVel", 3);
    logger.addIterm("baseAcc", 3);
    logger.addIterm("baseAngVel", 3);
    logger.finishItermAdding();

    // Write the headers to the first row of the log file
    logger.writeHeaders();

    // Define logging interval (e.g., 0.01s for 100Hz logging)
    double log_interval = 0.01;
    double last_log_time = 0.0;

    /// ----------------- sim Loop ---------------
    double simEndTime = 30;
    mjtNum simstart = mj_data->time;
    double simTime = mj_data->time;
    double startSquatingTime_1 = 0.5;
    double startStandingTime_1 = 3.5;
    double startSquatingTime_2 = 5.5;
    double startStandingTime_2 = 8.0;

    // init UI: GLFW
    uiController.iniGLFW();
    uiController.enableTracking(); // enable viewpoint tracking of the body 1 of the robot
    uiController.createWindow("Demo", false);

    while (!glfwWindowShouldClose(uiController.window))
    {
        simstart = mj_data->time;
        while (mj_data->time - simstart < 1.0 / 60.0 && uiController.runSim) // press "1" to pause and resume, "2" to step the simulation
        {

            mj_step(mj_model, mj_data);

            simTime = mj_data->time;
            printf("-------------%.3f s------------\n", simTime);

            mj_interface.updateSensorValues();

            mj_interface.dataBusWrite(RobotState);

            // update kinematics and dynamics info
            kinDynSolver.dataBusRead(RobotState);
            kinDynSolver.computeJ_dJ();
            kinDynSolver.computeDyn();
            kinDynSolver.dataBusWrite(RobotState);

            if (simTime > startSquatingTime_1 && simTime <= startStandingTime_1)
            {
                const double dt = 0.001;

                fe_l_pos_L_des(2) = Ramp(fe_l_pos_L_des(2), -stand_legLength * 0.75, 0.1 * dt); // 0.5
                fe_r_pos_L_des(2) = Ramp(fe_r_pos_L_des(2), -stand_legLength * 0.75, 0.1 * dt);

                fe_l_eul_L_des = {-0.0, -0.0, -0.0};
                fe_r_eul_L_des = {0.0, -0.0, 0.0};

                fe_l_rot_des = eul2Rot(fe_l_eul_L_des(0), fe_l_eul_L_des(1), fe_l_eul_L_des(2));
                fe_r_rot_des = eul2Rot(fe_r_eul_L_des(0), fe_r_eul_L_des(1), fe_r_eul_L_des(2));

                auto resLeg = kinDynSolver.computeInK_Leg(fe_l_rot_des, fe_l_pos_L_des, fe_r_rot_des, fe_r_pos_L_des);
                auto resHand = kinDynSolver.computeInK_Hand(hd_l_rot_des, hd_l_pos_L_des, hd_r_rot_des, hd_r_pos_L_des);

                RobotState.base_pos_stand = RobotState.base_pos;

                RobotState.motors_pos_des = eigen2std(resLeg.jointPosRes + resHand.jointPosRes);
                RobotState.motors_vel_des.assign(model_nv - 6, 0);
                RobotState.motors_tor_des.assign(model_nv - 6, 0);

                RobotState.motionState = DataBus::Stand;
            }
            else if (simTime > startStandingTime_1 && simTime <= startSquatingTime_2)
            {
                const double dt = 0.001;

                fe_l_pos_L_des(2) = Ramp(fe_l_pos_L_des(2), -stand_legLength, 0.2 * dt); // 0.5
                fe_r_pos_L_des(2) = Ramp(fe_r_pos_L_des(2), -stand_legLength, 0.2 * dt);

                fe_l_eul_L_des = {-0.0, -0.0, -0.0};
                fe_r_eul_L_des = {0.0, -0.0, 0.0};

                fe_l_rot_des = eul2Rot(fe_l_eul_L_des(0), fe_l_eul_L_des(1), fe_l_eul_L_des(2));
                fe_r_rot_des = eul2Rot(fe_r_eul_L_des(0), fe_r_eul_L_des(1), fe_r_eul_L_des(2));

                auto resLeg = kinDynSolver.computeInK_Leg(fe_l_rot_des, fe_l_pos_L_des, fe_r_rot_des, fe_r_pos_L_des);
                auto resHand = kinDynSolver.computeInK_Hand(hd_l_rot_des, hd_l_pos_L_des, hd_r_rot_des, hd_r_pos_L_des);

                RobotState.base_pos_stand = RobotState.base_pos;

                RobotState.motors_pos_des = eigen2std(resLeg.jointPosRes + resHand.jointPosRes);
                RobotState.motors_vel_des.assign(model_nv - 6, 0);
                RobotState.motors_tor_des.assign(model_nv - 6, 0);

                RobotState.motionState = DataBus::Stand;
            }
            else if (simTime > startSquatingTime_2 && simTime <= startStandingTime_2)
            {
                const double dt = 0.001;

                RobotState.base_pos_stand = RobotState.base_pos;
                RobotState.base_pos_des(2) = Ramp(RobotState.base_pos(2), stand_legLength * 0.8, 5 * dt);

                RobotState.motors_vel_des.assign(model_nv - 6, 0);
                RobotState.motors_tor_des.assign(model_nv - 6, 0);

                RobotState.motionState = DataBus::Stand;
            }
            else if (simTime > startStandingTime_2)
            {
                const double dt = 0.001;

                RobotState.base_pos_stand = RobotState.base_pos;
                RobotState.base_pos_des(2) = Ramp(RobotState.base_pos(2), stand_legLength, 50 * dt);

                RobotState.motors_vel_des.assign(model_nv - 6, 0);
                RobotState.motors_tor_des.assign(model_nv - 6, 0);

                RobotState.motionState = DataBus::Stand;
            }
            else
                jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), RobotState.base_rpy(2));

            // ------------- WBC ------------
            // WBC input
            RobotState.Fr_ff = Eigen::VectorXd::Zero(12);
            RobotState.des_ddq = Eigen::VectorXd::Zero(mj_model->nv);
            RobotState.des_dq = Eigen::VectorXd::Zero(mj_model->nv);
            RobotState.des_delta_q = Eigen::VectorXd::Zero(mj_model->nv);

            // WBC Calculation
            WBC_solv.dataBusRead(RobotState);

            WBC_solv.computeDdq(kinDynSolver);

            WBC_solv.computeTau();

            WBC_solv.dataBusWrite(RobotState);

            pvtCtr.dataBusRead(RobotState);
            if (simTime <= startSquatingTime_1)
            {
                pvtCtr.calMotorsPVT(100.0 / 1000.0 / 180.0 * 3.1415);
            }
            else
            {
                //**** Comment out this section to make the robot stand still & debug for foot placement ****//
                Eigen::VectorXd pos_des = kinDynSolver.integrateDIY(RobotState.q, RobotState.wbc_delta_q_final);
                RobotState.motors_pos_des = eigen2std(pos_des.block(7, 0, model_nv - 6, 1));
                RobotState.motors_vel_des = eigen2std(RobotState.wbc_dq_final);
                RobotState.motors_tor_des = eigen2std(RobotState.wbc_tauJointRes);

                pvtCtr.calMotorsPVT(0.1 / 180.0 * 3.1415);
                // pvtCtr.calMotorsPVT();
            }
            pvtCtr.dataBusWrite(RobotState);

            mj_interface.setMotorsTorque(RobotState.motors_tor_out);

            // Logging condition: log only if log_interval has passed
            if (simTime - last_log_time >= log_interval)
            {
                logger.startNewLine();
                logger.recItermData("simTime", simTime);
                logger.recItermData("motors_pos_cur", RobotState.motors_pos_cur);
                logger.recItermData("motors_vel_cur", RobotState.motors_vel_cur);
                logger.recItermData("motors_tor_cur", RobotState.motors_tor_cur);
                logger.recItermData("rpy", RobotState.rpy);
                logger.recItermData("fL", RobotState.fL);
                logger.recItermData("fR", RobotState.fR);
                logger.recItermData("basePos", RobotState.basePos);
                logger.recItermData("baseLinVel", RobotState.baseLinVel);
                logger.recItermData("baseAcc", RobotState.baseAcc);
                logger.recItermData("baseAngVel", RobotState.baseAngVel);
                logger.finishLine();

                last_log_time = simTime; // Update last log time
            }

            // printf("rpyVal=[%.5f, %.5f, %.5f]\n", RobotState.rpy[0], RobotState.rpy[1], RobotState.rpy[2]);
            // printf("gps=[%.5f, %.5f, %.5f]\n", RobotState.basePos[0], RobotState.basePos[1], RobotState.basePos[2]);
            // printf("vel=[%.5f, %.5f, %.5f]\n", RobotState.baseLinVel[0], RobotState.baseLinVel[1], RobotState.baseLinVel[2]);
        }

        if (mj_data->time >= simEndTime)
        {
            break;
        }

        uiController.updateScene();
    }

    //    // free visualization storage
    uiController.Close();

    return 0;
}