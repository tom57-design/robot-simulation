#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include "GLFW_callbacks.h"
#include "MJ_interface_G1.h"
#include "PVT_ctrl_G1.h"
#include "data_logger.h"
#include "data_bus.h"
#include "pino_kin_dyn_G1.h"
#include "useful_math.h"
#include "wbc_priority_G1.h"
#include "mpc.h"
#include "gait_scheduler_G1.h"
#include "foot_placement_G1.h"
#include "joystick_interpreter.h"
#include <string>
#include <iostream>

const double dt = 0.001;
const double dt_200Hz = 0.005;
// MuJoCo load and compile model
char error[1000] = "Could not load binary model";
mjModel *mj_model = mj_loadXML("../models/unitree_g1/g1_23dof.xml", 0, error, 1000);
mjData *mj_data = mj_makeData(mj_model);

int main(int argc, char **argv)
{
    // initialize classes
    UIctr uiController(mj_model, mj_data);                                                // UI control for Mujoco
    MJ_interface_G1 mj_interface(mj_model, mj_data);                                      // data interface for Mujoco
    Pin_KinDyn_G1 kinDynSolver("../models/unitree_g1/g1_23dof.urdf");                     // kinematics and dynamics solver
    DataBus RobotState(kinDynSolver.model_nv);                                            // data bus
    WBC_priority_G1 WBC_solv(kinDynSolver.model_nv, 18, 22, 0.7, mj_model->opt.timestep); // WBC solver
    MPC MPC_solv(dt_200Hz);                                                               // mpc controller
    GaitScheduler_G1 gaitScheduler(0.25, mj_model->opt.timestep);                         // gait scheduler
    PVT_Ctr_G1 pvtCtr(mj_model->opt.timestep, "../common/joint_ctrl_config_G1.json");     // PVT joint control
    FootPlacement_G1 footPlacement;                                                       // foot-placement planner
    JoyStickInterpreter jsInterp(mj_model->opt.timestep);                                 // desired baselink velocity generator
    DataLogger logger("../record/datalog.log");                                           // data logger

    // initialize UI: GLFW
    uiController.iniGLFW();
    uiController.enableTracking();            // enable viewpoint tracking of the body 1 of the robot
    uiController.createWindow("Demo", false); // NOTE: if the saveVideo is set to true, the raw recorded file could be 2.5 GB for 15 seconds!

    // initialize variables
    double stand_legLength = 0.72; // 0.97;// desired baselink height
    double foot_height = 0.07;     // distance between the foot ankel joint and the bottom
    double xv_des = 1.2 * 0.4;     // desired velocity in x direction
    int model_nv = kinDynSolver.model_nv;

    RobotState.width_hips = 0.229;
    footPlacement.kp_vx = 0.03 * 5;
    footPlacement.kp_vy = 0.03;
    footPlacement.kp_wz = 0.03;
    footPlacement.stepHeight = 0.1;
    footPlacement.legLength = stand_legLength;

    gaitScheduler.FzThrehold = 100;

    // mju_copy(mj_data->qpos, mj_model->key_qpos, mj_model->nq * 1); // set ini pos in Mujoco

    std::vector<double> motors_pos_des(model_nv - 6, 0);
    std::vector<double> motors_pos_cur(model_nv - 6, 0);
    std::vector<double> motors_vel_des(model_nv - 6, 0);
    std::vector<double> motors_vel_cur(model_nv - 6, 0);
    std::vector<double> motors_tau_des(model_nv - 6, 0);
    std::vector<double> motors_tau_cur(model_nv - 6, 0);

    // ini position and posture for foot-end and hand
    Eigen::Vector3d fe_l_pos_L_des = {0.04, 0.18, -stand_legLength};  // Tuned
    Eigen::Vector3d fe_r_pos_L_des = {0.04, -0.18, -stand_legLength}; // Tuned
    Eigen::Vector3d fe_l_eul_L_des = {-0.000, -0.15, -0.000};         // Tuned
    Eigen::Vector3d fe_r_eul_L_des = {0.000, -0.15, 0.000};           // Tuned

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

    //// -------------------------- main loop --------------------------------

    int MPC_count = 0; // count for controlling the mpc running period

    double startSteppingTime = 1.2;
    double startWalkingTime = 1.5;
    double simEndTime = 30;

    mjtNum simstart = mj_data->time;
    double simTime = mj_data->time;

    while (!glfwWindowShouldClose(uiController.window))
    {
        simstart = mj_data->time;
        while (mj_data->time - simstart < 1.0 / 60.0 && uiController.runSim)
        { // press "1" to pause and resume, "2" to step the simulation
            mj_step(mj_model, mj_data);
            simTime = mj_data->time;
            // Read the sensors:
            mj_interface.updateSensorValues();
            mj_interface.dataBusWrite(RobotState);

            // update kinematics and dynamics info
            kinDynSolver.dataBusRead(RobotState);
            kinDynSolver.computeJ_dJ();
            kinDynSolver.computeDyn();
            kinDynSolver.dataBusWrite(RobotState);

            if (simTime > startWalkingTime)
            {
                jsInterp.setWzDesLPara(0, 1);
                jsInterp.setVxDesLPara(xv_des, 2.0);    // jsInterp.setVxDesLPara(0.9,1);
                RobotState.motionState = DataBus::Walk; // start walking
            }
            else
                jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), RobotState.base_rpy(2));
            jsInterp.step();
            RobotState.js_pos_des(2) = stand_legLength + foot_height - 0.1; // pos z is not assigned in jyInterp
            RobotState.base_pos_des(2) = stand_legLength + foot_height - 0.1;
            jsInterp.dataBusWrite(RobotState); // only pos x, pos y, theta z, vel x, vel y , omega z are rewrote.

            if (simTime >= startSteppingTime)
            {
                // gait scheduler
                gaitScheduler.dataBusRead(RobotState);
                gaitScheduler.step();
                gaitScheduler.dataBusWrite(RobotState);

                footPlacement.dataBusRead(RobotState);
                footPlacement.getSwingPos();
                footPlacement.dataBusWrite(RobotState);
            }

            // ------------- MPC ------------
            MPC_count = MPC_count + 1;
            if (MPC_count > (dt_200Hz / dt - 1))
            {
                MPC_solv.dataBusRead(RobotState);
                MPC_solv.cal();
                MPC_solv.dataBusWrite(RobotState);
                MPC_count = 0;
            }

            // ------------- WBC ------------
            // WBC Calculation
            WBC_solv.dataBusRead(RobotState);
            WBC_solv.computeDdq(kinDynSolver);
            WBC_solv.computeTau();
            WBC_solv.dataBusWrite(RobotState);
            // get the final joint command
            if (simTime <= startSteppingTime)
            {
                RobotState.motors_pos_des = eigen2std(resLeg.jointPosRes + resHand.jointPosRes);
                RobotState.motors_vel_des = motors_vel_des;
                RobotState.motors_tor_des = motors_tau_des;
            }
            else
            {
                MPC_solv.enable();
                Eigen::Matrix<double, 1, nx> L_diag;
                Eigen::Matrix<double, 1, nu> K_diag;
                L_diag << 1.0, 1.0, 1.0, // eul
                    1.0, 2.0, 1.0,       // pCoM
                    1e-7, 1e-7, 1e-7,    // w
                    100.0, 10.0, 1.0;    // vCoM
                K_diag << 1.0, 1.0, 1.0, // fl
                    1.0, 1.0, 1.0,
                    1.0, 1.0, 1.0, // fr
                    1.0, 1.0, 1.0, 1.0;
                MPC_solv.set_weight(1e-6, L_diag, K_diag);

                Eigen::VectorXd pos_des = kinDynSolver.integrateDIY(RobotState.q, RobotState.wbc_delta_q_final);
                RobotState.motors_pos_des = eigen2std(pos_des.block(7, 0, model_nv - 6, 1));
                RobotState.motors_vel_des = eigen2std(RobotState.wbc_dq_final);
                RobotState.motors_tor_des = eigen2std(RobotState.wbc_tauJointRes);
            }

            // joint PVT controller
            pvtCtr.dataBusRead(RobotState);
            if (simTime <= startWalkingTime)
            {
                pvtCtr.calMotorsPVT(2 * 100.0 / 1000.0 / 180.0 * 3.1415);
            }
            else
            {
                // //**** For G1-23DOF ****//
                // // Also comment out this section if we want to make it stand still
                // pvtCtr.setJointPD(100, 5, "left_ankle_pitch_joint");
                // pvtCtr.setJointPD(40, 10, "left_ankle_roll_joint");
                // pvtCtr.setJointPD(100, 5, "right_ankle_pitch_joint");
                // pvtCtr.setJointPD(40, 10, "right_ankle_roll_joint");
                // pvtCtr.setJointPD(1600, 100, "left_knee_joint");
                // pvtCtr.setJointPD(1600, 100, "right_knee_joint");

                pvtCtr.calMotorsPVT(5 * 100.0 / 1000.0 / 180.0 * 3.1415);
                // pvtCtr.calMotorsPVT();
            }
            pvtCtr.dataBusWrite(RobotState);

            // give the joint torque command to Webots
            mj_interface.setMotorsTorque(RobotState.motors_tor_out);

            // print info to the console
            //            printf("f_L=[%.3f, %.3f, %.3f]\n", RobotState.fL[0], RobotState.fL[1], RobotState.fL[2]);
            //            printf("f_R=[%.3f, %.3f, %.3f]\n", RobotState.fR[0], RobotState.fR[1], RobotState.fR[2]);
            //
            //            printf("rpyVal=[%.5f, %.5f, %.5f]\n", RobotState.rpy[0], RobotState.rpy[1], RobotState.rpy[2]);
            //            printf("basePos=[%.5f, %.5f, %.5f]\n", RobotState.basePos[0], RobotState.basePos[1], RobotState.basePos[2]);

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

            //**** VISUALIZING PLANNED FOOT STEPS ****//
            Eigen::Vector3d swingPos = RobotState.swing_fe_pos_des_W; // Current swing foot position

            // Persistent storage for last known swing foot position
            static Eigen::Vector3d lastSwingPos_left(0, 0, 0);
            static Eigen::Vector3d lastSwingPos_right(0, 0, 0);

            int left_site_id = mj_name2id(mj_model, mjOBJ_SITE, "left_step_plan");
            int right_site_id = mj_name2id(mj_model, mjOBJ_SITE, "right_step_plan");

            int base_des_site_id = mj_name2id(mj_model, mjOBJ_SITE, "base_pos_des");

            // Update the site for the swing foot
            if (RobotState.legState == DataBus::LSt && right_site_id >= 0)
            { // Left stance, right swing
                mj_data->site_xpos[right_site_id * 3] = swingPos.x();
                mj_data->site_xpos[right_site_id * 3 + 1] = swingPos.y();
                mj_data->site_xpos[right_site_id * 3 + 2] = swingPos.z();

                mj_data->site_xpos[left_site_id * 3] = lastSwingPos_left.x();
                mj_data->site_xpos[left_site_id * 3 + 1] = lastSwingPos_left.y();
                mj_data->site_xpos[left_site_id * 3 + 2] = lastSwingPos_left.z();

                lastSwingPos_right = swingPos;
            }
            else if (RobotState.legState == DataBus::RSt && left_site_id >= 0)
            { // Right stance, left swing
                mj_data->site_xpos[left_site_id * 3] = swingPos.x();
                mj_data->site_xpos[left_site_id * 3 + 1] = swingPos.y();
                mj_data->site_xpos[left_site_id * 3 + 2] = swingPos.z();

                mj_data->site_xpos[right_site_id * 3] = lastSwingPos_right.x();
                mj_data->site_xpos[right_site_id * 3 + 1] = lastSwingPos_right.y();
                mj_data->site_xpos[right_site_id * 3 + 2] = lastSwingPos_right.z();

                lastSwingPos_left = swingPos;
            }

            // if (base_des_site_id >= 0)
            // {
            //     mj_data->site_xpos[base_des_site_id * 3] = RobotState.swingDesPosFinal_W(0);
            //     mj_data->site_xpos[base_des_site_id * 3 + 1] = RobotState.swingDesPosFinal_W(1);
            //     mj_data->site_xpos[base_des_site_id * 3 + 2] = RobotState.swingDesPosFinal_W(2);
            // }

            //**** END OF VISUALIZING PLANNED FOOT STEPS ****//
        }

        if (mj_data->time >= simEndTime)
            break;

        uiController.updateScene();
    };
    // free visualization storage
    uiController.Close();

    return 0;
}
