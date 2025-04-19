//
// Created by boxing on 24-1-12.
//
#pragma once

#include "Eigen/Dense"
#include <iostream>
#include <vector>
#include "iomanip"

struct DataBus{
    const int model_nv; // number of dq

    // motors, sensors and states feedback 
    double basePos[3];
    std::vector<double> motors_pos_cur;
    std::vector<double> motors_vel_cur;
    std::vector<double> motors_tor_cur;
    std::vector<double> maxTor;
    std::vector<double> maxVel;
    std::vector<double> maxPos;
    std::vector<double> minPos;
    std::vector<double> maxAcc;
    bool isdqIni;

    // PVT controls
    std::vector<double> motors_pos_des;
    std::vector<double> motors_vel_des;
    std::vector<double> motors_tor_des;
    std::vector<double> motors_tor_out;

    // states and key variables
    Eigen::VectorXd q, dq, ddq;
    Eigen::VectorXd qOld;
    Eigen::MatrixXd J_h;
    Eigen::MatrixXd dJ_h;
    Eigen::Vector3d base_pos;
    Eigen::Vector3d hd_pos_W; // in world frame
    Eigen::Matrix3d hd_rot_W;
    Eigen::Vector3d hd_pos_L; // in body frame
    Eigen::Matrix3d hd_rot_L;
    Eigen::Vector3d hd_pos_old_W;
    Eigen::Matrix3d hd_rot_old_W;
    Eigen::Vector3d hd_pos_des_W; // in world frame
    Eigen::Matrix3d hd_rot_des_W;
    Eigen::Vector3d hd_pos_des_L; // in body frame
    Eigen::Matrix3d hd_rot_des_L;
    Eigen::VectorXd qCmd, dqCmd;
    Eigen::VectorXd tauJointCmd;
    Eigen::MatrixXd dyn_M, dyn_M_inv, dyn_C, dyn_Ag, dyn_dAg;
    Eigen::VectorXd dyn_G, dyn_Non,hd_quat;

    Eigen::VectorXd hpvel;
    Eigen::VectorXd hptau;

    Eigen::Vector3d slop;
    Eigen::Matrix<double,3,3>   inertia;

    // cmd value from the joystick interpreter
    Eigen::Vector3d     js_eul_des;
    Eigen::Vector3d     js_pos_des;
    Eigen::Vector3d     js_omega_des;
    Eigen::Vector3d     js_vel_des;

    // cmd values for MPC
    Eigen::VectorXd     Xd;
    Eigen::VectorXd     X_cur;
    Eigen::VectorXd     X_cal;
    Eigen::VectorXd     dX_cal;
    Eigen::VectorXd     dxcmd,ddxcmd;


    int 	qp_nWSR_MPC;
    double 	qp_cpuTime_MPC;
    int 	qpStatus_MPC;

    // cmd values for WBC

    Eigen::Vector3d base_pos_des;
    Eigen::VectorXd des_ddq, des_dq, des_delta_q, des_q;
    Eigen::VectorXd wbc_delta_q_final, wbc_dq_final, wbc_ddq_final;
    Eigen::VectorXd wbc_tauJointRes;

    int qp_nWSR;
    double qp_cpuTime;
    int qp_status;

    enum MotionState{
        Track    
    };
    double thetaZ_des{0};
    MotionState motionState;

    DataBus(int model_nvIn): model_nv(model_nvIn){
        motors_pos_cur.assign(model_nv,0);
        motors_vel_cur.assign(model_nv,0);
        motors_tor_out.assign(model_nv,0);
        motors_tor_cur.assign(model_nv,0);
        motors_tor_des.assign(model_nv,0);
        motors_vel_des.assign(model_nv,0);
        motors_pos_des.assign(model_nv,0);
        maxTor.assign(model_nv,0);
        maxVel.assign(model_nv,0);
        maxPos.assign(model_nv,0);
        minPos.assign(model_nv,0);
        q=Eigen::VectorXd::Zero(model_nv);
        qOld=Eigen::VectorXd::Zero(model_nv);
        dq=Eigen::VectorXd::Zero(model_nv);
        ddq=Eigen::VectorXd::Zero(model_nv);
        qCmd=Eigen::VectorXd::Zero(model_nv);
        dqCmd=Eigen::VectorXd::Zero(model_nv);
        dxcmd=Eigen::VectorXd::Zero(model_nv-1);
        ddxcmd=Eigen::VectorXd::Zero(model_nv-1);
        tauJointCmd=Eigen::VectorXd::Zero(model_nv);
        Xd = Eigen::VectorXd::Zero(12*10);
        X_cur = Eigen::VectorXd::Zero(12);
        X_cal = Eigen::VectorXd::Zero(12);
        dX_cal = Eigen::VectorXd::Zero(12);
        des_ddq = Eigen::VectorXd::Zero(model_nv);
        des_dq = Eigen::VectorXd::Zero(model_nv);
        des_delta_q = Eigen::VectorXd::Zero(model_nv);
        base_pos_des.setZero();
        js_eul_des.setZero();
        js_pos_des.setZero();
        js_omega_des.setZero();
        js_vel_des.setZero();
        motionState = Track;
    };

    // update q according to sensor values, must update sensor values before
    void updateQ(){       

        //  q = [global_base_position, global_base_quaternion, joint_positions]
        //  dq = [global_base_velocity_linear, global_base_velocity_angular, joint_velocities]
        for (int i=0;i<model_nv;i++)
            q(i)=motors_pos_cur[i];

        
        for (int i=0;i<model_nv;i++)
        {
            dq(i)=motors_vel_cur[i];
        }
        qOld=q;
    }


    static void printdq(const Eigen::VectorXd &q){
        std::cout<<std::setprecision(5)<<q.block<6,1>(0,0).transpose()<<std::endl;
        std::cout<<std::setprecision(5)<<q.block<7,1>(6,0).transpose()<<std::endl;
        std::cout<<std::setprecision(5)<<q.block<7,1>(13,0).transpose()<<std::endl;
        std::cout<<std::setprecision(5)<<q.block<4,1>(20,0).transpose()<<std::endl;
        std::cout<<std::setprecision(5)<<q.block<6,1>(24,0).transpose()<<std::endl;
        std::cout<<std::setprecision(5)<<q.block<6,1>(30,0).transpose()<<std::endl;
    }

    static void printq(const Eigen::VectorXd &q){
        std::cout<<std::setprecision(5)<<q.block<7,1>(0,0).transpose()<<std::endl;
        std::cout<<std::setprecision(5)<<q.block<7,1>(7,0).transpose()<<std::endl;
        std::cout<<std::setprecision(5)<<q.block<7,1>(14,0).transpose()<<std::endl;
        std::cout<<std::setprecision(5)<<q.block<4,1>(21,0).transpose()<<std::endl;
        std::cout<<std::setprecision(5)<<q.block<6,1>(25,0).transpose()<<std::endl;
        std::cout<<std::setprecision(5)<<q.block<6,1>(31,0).transpose()<<std::endl;
    }

    Eigen::Matrix<double, 3, 3> eul2Rot(double roll, double pitch, double yaw) {
        Eigen::Matrix<double,3,3> Rx,Ry,Rz;
        Rz<<cos(yaw),-sin(yaw),0,
                sin(yaw),cos(yaw),0,
                0,0,1;
        Ry<<cos(pitch),0,sin(pitch),
                0,1,0,
                -sin(pitch),0,cos(pitch);
        Rx<<1,0,0,
                0,cos(roll),-sin(roll),
                0,sin(roll),cos(roll);
        return Rz*Ry*Rx;
    }

    /*Eigen::Quaterniond eul2quat(double roll, double pitch, double yaw) {
        Eigen::Matrix3d R= eul2Rot(roll,pitch,yaw);
        Eigen::Quaternion<double> quatCur;
        quatCur = R; //rotation matrix converted to quaternion
        Eigen::Quaterniond resQuat;
        resQuat=quatCur;
        return resQuat;
    }*/
};

