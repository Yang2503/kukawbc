/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/

//-------------------------------------------NOTE------------------------------------//
//
// The damping(Kd) in the joint_ctrl_config.json is relatively large, and they may not match the real ones.
//
//-----------------------------------------------------------------------------------//
#pragma once
#include <fstream>
#include "json/json.h"
#include <string>
#include <vector>
#include <cmath>
#include "data_bus.h"

class PVT_Ctr {
public:
    int jointNum;
    std::vector<double> motor_pos_cur;//当前关节位置。
    std::vector<double> motor_pos_des_old;//上一时刻的期望关节位置。
    std::vector<double> motor_vel;//关节速度。
    std::vector<double> motor_tor_out; // 最终输出的关节扭矩。
    std::vector<double> motor_pos_des; // P des
    std::vector<double> motor_vel_des; // V des
    std::vector<double> motor_tor_des; // T des

    std::vector<double> pvt_Kp;
    std::vector<double> pvt_Kd;
    std::vector<double> maxTor;
    std::vector<double> maxVel;
    std::vector<double> maxPos;
    std::vector<double> minPos;
    std::vector<double> maxAcc;
    PVT_Ctr(const char * jsonPath);
    void calMotorsPVT();//计算关节的 PVT 控制输出
    void calMotorsPVT(double deltaP_Lim);//考虑位置变化限制的 PVT 控制输出计算。
    void enablePV(); // 启用所有关节的 PV 控制。
    void disablePV(); // disable PV control item
    void enablePV(int jtId); // 启用指定关节的 PV 控制
    void disablePV(int jtId); // disable PV control item
    void setJointPD(double kp, double kd, const char * jointName);//用于设置指定关节的位置和速度控制增益。
    void dataBusRead(DataBus &busIn);
    void dataBusWrite(DataBus &busIn);

    

private:
    std::vector<int> PV_enable;//一个整数向量，用于标记每个关节的位置 - 速度（PV）控制是否启用。
    double sign(double in);
    const std::vector<std::string> motorName={"joint1","joint2","joint3","joint4","joint5","joint6","joint7"}; // joint name in urdf and jason config files
};


