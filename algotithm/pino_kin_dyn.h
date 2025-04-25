/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#pragma once

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "data_bus.h"
#include <string>
#include "json/json.h"
#include "useful_math.h"
#include <vector>

class Pin_KinDyn {
public:
    // 电机是否达到限位的标志数组
    std::vector<bool> motorReachLimit;
    // 电机名称列表，与 URDF 和 JSON 配置文件中的关节名称对应
    const std::vector<std::string> motorName={"joint1","joint2","joint3","joint4","joint5","joint6","joint7"}; // joint name in urdf and jason config files
    Eigen::VectorXd motorMaxTorque;
    Eigen::VectorXd motorMaxPos;
    Eigen::VectorXd motorMinPos;

    
    Eigen::VectorXd tauJointOld;// 上一时刻的关节扭矩
    Eigen::VectorXd qJointOld;
    std::string urdf_path;//input
    
    pinocchio::Model model_biped_fixed;// 固定状态下双足机器人的 Pinocchio 模型
    int model_nv;
    pinocchio::JointIndex hand;
    Eigen::VectorXd q,dq,ddq;
    
    Eigen::Matrix3d Rcur;// 当前的旋转矩阵   
    Eigen::Quaternion<double> quatCur;// 当前的四元数
    
    Eigen::Matrix<double,6,-1> J_h;//雅可比矩阵.雅可比矩阵导数
    Eigen::Matrix<double,6,-1> dJ_h;   
    Eigen::Vector3d base_pos;
    Eigen::Vector3d hd_pos;  // hand position in world frame
    Eigen::Vector3d hd_pos_body; // hand position in body frame
    Eigen::Matrix3d hd_rot;
    Eigen::Matrix3d hd_rot_body;
   
    Eigen::MatrixXd dyn_M, dyn_M_inv, dyn_C, dyn_G, dyn_Ag, dyn_dAg; // 动力学矩阵、动力学矩阵的逆、科里奥利力矩阵、重力矩阵、质心动量矩阵、质心动量矩阵的导数   
    Eigen::VectorXd dyn_Non;// 动力学非线性项      
    Eigen::Matrix3d inertia;// 惯性矩阵

    // 逆运动学求解结果结构体
    struct IkRes{       
        int status;// 求解状态       
        int itr;// 迭代次数
        Eigen::VectorXd err;       
        Eigen::VectorXd jointPosRes;// 关节位置求解结果
    };

    Pin_KinDyn(std::string urdf_pathIn);// 构造函数，传入 URDF 文件路径
    void dataBusRead(DataBus const &robotState);// 从数据总线读取机器人状态
    void dataBusWrite(DataBus &robotState);
    void computeJ_dJ();// 计算雅可比矩阵及其导数
    void computeDyn();// 计算动力学相关矩阵
    IkRes computeInK_Hand(const Eigen::Matrix3d &Rdes_L, const Eigen::Vector3d &Pdes_L);// 计算手部逆运动学
    Eigen::VectorXd integrateDIY(const Eigen::VectorXd &qI, const Eigen::VectorXd &dqI);// 自定义的积分函数
    static Eigen::Quaterniond intQuat(const Eigen::Quaterniond &quat, const Eigen::Matrix<double,3,1> &w);// 四元数积分函数
    void workspaceConstraint(Eigen::VectorXd &qFT, Eigen::VectorXd &tauJointFT);// 工作空间约束函数
private:
    pinocchio::Data data_biped_fixed;
    int timestep=32;
};
