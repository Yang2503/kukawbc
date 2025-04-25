/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#pragma once

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include "data_bus.h"
#include <string>
#include <vector>

class Wb_Interface {
public:
    int jointNum{0};
    std::vector<double> motor_pos;//存储当前和上一时刻的关节位置
    std::vector<double> motor_pos_Old;
    std::vector<double> motor_vel;   
    std::vector<double> motor_acc;
    std::vector<double> motor_vel_Old;  
    double basePos[3]{0}; // position of baselink, in world frame
    const std::vector<std::string> JointName={"joint1","joint2","joint3","joint4","joint5","joint6","joint7"}; // joint name in XML file, the corresponds motors name should be M_*, ref to line 29 of MJ_Interface.cpp
    
    Wb_Interface(webots::Robot* robot);
    void updateSensorValues();
    void setMotorsVelocity(std::vector<double> &velIn);
    void setMotorsTorque(std::vector<double> &tauIn);
    void dataBusWrite(DataBus &busIn);

private:
    webots::Robot* robot;
    std::vector<webots::Motor*> motors;
    std::vector<webots::PositionSensor*> positionSensors;
    int timeStep = (int)robot->getBasicTimeStep();
    bool isIni{false};
};


