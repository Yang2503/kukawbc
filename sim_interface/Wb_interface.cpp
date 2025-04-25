/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#include "Wb_interface.h"
#include "useful_math.h"

Wb_Interface::Wb_Interface(webots::Robot* robot) : robot(robot){
    jointNum=JointName.size();
    motor_pos.assign(jointNum,0);
    motor_vel.assign(jointNum,0);
    motor_acc.assign(jointNum,0);
    motor_pos_Old.assign(jointNum,0);
    motor_vel_Old.assign(jointNum,0);
    basePos[0]=-0.013;
    basePos[1]=0;
    basePos[2]=0.07;
    // 初始化电机和位置传感器
    for (const auto& name : JointName) {
        webots::Motor* motor = robot->getMotor(name);
        if (motor) {
            motors.push_back(motor);
            webots::PositionSensor* sensor = motor->getPositionSensor();
            if (sensor) {
                timeStep = static_cast<int>(robot->getBasicTimeStep());
                sensor->enable(timeStep);
                positionSensors.push_back(sensor);
            }
        }
    }

    isIni = true;
}



void Wb_Interface::updateSensorValues() {
    for (int i=0;i<jointNum;i++){
        motor_pos_Old[i]=motor_pos[i];
        motor_vel_Old[i]=motor_vel[i];
        motor_pos[i] = positionSensors[i]->getValue();   
        motor_vel[i] = (motor_pos[i] - motor_pos_Old[i]) / (double(timeStep) / 1000);//速度传感器？
        motor_acc[i] = (motor_vel[i] - motor_vel_Old[i]) / (double(timeStep) / 1000);//速度传感器？
    }
     
}

void Wb_Interface::setMotorsVelocity(std::vector<double> &velIn) {
    for (int i=0;i<jointNum;i++){
        motors[i]->setPosition(INFINITY);
        motors[i]->setVelocity(velIn[i]);
    }
}

void Wb_Interface::setMotorsTorque(std::vector<double> &tauIn) {
    for (int i=0;i<jointNum;i++)
    motors[i]->setTorque(tauIn[i]);
}


void Wb_Interface::dataBusWrite(DataBus &busIn) {
    busIn.motors_pos_cur=motor_pos;
    busIn.motors_vel_cur=motor_vel;
    busIn.q=Eigen::Map<Eigen::VectorXd>(motor_pos.data(), motor_pos.size());
    busIn.dq=Eigen::Map<Eigen::VectorXd>(motor_vel.data(), motor_vel.size());
    busIn.ddq=Eigen::Map<Eigen::VectorXd>(motor_acc.data(), motor_acc.size());
    busIn.basePos[0]=basePos[0];
    busIn.basePos[1]=basePos[1];
    busIn.basePos[2]=basePos[2];  
    busIn.updateQ();
}








