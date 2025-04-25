// File:          ball_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
//#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <fstream>
#include "useful_math.h"
#include <fcntl.h>  // 添加文件锁相关头文件
#include <unistd.h>
// All the webots classes are defined in the "webots" namespace
using namespace webots;
#define pi 3.1415926
int main(int argc, char **argv) {
  // create the Robot instance.
  Supervisor *supervisor = new Supervisor();
  // 获取小球节点
  Node *ball = supervisor->getFromDef("ball");
  if (ball == NULL) {
      std::cerr << "未找到定义为BALL的节点" << std::endl;
      return 1;
  }
  // 获取小球的位置域和方向域
  Field *translationField = ball->getField("translation");
  Field *rotationField = ball->getField("rotation");

  
  //const   double  dt = 32*0.001;
  //const   double  dt_200Hz = 0.005;  
  // 初始化各类对象
  int timeStep = (int)supervisor->getBasicTimeStep();

  // 模拟参数
  double vx = 0;//m/s
  double vy = 0.15;//0.5;
  double vz= 0;//0.3平移,0.26
  double t = 0.0;
  //double w=0.5;
  double rotationxSpeed = 0;  // 绕Z轴旋转速度
  double rotationySpeed = 0;  // 绕Z轴旋转速度
  double rotationzSpeed = 0;  // 绕Z轴旋转速度

  std::ofstream file;
  // 使用完整的文件路径，确保有写入权限
  std::string filePath = "/home/user/kukawbc/record/ball_information.txt";
  /*file.open(filePath);
  if (!file.is_open()) {
    std::cout << "Failed to open the file." << std::endl;
    return 1; // 返回错误代码
  }*/
  int fd = open(filePath.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0666);  // 以写入模式打开文件并创建（如果不存在）
  if (fd == -1) {
      std::cerr << "Failed to open the file for writing." << std::endl;
      return 1;
  }

  struct flock lock;
  lock.l_type = F_WRLCK;  // 写锁
  lock.l_whence = SEEK_SET;
  lock.l_start = 0;
  lock.l_len = 0;
  // Main loop: 
  // - perform simulation steps until Webots is stopping the controller
  while (supervisor->step(timeStep) != -1) {       
    //如果超过边界，往回走  
    // 计算新的位置
    double x = 0.2+vx*t;//0.4+vx*t,0.2,-0.2,1.2too close//0.5,-0.5,1.1bad
    double y = -0.2+vy*t;//0+vy*t
    double z = 1.0+vz*t;  // 1.0+vz*t.假设在地面上
    
    double newPosition[3] = {x, y, z};
    // 设置新的位置
    //translationField->setSFVec3f(newPosition);
    t += (double)timeStep / 1000.0;
    
     // 计算姿态变化（绕Z轴旋转）
    double angle_z= rotationzSpeed * t;
    double angle_x= rotationxSpeed * t;
    double angle_y= rotationySpeed * t;
    Eigen::Quaterniond newOrientation=eul2quat(angle_x,angle_y,angle_z);
    newOrientation.normalize();
    Eigen::Matrix<double, 4, 1> res=quat2axisAngle(newOrientation);
    double x_0=res(0);
    double y_0=res(1);
    double z_0=res(2);
    double angle_0=res(3);
    double newOrientation_1[4]={0,1,0,1.57};
    rotationField->setSFRotation(newOrientation_1);

    // 获取小球的位置
    const double *ballPosition = ball->getPosition();
    double ballX = ballPosition[0];
    double ballY = ballPosition[1];
    double ballZ = ballPosition[2];
    // 获取小球的位置
    const double *ballOrientation = ball->getOrientation();

    // 左手期望的局部位置
    Eigen::Vector3d hd_pos_des_W ={ballX,ballY,ballZ};// {-0.02, 0.32, 0.8};
    // 左手期望的局部欧拉角
    //Eigen::Vector3d hd_eul_L_des = {ballRX,ballRY,ballRZ};//rx,ry,rz
    // 左手期望的旋转矩阵
    // 将数据填充到Eigen矩阵中
    Eigen::Matrix3d hd_rot_des_W = Eigen::Matrix3d::Zero();
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        hd_rot_des_W(i, j) = ballOrientation[i * 3 + j];
      }
    }  
    
    // 获取锁
    if (fcntl(fd, F_SETLKW, &lock) == -1) {
      perror("fcntl");
      continue;
    }

    std::ofstream file(filePath, std::ios::trunc);  // 以追加模式写入app
    file << "position" << std::endl;
    file << hd_pos_des_W << std::endl;
    file << "rotation" << std::endl;
    file << hd_rot_des_W << std::endl;
    file.flush();
    file.close();

    // 释放锁
    lock.l_type = F_UNLCK;
    if (fcntl(fd, F_SETLK, &lock) == -1) {
      perror("fcntl");
    } 
      // 假设这里有获取小球位置的代码，将位置信息写入文件
    /*file << "position" << std::endl;  // 示例小球位置信息
    file << hd_pos_des_W << std::endl;  // 示例小球位置信息
    file << "rotation" << std::endl;  // 示例小球位置信息
    file << hd_rot_des_W << std::endl;  // 示例小球位置信息
     
    // 刷新文件缓冲区
    file.flush();*/
  }
  //file.close();
  close(fd);
  delete supervisor;
  return 0;
}