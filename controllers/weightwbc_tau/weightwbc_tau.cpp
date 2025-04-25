// File:          my_controller_cpp.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Node.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <sstream>
#include "useful_math.h"
#include "Wb_interface.h"
#include "pino_kin_dyn.h"
#include <fcntl.h>  // 添加文件锁相关头文件
#include <unistd.h>
#include <qpOASES.hpp>
// #include "pinocchio/parsers/urdf.hpp"
// #include "pinocchio/algorithm/joint-configuration.hpp"
// #include "pinocchio/algorithm/kinematics.hpp"
#define DOF 7
// All the webots classes are defined in the "webots" namespace
using MatrixRowMajor = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
int main(int argc, char **argv)
{
  // create the Robot instance.
  webots::Robot *robot = new webots::Robot();
  // 加载 URDF 模型到 Pinocchio
  std::ifstream file;
  // 使用完整的文件路径，确保有写入权限
  std::string filePath = "/home/user/kukawbc/record/ball_information.txt";
  /*file.open(filePath);
  if (!file.is_open()) {
    std::cout << "Failed to open the file." << std::endl;
    return 1; // 返回错误代码
  }*/
  int fd = open(filePath.c_str(), O_RDONLY);  // 以读取模式打开文件
  if (fd == -1) {
      std::cerr << "Failed to open the file for reading." << std::endl;
      return 1;
  }

  struct flock lock;
  lock.l_type = F_RDLCK;  // 读锁
  lock.l_whence = SEEK_SET;
  lock.l_start = 0;
  lock.l_len = 0;
  std::string urdf_path = "/home/user/kukawbc/lbr_iiwa7_r800.urdf";
  Wb_Interface wb_interface(robot);  
  // 运动学和动力学求解器，使用 URDF 文件初始化
  Pin_KinDyn kinDynSolver(urdf_path); 
  // 数据总线，用于在不同模块间传递数据
  DataBus RobotState(kinDynSolver.model_nv); 
  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();
  int time_iter = 0;

  // 目标位置，相对于基坐标系
  Eigen::Vector3d pos_target;
  Eigen::VectorXd des_target=Eigen::VectorXd::Zero(6);
  Eigen::Vector3d angle_target;
  Eigen::Matrix3d rot_target;
  // 正常target
  // 有grad，无振荡
  // 无grad，无振荡
  // pos_target << 1.41, 3.41, 0.2;

  // 雅可比奇异target
  // 该target无论是否有grad，速度指令都有轻微振荡
  // pos_target << 1.0, 4.0, 0;

  // 关节限位target
  // 无论有无grad，都几乎没有振荡
  // pos_target << 4.0, 1.0, 0;

  // 雅可比奇异target
  // 该target无论是否有grad，速度指令都有剧烈振荡
  // pos_target << 0.0, 5.0, 0;

  // 雅可比奇异target
  // 如果没有grad，则不动
  // 如果有，则不会稳定到零位22
  // pos_target << 0.0, 4.0, 0;

  // testtarget
  // 无论有无grad，都几乎没有振荡
  //pos_target << 0.2,0,1.1;//0.3,-0.2,0.9;0.2,-0.2,1.0;0.2,-0.2,0.87;-0.34,r>0.4,<0.8
  /*rot_target << 1,0,0,
                0,1,0,
                0,0,1;
  /*rot_target << 0,0,1,
                0,1,0,
                -1,0,0;*/
  
  //angle_target << 0,0,0;
  double KP = 20;//20.20+xianfu2,15
  double KD = 25;//10.5+xainfu2,15
  // 关节速度限制
  Eigen::VectorXd up_vel_limit(DOF);
  up_vel_limit << 1.71,1.71,1.75,2.27,2.24,3.14,3.14;
  Eigen::VectorXd low_vel_limit(DOF);
  low_vel_limit << -1.71,-1.71,-1.75,-2.27,-2.24,-3.14,-3.14;
  // 关节加速度限制
  Eigen::VectorXd up_acc_limit;
  up_acc_limit.setZero(DOF);
  up_acc_limit << 1e1, 1e1, 1e1, 1e1, 1e1, 1e1, 1e1;//1e2, 1e2, 1e2, 140, 140, 200, 200;//1e1, 1e1, 1e1, 1e1, 1e1, 1e1, 1e1;
  Eigen::VectorXd low_acc_limit;
  low_acc_limit.setZero(DOF);
  low_acc_limit << -1e1, -1e1, -1e1, -1e1, -1e1, -1e1, -1e1;//-1e2, -1e2, -1e2, -140, -140, -200, -200;//-1e1, -1e1, -1e1, -1e1, -1e1, -1e1, -1e1;
  // 关节力矩限制
  Eigen::VectorXd up_tau_limit;
  up_tau_limit.setZero(DOF);
  up_tau_limit << 176.0, 176.0, 110.0, 110.0, 110.0, 40.0, 40.0;
  Eigen::VectorXd low_tau_limit;
  low_tau_limit.setZero(DOF);
  low_tau_limit << -176.0, -176.0, -110.0, -110.0, -110.0, -40.0, -40.0;
  double threshold = 0.1;

  // 关节位置限制
  Eigen::VectorXd up_pos_limit(DOF);
  Eigen::VectorXd low_pos_limit(DOF);
  up_pos_limit << 2.97,2.09,2.97,2.09,2.97,2.09,3.05;
  low_pos_limit << -2.97,-2.09,-2.97,-2.09,-2.97,-2.09,-3.05;
  
  bool init = false;
  Eigen::VectorXd pre_pos(DOF);
  pre_pos.setZero();
  Eigen::VectorXd pre_vec(DOF);
  pre_vec.setZero();
  //bool isFirstIteration = true;
  // Main loop:
  while (robot->step(timeStep) != -1)
  {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    // double val = ds->getValue();
    double time = double(time_iter) * timeStep / 1000;
    std::cout << "************ time= " << time << " ************" << std::endl;
    //pos_target[1]+=0.3* (double(timeStep) / 1000);
    //pos_target[2]-=0.01* (double(timeStep) / 1000);

    
    // getJointJacobian(LOCAL_WORLD_ALIGNED)
    // 获取锁
    if (fcntl(fd, F_SETLKW, &lock) == -1) {
      perror("fcntl");
      continue;
    }
    std::ifstream file(filePath);
    if (!file.is_open()) {
      std::cout << "Failed to open the file." << std::endl;
      return 1;
    }    
    std::string line;    
    while (std::getline(file, line)) {
        if (line == "position") {
          for(int i=0;i<3;++i){
            std::getline(file, line);
            std::istringstream iss(line);
            iss >> pos_target(i);
          }           
          break;
        }
    }
    while (std::getline(file, line)) {
      if (line == "rotation") {
          for (int i = 0; i < 3; ++i) {
              std::getline(file, line);
              std::istringstream iss(line);
              for (int j = 0; j < 3; ++j) {
                  iss >> rot_target(i, j);
              }
          }
          break;
      }
    }
    file.close();
    // 释放锁
    lock.l_type = F_UNLCK;
    if (fcntl(fd, F_SETLK, &lock) == -1) {
      perror("fcntl");
    }
    angle_target=Rot2eul(rot_target);
    des_target.block(0,0,3,1)=pos_target;
    des_target.block(3,0,3,1)=angle_target;
    //std::cout << "pos_target after modification: " << pos_target.transpose() << std::endl;
    std::cout << "des_target after modification: " << des_target.transpose() << std::endl;
    // 更新传感器值
    wb_interface.updateSensorValues();
    // 将传感器数据写入数据总线
    wb_interface.dataBusWrite(RobotState);
    // 更新运动学和动力学信息
    // 从数据总线读取机器人状态
    kinDynSolver.dataBusRead(RobotState);
    // 计算雅可比矩阵及其导数
    kinDynSolver.computeJ_dJ();//T_b
    // 计算动力学信息
    kinDynSolver.computeDyn();
    // 将计算结果写入数据总线
    kinDynSolver.dataBusWrite(RobotState);
    
    // calc real position and velocity
    Eigen::VectorXd pos_real(DOF);
    pos_real=RobotState.q;
    Eigen::VectorXd vec_real(DOF);
    Eigen::VectorXd acc_real(DOF);

    vec_real = (pos_real - pre_pos) / (double(timeStep) / 1000);
    acc_real = (vec_real - pre_vec) / (double(timeStep) / 1000);
    pre_pos = pos_real;
    pre_vec = vec_real;
    std::cout << "pos_real= " << pos_real.transpose() << std::endl;
    std::cout << "vec_real= " << vec_real.transpose() << std::endl;
    std::cout << "acc_real= " << acc_real.transpose() << std::endl;

    auto Jac = RobotState.J_h;
    auto dJac = RobotState.dJ_h;
    int num_var =  DOF + 6;
    int num_output = 6; // 平面问题，速度只有3个变量


    // Hess
    Eigen::MatrixXd Hess = Eigen::MatrixXd(num_var, num_var);
    Hess.block(0, 0, DOF, DOF).diagonal() << 20,20,20,20,10,10,1;//20,20,20,20,10,10,1;//
    Hess.block(DOF, DOF, num_output, num_output).diagonal() << 1e4,1e4,1e4,1e4,1e4,1e4;//1e6,1e6,1e6,1e6,1e6,1e6;//1e5,1e5,1e5,1e5,1e5,1e5;//1e4,1e4,1e4,1e4,1e4,1e4;



    //grad
    Eigen::VectorXd grad(num_var);
    grad.setZero();


    //lbx, ubx
    Eigen::VectorXd lbx(num_var);
    Eigen::VectorXd ubx(num_var);
    lbx.head(DOF) = low_acc_limit;
    lbx.segment(DOF, num_output) << -1e6, -1e6, -1e6,-1e6, -1e6, -1e6;
    ubx.head(DOF) = up_acc_limit;
    ubx.segment(DOF, num_output) << 1e6, 1e6, 1e6,1e6, 1e6, 1e6;
    //根据关节位置处理加速度限制
    /*for (int i = 0; i < DOF; ++i) {
      double temp_up = up_pos_limit(i) - pos_real(i);
      ubx(i) = temp_up > threshold ? up_acc_limit(i)
                                   : (temp_up / threshold) * up_acc_limit(i);
      if (temp_up < 0) {
        ubx(i) = 0.0;
      }
      double temp_low = pos_real(i) - low_pos_limit(i);
      lbx(i) = temp_low > threshold ? low_acc_limit(i)
                                    : (temp_low / threshold) * low_acc_limit(i);
      if (temp_low < 0) {
        lbx(i) = 0.0;
      }
    }*/


    // CST and lbc ubc
    Eigen::MatrixXd Cst = Eigen::MatrixXd(num_output , num_var);
    Cst.setZero();
    Cst.block(0, 0, num_output, DOF) = Jac;
    Cst.block(0, DOF, num_output, num_output).setIdentity();

    double EPS = 1e-6;

    Eigen::VectorXd lbc(num_output);
    Eigen::VectorXd ubc(num_output);

    Eigen::VectorXd vec_real_end(num_output);
    Eigen::VectorXd dJdq(num_output);
    Eigen::VectorXd error_end(num_output);
    Eigen::VectorXd error_rot_end(num_output);
    //Eigen::Matrix3d error_rot;
    vec_real_end = Jac * vec_real;            // 真实末端坐标系速度旋量
    dJdq = dJac * vec_real;            // 真实末端坐标系速度旋量
    auto pos_end = RobotState.hd_pos_L;     // 真实末端坐标系位置
    auto angle_end = Rot2eul(RobotState.hd_rot_L);     // 真实末端坐标系位置
    Eigen::VectorXd acc_target=Eigen::VectorXd::Zero(6);               // 末端坐标系目标速度旋量
    Eigen::VectorXd cur_end=Eigen::VectorXd::Zero(6);
    Eigen::VectorXd o=Eigen::VectorXd::Zero(6); 
    cur_end.block(0,0,3,1)=pos_end;
    cur_end.block(3,0,3,1)=angle_end;
    error_end.block(0,0,3,1)=pos_target - pos_end;
    error_end.block(3,0,3,1)=diffRot(RobotState.hd_rot_L,rot_target);
    acc_target = error_end * KP+(o - vec_real_end ) * KD; // 比例控制

    //xianfu
    for (int i=0;i<3;i++){
      if(acc_target(i)>2){
        acc_target(i)=2;//2,1.9
      }
      else if(acc_target(i)<-2){
        acc_target(i)=-2;
      }
    }
    for (int i=3;i<6;i++){
      if(acc_target(i)>15){
        acc_target(i)=15;//15
      }
      else if(acc_target(i)<-15){
        acc_target(i)=-15;
      }
    }
    // vec_target << 0,1,0;
    lbc.head(num_output) = (acc_target-dJdq).array() - EPS;
    ubc.head(num_output) = (acc_target-dJdq).array() + EPS;

    Eigen::VectorXd res(num_var);
    res.setZero();

    qpOASES::QProblem qp(num_var, num_output );
    qpOASES::Options options;
    // options.setToReliable();
    options.setToMPC();
    options.terminationTolerance = 1e-6;
    //options.printLevel = qpOASES::PL_HIGH;
    options.printLevel = qpOASES::PL_NONE;
    qp.setOptions(options);
    int nWSR = 1e4;
    MatrixRowMajor Hess_row(Hess);
    MatrixRowMajor Cst_row(Cst);
    int init_qp = qp.init(Hess_row.data(), grad.data(), Cst_row.data(), lbx.data(), ubx.data(), lbc.data(), ubc.data(), nWSR);
    if (init_qp != qpOASES::SUCCESSFUL_RETURN)
    {
      std::cout << "Failed solve qp" << std::endl;
    }
    else
    {
      qp.getPrimalSolution(res.data());
    }

    
    std::cout << "real error end pos = [" << error_end.transpose() << "];" << std::endl;
    std::cout << "des end pos = [" << des_target.transpose() << "];" << std::endl;
    std::cout << "target end velocity screw = [" << acc_target.transpose() << "];"  << std::endl;
    std::cout << "dJ*dq = [" << dJdq.transpose() << "];"  << std::endl;
    std::cout << "vec_real_end = [" << vec_real_end.transpose() << "];"  << std::endl;
    //std::cout << "dyn_M = [" << RobotState.dyn_M << "];"  << std::endl;
    //std::cout << "dyn_C = [" << RobotState.dyn_C << "];"  << std::endl;
    //std::cout << "dyn_G = [" << RobotState.dyn_G.transpose() << "];"  << std::endl;
    // std::cout << "Hess = [\n" << Hess << "];" << std::endl;
    // std::cout << "grad = [" << grad.transpose() << "];" << std::endl;
  
    std::cout << "Cst = [\n"
              << Cst << "];" << std::endl;
    std::cout << "lbx = [" << lbx.transpose() << "];" << std::endl;
    std::cout << "ubx = [" << ubx.transpose() << "];" << std::endl;
    std::cout << "lbc = [" << lbc.transpose() << "];" << std::endl;
    std::cout << "ubc = [" << ubc.transpose() << "];" << std::endl;
    std::cout << "opt_value = [" << res.transpose() << "];" << std::endl;
    std::cout << "Cst * Opt = " << (Cst * res).transpose() << std::endl;
    std::cout<<"Jacobian * ddq =  [";
    std::cout<<(Jac * res.head(DOF)).transpose()<<" ];"<<std::endl;
    
    std::cout << "mt = " << std::sqrt((Jac * Jac.transpose()).determinant()) << std::endl;

    // 发布jia速度指令
    for (int i = 0; i < DOF; ++i)
    {
      if (pos_real(i) > up_pos_limit(i) - 1e-6)
      {
        // res(i) = -1e-3;
        res(i) = 0.0;
      }
      if (pos_real(i) < low_pos_limit(i) + 1e-6)
      {
        // res(i) = -1e-3;
        res(i) = 0.0;
      }
    }
    Eigen::VectorXd tau(DOF);
    tau=RobotState.dyn_M*res.head(DOF)+RobotState.dyn_Non;
    double delta_T = double(timeStep) / 1000;
    vec_real += res.head(DOF) * (delta_T);
    std::vector<double> res_v=eigen2std(vec_real);//ddq
    std::vector<double> res_f=eigen2std(tau);
    //wb_interface.setMotorsVelocity(res_v);
    wb_interface.setMotorsTorque(res_f);
    time_iter++;

    std::cout << "vel_cmd = [" << vec_real.transpose() << "];" << std::endl;
    std::cout << "tau_cmd = " << tau.transpose() << std::endl;

  };

  // Enter here exit cleanup code.
  close(fd);
  delete robot;
  return 0;
}
