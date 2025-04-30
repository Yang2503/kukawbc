/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#include "pino_kin_dyn.h"
#include "useful_math.h"
#include <utility>
// Pin_KinDyn 类的构造函数，接受 URDF 文件路径作为参数
Pin_KinDyn::Pin_KinDyn(std::string urdf_pathIn) {
    pinocchio::urdf::buildModel(urdf_pathIn,model_biped_fixed);// 根据给定的 URDF 文件路径构建固定基座的双足机器人模型
    data_biped_fixed=pinocchio::Data(model_biped_fixed);
    model_nv=model_biped_fixed.nv;
    
    J_h=Eigen::MatrixXd::Zero(6,model_nv); // 初始化左手雅可比矩阵为零矩阵  
    dJ_h=Eigen::MatrixXd::Zero(6,model_nv);
    J5_h=Eigen::MatrixXd::Zero(6,model_nv); // 初始化左手雅可比矩阵为零矩阵  
    dJ5_h=Eigen::MatrixXd::Zero(6,model_nv);
    
    q=Eigen::VectorXd::Zero(model_nv);
    dq=Eigen::VectorXd::Zero(model_nv);
    ddq=Eigen::VectorXd::Zero(model_nv);
    Rcur.setIdentity();
    dyn_M=Eigen::MatrixXd::Zero(model_nv,model_nv);
    dyn_M_inv=Eigen::MatrixXd::Zero(model_nv,model_nv);
    dyn_C=Eigen::MatrixXd::Zero(model_nv,model_nv);
    dyn_G=Eigen::MatrixXd::Zero(model_nv,1);

    // get joint index for Pinocchio Lib, need to redefined the joint name for new model   
    hand=model_biped_fixed.getJointId("joint7");
    small_ball=model_biped_fixed.getJointId("jointball");
    hand5=model_biped_fixed.getJointId("joint7");

    // 读取关节的 PVT（位置、速度、时间）参数
    Json::Reader reader;
    Json::Value root_read;
    // 以二进制模式打开名为 "joint_ctrl_config.json" 的文件
    std::ifstream in("joint_ctrl_config.json",std::ios::binary);

    motorMaxTorque=Eigen::VectorXd::Zero(motorName.size());
    motorMaxPos=Eigen::VectorXd::Zero(motorName.size());
    motorMinPos=Eigen::VectorXd::Zero(motorName.size());
    // 解析 JSON 文件内容到 root_read 对象
    reader.parse(in,root_read);
     // 遍历电机名称列表，从 JSON 文件中读取每个电机的最大扭矩、最大位置和最小位置
    for (int i=0;i<motorName.size();i++){
        motorMaxTorque(i)=(root_read[motorName[i]]["maxTorque"].asDouble());
        motorMaxPos(i)=(root_read[motorName[i]]["maxPos"].asDouble());
        motorMinPos(i)=(root_read[motorName[i]]["minPos"].asDouble());
    }
    
    motorReachLimit.assign(motorName.size(),false);// 将电机是否达到限位的标志列表初始化为 false，大小为电机名称列表的长度
    
    tauJointOld=Eigen::VectorXd::Zero(motorName.size());// 将上一时刻的关节扭矩向量初始化为零向量，大小为电机名称列表的长度
}

void Pin_KinDyn::dataBusRead(const DataBus &robotState) {
    //  For Pinocchio: The base translation part is expressed in the parent frame (here the world coordinate system)
    //  while its velocity is expressed in the body coordinate system.
    //  https://github.com/stack-of-tasks/pinocchio/issues/1137
    //  q = [global_base_position, global_base_quaternion, joint_positions]
    //  v = [local_base_velocity_linear, local_base_velocity_angular, joint_velocities]
    q.block(0,0,7,1)=robotState.q;
    dq.block(0,0,7,1)=robotState.dq;
    ddq.block(0,0,7,1)=robotState.ddq;
    base_pos=robotState.base_pos;
}

void Pin_KinDyn::dataBusWrite(DataBus &robotState) {
    robotState.J_h=J_h.block(0,0,6,7);
    robotState.dJ_h=dJ_h.block(0,0,6,7);
    robotState.hd_pos_L=hd_pos;/////?
    robotState.hd_rot_L=hd_rot;/////?
    robotState.hd_pos_W=hd_pos+base_pos;
    robotState.hd_rot_W=hd_rot;
    robotState.dyn_M=dyn_M.block(0,0,7,7);
    //robotState.dyn_M_inv=dyn_M_inv.block(0,0,6,7);
    //robotState.dyn_C=dyn_C.block(0,0,7,1);
    //robotState.dyn_G=dyn_G;
    //robotState.dyn_Ag=dyn_Ag;// 将质心动量矩阵写入数据总线
    //robotState.dyn_dAg=dyn_dAg;// 将质心动量矩阵的导数写入数据总线
    robotState.dyn_Non=dyn_Non.block(0,0,7,1);
    //robotState.inertia = inertia;  // w.r.t body frame,将惯性矩阵（相对于机体坐标系）写入数据总线

    robotState.J5_h=J5_h.block(0,0,3,7);
    robotState.dJ5_h=dJ5_h.block(0,0,3,7);
    //robotState.J6_h=J6_h.block(0,0,3,7);
    //robotState.dJ6_h=dJ6_h.block(0,0,3,7);
    robotState.small_pos=small_pos;
    robotState.link5_posW=link5_posW;
    
}

// update jacobians and joint positions
void Pin_KinDyn::computeJ_dJ() {

    
    pinocchio::forwardKinematics(model_biped_fixed,data_biped_fixed,q);    
    pinocchio::updateGlobalPlacements(model_biped_fixed,data_biped_fixed);
    pinocchio::getJointJacobian(model_biped_fixed,data_biped_fixed,hand,pinocchio::LOCAL_WORLD_ALIGNED,J_h);
    pinocchio::getJointJacobianTimeVariation(model_biped_fixed,data_biped_fixed,hand,pinocchio::LOCAL_WORLD_ALIGNED,dJ_h);
    hd_pos=data_biped_fixed.oMi[hand].translation();
    hd_rot=data_biped_fixed.oMi[hand].rotation();
    pinocchio::getJointJacobian(model_biped_fixed,data_biped_fixed,hand5,pinocchio::LOCAL_WORLD_ALIGNED,J5_h);
    pinocchio::getJointJacobianTimeVariation(model_biped_fixed,data_biped_fixed,hand5,pinocchio::LOCAL_WORLD_ALIGNED,dJ5_h);
    //pinocchio::getJointJacobian(model_biped_fixed,data_biped_fixed,6,pinocchio::LOCAL_WORLD_ALIGNED,J6_h);
    //pinocchio::getJointJacobianTimeVariation(model_biped_fixed,data_biped_fixed,6,pinocchio::LOCAL_WORLD_ALIGNED,dJ6_h);
    small_pos=data_biped_fixed.oMi[small_ball].translation();    


}
//实现了四元数的积分操作，从而更新了姿态表示。在机器人的姿态估计和控制中，这种方法常用于根据角速度信息实时更新机器人的姿态。(可delete)
Eigen::Quaterniond Pin_KinDyn::intQuat(const Eigen::Quaterniond &quat, const Eigen::Matrix<double, 3, 1> &w) {
    Eigen::Matrix3d Rcur=quat.normalized().toRotationMatrix();
    Eigen::Matrix3d Rinc=Eigen::Matrix3d::Identity();//初始化增量旋转矩阵
    double theta=w.norm();
    //说明有明显的旋转，需要进行增量旋转的计算；
    if (theta>1e-8) {
        Eigen::Vector3d w_norm;
        w_norm = w / theta;
        Eigen::Matrix3d a;//构建反对称矩阵 a，用于后续的旋转矩阵计算。反对称矩阵 a 与单位角速度向量 w_norm 相关，它在旋转矩阵的指数映射计算中起到关键作用。
        a << 0, -w_norm(2), w_norm(1),
                w_norm(0), 0, -w_norm(0),
                -w_norm(1), w_norm(0), 0;
        Rinc=Eigen::Matrix3d::Identity()+a*sin(theta)+a*a*(1-cos(theta));//根据罗德里格斯公式（Rodrigues' formula）计算增量旋转矩阵 Rinc。罗德里格斯公式描述了如何从旋转轴和旋转角度计算旋转矩阵。
    }
    Eigen::Matrix3d Rend=Rcur*Rinc;
    Eigen::Quaterniond quatRes;
    quatRes=Rend;
    return quatRes;
}

// intergrate the q with dq, for floating base dynamics
Eigen::VectorXd Pin_KinDyn::integrateDIY(const Eigen::VectorXd &qI, const Eigen::VectorXd &dqI) {
    Eigen::VectorXd qRes=Eigen::VectorXd::Zero(model_nv);    
    for (int i=0;i<model_nv;i++)
        qRes(i)=qI(i)+dqI(i);
    return qI;
}

// update dynamic parameters, M*ddq+C*dq+G=tau
void Pin_KinDyn::computeDyn() {
    // cal M
    pinocchio::crba(model_biped_fixed, data_biped_fixed, q);
    // Pinocchio only gives half of the M, needs to restore it here
    data_biped_fixed.M.triangularView<Eigen::Lower>() = data_biped_fixed.M.transpose().triangularView<Eigen::Lower>();
    dyn_M = data_biped_fixed.M;

    // cal Minv
    pinocchio::computeMinverse(model_biped_fixed, data_biped_fixed, q);
    data_biped_fixed.Minv.triangularView<Eigen::Lower>() = data_biped_fixed.Minv.transpose().triangularView<Eigen::Lower>();
    dyn_M_inv=data_biped_fixed.Minv;

    // cal C
    pinocchio::computeCoriolisMatrix(model_biped_fixed, data_biped_fixed, q, dq);
    dyn_C = data_biped_fixed.C;

    // cal G
    pinocchio::computeGeneralizedGravity(model_biped_fixed, data_biped_fixed, q);
    dyn_G = data_biped_fixed.g;

    // cal Ag, Centroidal Momentum Matrix. First three rows: linear, other three rows: angular
    pinocchio::dccrba(model_biped_fixed, data_biped_fixed,q,dq);
    pinocchio::computeCentroidalMomentum(model_biped_fixed, data_biped_fixed,q,dq);
    dyn_Ag=data_biped_fixed.Ag;
    dyn_dAg=data_biped_fixed.dAg;

    // cal nonlinear item
    dyn_Non=dyn_C*dq+dyn_G;

    // cal I
    pinocchio::ccrba(model_biped_fixed, data_biped_fixed, q, dq);
    inertia = data_biped_fixed.Ig.inertia().matrix();//提取质心处的惯性张量矩阵 
    pinocchio::centerOfMass(model_biped_fixed, data_biped_fixed, q); 
    link5_trans = data_biped_fixed.oMi[hand5].translation();
    link5_rot = data_biped_fixed.oMi[hand5].rotation();
    link5_posL=model_biped_fixed.inertias[hand5].lever();
    // 将连杆质心相对于连杆坐标系的位置变换到世界坐标系
    link5_posW = link5_trans+link5_rot*link5_posL;
        
//    std::cout<<"CoM_W"<<std::endl;
//    std::cout<<CoM_pos.transpose()<<std::endl;
}

// Inverse Kinematics for hand posture. Note: the Rdes and Pdes are both w.r.t the baselink coordinate in body frame!相对于基座坐标系
Pin_KinDyn::IkRes
Pin_KinDyn::computeInK_Hand(const Eigen::Matrix3d &Rdes_L, const Eigen::Vector3d &Pdes_L) {
    const pinocchio::SE3 oMdesL(Rdes_L, Pdes_L);
    std::vector<double> maxPos={2.97,2.09,2.97,2.09,2.97,2.09,3.05};
    Eigen::VectorXd qIk=Eigen::VectorXd::Zero(model_biped_fixed.nv); // initial guess
    // arm-l: 0-6, arm-r: 7-13, head: 14,15 waist: 16-18, leg-l: 19-24, leg-r: 25-30
    qIk.block<7,1>(0,0)<< 0.433153883479341,    -1.11739345867607,    1.88491913406236,
            0.802378252758275,    -0.356,    0.0,  -0.0;
    const double eps  = 1e-4;//收敛误差阈值，当误差小于该值时认为求解成功
    const int IT_MAX  = 100;
    const double DT   = 6e-1;//步长，用于更新关节位置。
    const double damp = 1e-2;//阻尼因子，用于避免雅可比矩阵求逆时出现奇异问题。
    Eigen::MatrixXd JL(6,model_biped_fixed.nv);
    JL.setZero();    

    bool success = false;
    Eigen::Matrix<double, 6, 1> errL;//包含 3 个线位置误差分量和 3 个姿态误差分量
    Eigen::VectorXd v(model_biped_fixed.nv);//用于表示关节速度。在逆运动学迭代求解过程中，通过计算得到关节速度，然后更新关节位置。

    pinocchio::JointIndex J_Idx_l;
    J_Idx_l = hand;
    
    int itr_count{0};
    for (itr_count=0;; itr_count++)
    {
        pinocchio::forwardKinematics(model_biped_fixed,data_biped_fixed,qIk);
        const pinocchio::SE3 iMdL = data_biped_fixed.oMi[J_Idx_l].actInv(oMdesL);//当前位姿到期望位姿的相对变换
        
        errL = pinocchio::log6(iMdL).toVector();  // in joint frame
       
        if(errL.norm() < eps)
        {
            success = true;
            break;
        }
        if (itr_count >= IT_MAX)
        {
            success = false;
            break;
        }

        pinocchio::computeJointJacobian(model_biped_fixed,data_biped_fixed,qIk,J_Idx_l,JL);  // 计算JL in joint frame
        pinocchio::Data::Matrix6 JlogL;       
        pinocchio::Jlog6(iMdL.inverse(), JlogL);       
        JL = -JlogL * JL;
        
        // pinocchio::Data::Matrix6 JJt;
        Eigen::Matrix<double,6,6> JJt;
        JJt.noalias() = JL * JL.transpose();
        JJt.diagonal().array() += damp;
        v.noalias() = - JL.transpose() * JJt.ldlt().solve(errL);
        qIk = pinocchio::integrate(model_biped_fixed,qIk,v*DT);
    }

    IkRes res;
    res.err=errL;
    res.itr=itr_count;

    if(success){
        res.status=0;
    }
    else{
        res.status=-1;
    }
    for (int i=0;i<7;i++){
        while((qIk(i)>maxPos[i])||(qIk(i)<-maxPos[i])){
            if(qIk(i)>maxPos[i]){
                qIk(i)=qIk(i)-2*3.1415926;
            }
            else if(qIk(i)<-maxPos[i]){
                qIk(i)=qIk(i)+2*3.1415926;
            }
        }
    }
    res.jointPosRes=qIk;
    return res;
}

// must call computeDyn() first!该函数要求在调用之前先调用 computeDyn() 函数，以确保相关动力学参数已经计算好。
void Pin_KinDyn::workspaceConstraint(Eigen::VectorXd &qFT, Eigen::VectorXd &tauJointFT) {
    for (int i=0; i<motorName.size();i++)
        if (qFT(i)>motorMaxPos(i)){
            qFT(i)=motorMaxPos(i);
            motorReachLimit[i]=true;
            tauJointFT(i)=tauJointOld(i);
        }
        else if (qFT(i)<motorMinPos(i)){
            qFT(i)=motorMinPos(i);
            motorReachLimit[i]=true;
            tauJointFT(i)=tauJointOld(i);
        }
        else
            motorReachLimit[i]=false;

    tauJointOld=tauJointFT;
}

























