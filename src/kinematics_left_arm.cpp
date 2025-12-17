#include "kinematics_new.h"


// 常量实现
const Eigen::Matrix4d ArmLeftKine::M_mirror = 
(Eigen::Matrix4d() << 
    1.0, 0.0, 0.0, 0.0,    
    0.0, 1.0, 0.0, 0.0,    
    0.0, 0.0,-1.0, 0.0,    
    0.0, 0.0, 0.0, 1.0 
).finished();

const Eigen::Matrix4d ArmLeftKine::T_GR = 
(Eigen::Matrix4d() << 
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, ArmLeftKine::length_LR / 2.0, // 可以访问类内其他常量
    0.0, 0.0, 0.0, 1.0
).finished();

const Eigen::Matrix4d ArmLeftKine::T_RG = 
(Eigen::Matrix4d() << 
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, -ArmLeftKine::length_LR / 2.0,
    0.0, 0.0, 0.0, 1.0
).finished();

// const Vector7d ArmLeftKine::sign_vector = {
//     -1.0, 1.0, -1.0, 1.0, -1.0, -1.0, -1.0
// };
const Vector7d ArmLeftKine::sign_vector = (Vector7d() << 1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0).finished();

const double ArmLeftKine::length_LR = 0.5;

// 实现 ArmLeftKine 的构造函数
ArmLeftKine::ArmLeftKine(std::shared_ptr<ArmKineStd> std_solver,
                         std::shared_ptr<ArmKineOfst> ofst_solver)
    // 成员初始化列表：
    // 使用 std::move 将传入的指针移动给 right_arm 成员
    : right_arm(std_solver, ofst_solver) 
{
    // 构造函数体为空，或者执行任何额外的左臂初始化工作
}



IKResult ArmLeftKine::cal_left_arm_IK(
            const Matrix4d& target_pose,  // 左臂末端在全局坐标系下的位姿
            double current_joints_array[], 
            double arm_angle){
    
    Matrix4d TeeL_G = target_pose; // 左臂末端在全局坐标系下的位姿
    Matrix4d TeeRMir_R = T_RG * M_mirror * TeeL_G * M_mirror; // 左臂末端镜像到右边之后在右臂坐标系下的表示，因此可以直接用IK

    IKResult result_R = right_arm.calculateIK(
        TeeRMir_R, 
        current_joints_array, 
        arm_angle // 传入arm_angle
        // theta7 传入 std::nullopt，因为 ArmKineOfst 默认是空
    );
    return result_R;
}

FKResult ArmLeftKine::calculateFK(const Vector7d& theta){

    FKResult res_L;
    Vector7d q_L = theta;
    Vector7d q_R = sign_vector.cwiseProduct(q_L); //q_L 经过sign_vector变换后要直接能计算出等效的Arm_right才可以
    FKResult res_R = right_arm.calculateFK(q_R);
    res_L = res_R;
    Matrix4d TeeL_G;
    TeeL_G = M_mirror * (T_GR * res_R.T_08) * M_mirror;
    res_L.T_08 = TeeL_G;

    return res_L;
}

IKResult ArmLeftKine::cal_left_arm_feasible_IK_vec_ref(
            const Matrix4d& target_pose,  // 左臂末端在全局坐标系下的位姿
            double current_joints_array[], 
            double arm_angle,
            const std::vector<double>& arm_angle_deviation_list,
            double offset_ref){
    
    Matrix4d TeeL_G = target_pose; // 左臂末端在全局坐标系下的位姿
    Matrix4d TeeRMir_R = T_RG * M_mirror * TeeL_G * M_mirror; // 左臂末端镜像到右边之后在右臂坐标系下的表示，因此可以直接用IK

    // IKResult result_R = right_arm.calculateIK(
    //     TeeRMir_R, 
    //     current_joints_array, 
    //     arm_angle // 传入arm_angle
    //     // theta7 传入 std::nullopt，因为 ArmKineOfst 默认是空
    // );

    IKResult result_R = right_arm.cal_IK_feasible_armAngle_vec_ref(
        TeeRMir_R,
        current_joints_array,
        -arm_angle, // current_arm_angle,
        arm_angle_deviation_list, //const std::vector<double>& arm_angle_deviation_list, // 默认臂角偏差列表
        offset_ref //double offset_ref  // 默认最大关节跳变阈值
    );




    return result_R;
}