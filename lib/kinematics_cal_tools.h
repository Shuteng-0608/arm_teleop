#ifndef KINEMATICS_CAL_TOOLS_H
#define KINEMATICS_CAL_TOOLS_H

#include "kinematics_new.h"

// 该头文件应该仅被kinematics.cpp 和 kinematics_cal_tools.cpp 包含,不被外界所访问

// 从SerialJoints-> SerialJointsWithPhi
struct solWithPhi {
    SerialJoints sol;
    double arm_angle;

    solWithPhi() : 
        sol(SerialZeros),
        arm_angle(0)
    {}
};

// 改进DH模型变换矩阵生成
Matrix4d modified_DH_transform(
    double theta,   // 关节转角 (rad)
    double d,       // 连杆偏移 (m)
    double a,       // 连杆长度 (m)
    double alpha    // 连杆扭角 (rad)
);

// 向量转反对称矩阵
Matrix3d vec_to_skew_matrix(const Vector3d& v);

// 解验证与关节限位检查
std::tuple<bool, Vector7d, std::vector<int>> validate_solution(
    const Vector7d& angles,            // 输入关节角度
    const std::vector<std::pair<double, double>>& limits  // 关节限位
);

std::optional<SerialJoints> select_closest_ik_solution(
    const std::vector<SerialJoints>& possible_serial_joints_vec,
    const SerialJoints& current_joints_tuple);

std::optional<SerialJointsWithPhi> 
    select_closest_ik_solution_with_phi(
        const std::vector<SerialJointsWithPhi>& 
                            possible_serial_joints_vec_with_phi,
        const SerialJoints& current_joints_tuple);

double normalize_angle_to_open_interval(double angle);
double normalize_angle_to_neg_pi_to_pi(double angle);
double normalize_angle(double angle);

void seperate_serial_joints_with_arm_angle(
    solWithPhi& result, // 通过引用传递，函数会直接修改它
    const std::optional<SerialJointsWithPhi>& closest_solution_with_phi);

#endif