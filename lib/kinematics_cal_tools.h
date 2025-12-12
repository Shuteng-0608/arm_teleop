#ifndef KINEMATICS_CAL_TOOLS_H
#define KINEMATICS_CAL_TOOLS_H

#include "kinematics_new.h"


extern const SerialJoints SerialZeros;

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

struct solWithPhiAndPlane {
    SerialJoints sol;
    double arm_angle;
    Vector3d plane;

    solWithPhiAndPlane() : 
        sol(SerialZeros),
        arm_angle(0),
        plane(0,0,0)
    {}
};

struct solWithPlane {
    SerialJoints sol;
    Vector3d plane;

    solWithPlane() : 
        sol(SerialZeros),
        plane(0,0,0)
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

std::optional<SerialJointsWithPhiAndPlane> 
    select_closest_ik_solution_with_phi_and_plane(
        const std::vector<SerialJointsWithPhiAndPlane>& 
                                    possible_serial_joints_vec_with_phi_and_plane,
        const SerialJoints& current_joints_tuple);

std::optional<SerialJointsWithIndex> 
    select_closest_ik_solution_with_index(
        const std::vector<SerialJoints>& 
                                    possible_serial_joints_vec,
        const SerialJoints& current_joints_tuple);

double normalize_angle_to_open_interval(double angle);
double normalize_angle_to_neg_pi_to_pi(double angle);
double normalize_angle(double angle);

void seperate_serial_joints_with_arm_angle(
    solWithPhi& result, // 通过引用传递，函数会直接修改它
    const std::optional<SerialJointsWithPhi>& closest_solution_with_phi);

void seperate_serial_joints_with_plane(
    solWithPlane& result, // 通过引用传递，函数会直接修改它
    const std::optional<SerialJointsWithPlane>& closest_solution_with_plane);

void seperate_serial_joints_with_arm_angle_and_plane(
    solWithPhiAndPlane& result, // 通过引用传递，函数会直接修改它
    const std::optional<SerialJointsWithPhiAndPlane>& closest_solution_with_phi_and_plane);

void separate_vec_serial_joints_with_plane(
    const std::vector<SerialJointsWithPlane>& input_vec,
    std::vector<SerialJoints>& joints_vec,
    std::vector<Vector3d>& planes_vec);

std::optional<SerialJoints> select_sol_from_possible(
    size_t sol_id,
    const std::vector<SerialJoints>& possible_serial_joints);

// 针对只有一个额外数据类型的特化（例如：double, int, Vector3d）
template <typename TupleWithData, typename DataType>
void separate_vector_single(const std::vector<TupleWithData>& input_vec,
                            std::vector<SerialJoints>& joints_vec,
                            std::vector<DataType>& data_vec) {
    joints_vec.clear();
    data_vec.clear();

    for (const auto& entry : input_vec) {
        joints_vec.emplace_back(std::get<0>(entry), std::get<1>(entry), std::get<2>(entry),
                                std::get<3>(entry), std::get<4>(entry), std::get<5>(entry),
                                std::get<6>(entry));
        data_vec.emplace_back(std::get<7>(entry));
    }
}



// 这是一个通用的模板函数，用于从任何元组中提取前7个double
template <typename... Args>
SerialJoints extract_serialjoints_from_tuple(const std::tuple<Args...>& input_tuple) {
    // 编译时检查，确保元组至少有7个元素
    static_assert(sizeof...(Args) >= 7, "Input tuple must have at least 7 elements.");

    // 直接使用 std::get 来提取前7个元素，并构造一个新的 SerialJoints 元组
    return std::make_tuple(
        std::get<0>(input_tuple),
        std::get<1>(input_tuple),
        std::get<2>(input_tuple),
        std::get<3>(input_tuple),
        std::get<4>(input_tuple),
        std::get<5>(input_tuple),
        std::get<6>(input_tuple)
    );
}

double calculate_max_joint_deviation(SerialJoints joints_sol, 
                                     const double current_joints_array[]);

// 检查一个 IKResult 是否有效且不发生关节跳变。
bool is_solution_acceptable(const IKResult& result, 
                            const double current_joints_array[], 
                            double offset_ref);


#endif