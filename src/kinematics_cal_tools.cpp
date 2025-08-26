#include "kinematics_cal_tools.h"


Eigen::Matrix4d modified_DH_transform(double theta, double d, double a, double alpha) {
    Eigen::Matrix4d T;
    T << std::cos(theta), -std::sin(theta), 0, a,
         std::sin(theta)*std::cos(alpha), std::cos(theta)*std::cos(alpha), -std::sin(alpha), -std::sin(alpha)*d,
         std::sin(theta)*std::sin(alpha), std::cos(theta)*std::sin(alpha),  std::cos(alpha),  std::cos(alpha)*d,
         0, 0, 0, 1;
    return T;
}

Matrix3d vec_to_skew_matrix(const Vector3d& v) {
    Matrix3d skew;
    skew << 0,   -v.z(),  v.y(),
            v.z(), 0,    -v.x(),
           -v.y(), v.x(), 0;
    return skew;
}

double normalize_angle(double angle) {
    // 将角度移到 [0, 2*pi) 范围
    angle = fmod(angle, 2 * M_PI);
    if (angle < 0) {
        angle += 2 * M_PI;
    }
    // 将角度移到 [-pi, pi) 范围
    if (angle >= M_PI) {
        angle -= 2 * M_PI;
    }
    return angle;
}

std::tuple<bool, Vector7d, std::vector<int>> validate_solution(
    const Vector7d& angles,


    const std::vector<std::pair<double, double>>& limits)
{
    Vector7d normalized = angles;
    std::vector<int> violations;

    for(int i=0; i<7; ++i) {
        normalized[i] = normalize_angle(angles[i]);
        if(normalized[i] < limits[i].first || normalized[i] > limits[i].second) {
            violations.push_back(i); //违规关节索引
        }
    }
    //返回的是一个元组
    return {violations.empty(), normalized, violations};
}

/**
 * @brief 从给定的可能解中选择距离当前关节位置最近的一组解。
 *
 * @param possible_serial_joints_vec 包含多组关节解的vector，每组是一个 SerialJoints 类型。
 * @param current_joints_tuple 当前的关节位置，一个 SerialJoints 类型。
 * @return 距离 current_joints_tuple 最近的关节解 (std::optional<SerialJoints> 类型)。
 * 如果 possible_serial_joints_vec 为空，则返回一个空的 std::optional 对象。
 */
std::optional<SerialJoints> select_closest_ik_solution(
    const std::vector<SerialJoints>& possible_serial_joints_vec,
    const SerialJoints& current_joints_tuple)
{
    // 如果没有可能的解，返回一个空的 std::optional
    if (possible_serial_joints_vec.empty()) {
        std::cerr << "警告: possible_serial_joints_vec 为空。返回一个空的 optional 对象。" << std::endl;
        return std::nullopt; // 返回一个表示“无值”的 std::optional 对象
    }

    double min_distance_sq = std::numeric_limits<double>::max(); // 初始化为最大可能值
    SerialJoints closest_solution; // 用于存储最近的解

    // 使用 std::apply 来计算距离，避免运行时索引
    auto calculate_distance_sq = [&](const SerialJoints& s1, const SerialJoints& s2) {
        double dist_sq = 0.0;
        std::apply([&](auto&&... args1) {
            std::apply([&](auto&&... args2) {
                // 使用折叠表达式计算平方和
                double temp_arr[] = { ( (args1 - args2) * (args1 - args2) )... };
                for (double val : temp_arr) {
                    dist_sq += val;
                }
            }, s2);
        }, s1);
        return dist_sq;
    };

    // 遍历所有可能的关节解
    for (const auto& solution_tuple : possible_serial_joints_vec) {
        // 计算当前解与当前关节位置之间的欧几里得距离平方
        double current_distance_sq = calculate_distance_sq(solution_tuple, current_joints_tuple);

        // 如果当前距离更小，则更新最小距离和最近的解
        if (current_distance_sq < min_distance_sq) {
            min_distance_sq = current_distance_sq;
            closest_solution = solution_tuple;
        }
    }

    // 找到了最近的解，将其包装在 std::optional 中返回
    return closest_solution;
}

/**
 * @brief 从给定的可能解中选择距离当前关节位置最近的一组解.
 * @param possible_serial_joints_vec 包含多组关节解的vector，每组是一个 SerialJoints 类型 加上臂角记录phi。
 * @param current_joints_tuple 当前的关节位置，一个 SerialJoints 类型。
 * @return 距离 current_joints_tuple 最近的关节解 (std::optional<SerialJoints> 类型， 加上臂角)。
 * 如果 possible_serial_joints_vec 为空，则返回一个空的 std::optional 对象。
 */
std::optional<SerialJointsWithPhi> 
    select_closest_ik_solution_with_phi(
        const std::vector<SerialJointsWithPhi>& 
                                    possible_serial_joints_vec_with_phi,
        const SerialJoints& current_joints_tuple)
{
    // 如果没有可能的解，返回一个空的 std::optional
    if (possible_serial_joints_vec_with_phi.empty()) {
        std::cerr << "警告: possible_serial_joints_vec_with_phi 为空。返回一个空的 optional 对象。" << std::endl;
        return std::nullopt; // 返回一个表示“无值”的 std::optional 对象
    }

    double min_distance_sq = std::numeric_limits<double>::max(); // 初始化为最大可能值
    double min_phi_abs = std::numeric_limits<double>::max();     // 初始化最小phi绝对值
    std::tuple<double, double, double, double, double, double, double, double> closest_solution_with_phi; // 用于存储最近的解

    // 使用 std::apply 来计算距离，避免运行时索引
    auto calculate_distance_sq = [&](const SerialJoints& s1, const SerialJoints& s2) {
        double dist_sq = 0.0;
        std::apply([&](auto&&... args1) {
            std::apply([&](auto&&... args2) {
                // 使用折叠表达式计算平方和
                double temp_arr[] = { ( (args1 - args2) * (args1 - args2) )... };
                for (double val : temp_arr) {
                    dist_sq += val;
                }
            }, s2);
        }, s1);
        return dist_sq;
    };

    // 遍历所有可能的关节解
    for (const auto& solution_tuple_with_phi : possible_serial_joints_vec_with_phi) {
        // 计算当前解与当前关节位置之间的欧几里得距离平方
        solWithPhi temp_sol;
        seperate_serial_joints_with_arm_angle(temp_sol,solution_tuple_with_phi);
        SerialJoints solution_tuple = temp_sol.sol;
        double current_phi = temp_sol.arm_angle;
        double current_distance_sq = calculate_distance_sq(solution_tuple, current_joints_tuple);

        if (fabs(current_distance_sq - min_distance_sq) <= EPSILON*1e5) {
            if (std::fabs(current_phi) < min_phi_abs) {
                min_phi_abs = std::fabs(current_phi); // 更新最小phi绝对值
                closest_solution_with_phi = solution_tuple_with_phi; // 更新为当前解
            }
        }

        // 如果当前距离更小，则更新最小距离和最近的解
        else if (current_distance_sq < min_distance_sq) {
            min_distance_sq = current_distance_sq;
            min_phi_abs = std::fabs(current_phi);
            closest_solution_with_phi = solution_tuple_with_phi;
        }

    }

    // 找到了最近的解，将其包装在 std::optional 中返回
    return closest_solution_with_phi;
}

// 定义一个辅助函数来处理 atan2 的结果
double normalize_angle_to_open_interval(double angle) {
    if (std::abs(angle - M_PI) < 1e-9) { // 如果角度非常接近 pi
        return -M_PI; // 映射到 -pi
    }
    return angle;
}

double normalize_angle_to_neg_pi_to_pi(double angle) {
    // 1. 将角度转换为 [0, 2*pi) 范围
    double normalized = std::fmod(angle, 2.0 * M_PI);
    if (normalized < 0) {
        normalized += 2.0 * M_PI;
    }

    // 2. 将 [0, 2*pi) 范围转换为 [-pi, pi)
    if (normalized >= M_PI) {
        normalized -= 2.0 * M_PI;
    }
    return normalized;
}

void seperate_serial_joints_with_arm_angle(
    solWithPhi& result, // 通过引用传递，函数会直接修改它
    const std::optional<SerialJointsWithPhi>& closest_solution_with_phi
) {
    // 检查 optional 是否包含值
    if (closest_solution_with_phi.has_value()) {
        const auto& solution_tuple_8d = closest_solution_with_phi.value();

        // 提取前7个值赋给 result.final_sol
        result.sol = std::make_tuple(
            std::get<0>(solution_tuple_8d),
            std::get<1>(solution_tuple_8d),
            std::get<2>(solution_tuple_8d),
            std::get<3>(solution_tuple_8d),
            std::get<4>(solution_tuple_8d),
            std::get<5>(solution_tuple_8d),
            std::get<6>(solution_tuple_8d)
        );
        // 提取第8个值赋给 result.arm_angle
        result.arm_angle = std::get<7>(solution_tuple_8d);

    } else {
        // 如果 optional 为空，可以设置一个默认或无效状态
        // 例如，将关节设为零，角度设为 NaN，并标记为无效
        result.sol = std::make_tuple(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        result.arm_angle = std::numeric_limits<double>::quiet_NaN(); // 需要 #include <limits>
        std::cerr << "Warning: closest_solution_with_phi does not contain a value. IKResultOffset not populated." << std::endl;
    }
}

