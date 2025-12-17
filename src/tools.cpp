#include "tools.h"

PoseComparisonResult compare_poses_detailed(
    const Matrix4d& ref_pose,
    const Matrix4d& test_pose,
    double trans_tol,
    double rot_tol) 
{
    PoseComparisonResult result;
    result.is_approximate = true;
    result.translation_error = 0.005;
    result.rotation_error = 0.01;
    result.error_type = "";

    // 计算平移误差
    Vector3d ref_trans = ref_pose.block<3,1>(0,3);
    Vector3d test_trans = test_pose.block<3,1>(0,3);
    result.translation_error = (ref_trans - test_trans).norm();
    std::cout << " result.translation_error :" << result.translation_error << std::endl;

    // 检查平移误差
    if (result.translation_error > trans_tol) {
        result.is_approximate = false;
        std::ostringstream oss;
        oss << "平移误差超标 (" << std::fixed << result.translation_error 
            << " > " << trans_tol << ")";
        result.error_type = oss.str();
    }

    // 计算旋转误差
    Matrix3d R_ref = ref_pose.block<3,3>(0,0);
    Matrix3d R_test = test_pose.block<3,3>(0,0);
    Matrix3d R_rel = R_ref.transpose() * R_test;

    // 计算旋转角度
    double trace = R_rel.trace();
    trace = std::clamp(trace, -1.0, 3.0); // Eigen 3.4+ 支持clamp
    double cos_theta = (trace - 1.0) / 2.0;
    cos_theta = std::clamp(cos_theta, -1.0, 1.0);
    result.rotation_error = std::acos(cos_theta);

    // 检查旋转误差
    std::cout << "result.rotation_error :" << result.rotation_error 
        << ", rot_tol :" << rot_tol << std::endl;
    if (result.rotation_error > rot_tol) {
        result.is_approximate = false;
        std::ostringstream oss;
        oss << "旋转误差超标 (" << std::fixed << result.rotation_error 
            << " > " << rot_tol << ")";
        
        // 合并错误信息
        if (!result.error_type.empty()) {
            result.error_type += "；";
        }
        result.error_type += oss.str();
    }

    return result;
}

void compare_vector7d(const Vector7d& vec1,
                    const Vector7d& vec2,
                    double tolerance = 1e-5) 
{
    // 设置输出格式
    Eigen::IOFormat fmt(5, 0, ", ", " | ", "", "", "[", "]");
    std::cout << std::fixed << std::setprecision(5);

    // 打印向量内容
    std::cout << "向量1: " << vec1.transpose().format(fmt) << "\n";
    std::cout << "向量2: " << vec2.transpose().format(fmt) << "\n";

    // 比较每个元素
    bool has_diff = false;
    for (int i = 0; i < 7; ++i) {
        const double diff = std::abs(vec1[i] - vec2[i]);
        if (diff > tolerance) {
            if (!has_diff) {
                std::cout << "发现差异 (容忍度: " << tolerance << "):\n";
                has_diff = true;
            }
            std::cout << "  元素[" << i+1 << "]: " 
                      << vec1[i] << " vs " << vec2[i] 
                      << " (Δ=" << diff << ")\n";
        }
    }

    if (!has_diff) {
        std::cout << "向量一致\n";
    }
}

void convert_Hmatrix_to_PQarray(const Matrix4d& matrix, double output[7]) {
    // 提取平移部分（访问第四列的前三个元素）
    output[0] = matrix(0, 3); // X
    output[1] = matrix(1, 3); // Y
    output[2] = matrix(2, 3); // Z

    // 提取旋转矩阵元素（使用Eigen的括号运算符）
    const double& m00 = matrix(0, 0);
    const double& m01 = matrix(0, 1);
    const double& m02 = matrix(0, 2);
    const double& m10 = matrix(1, 0);
    const double& m11 = matrix(1, 1);
    const double& m12 = matrix(1, 2);
    const double& m20 = matrix(2, 0);
    const double& m21 = matrix(2, 1);
    const double& m22 = matrix(2, 2);

    // 保持原有的四元数转换逻辑
    double trace = m00 + m11 + m22;
    double qx, qy, qz, qw;

    if (trace > 0) {
        double s = 0.5 / std::sqrt(trace + 1.0);
        qw = 0.25 / s;
        qx = (m21 - m12) * s;
        qy = (m02 - m20) * s;
        qz = (m10 - m01) * s;
    } else {
        if (m00 > m11 && m00 > m22) {
            double s = 2.0 * std::sqrt(1.0 + m00 - m11 - m22);
            qw = (m21 - m12) / s;
            qx = 0.25 * s;
            qy = (m01 + m10) / s;
            qz = (m02 + m20) / s;
        } else if (m11 > m22) {
            double s = 2.0 * std::sqrt(1.0 + m11 - m00 - m22);
            qw = (m02 - m20) / s;
            qx = (m01 + m10) / s;
            qy = 0.25 * s;
            qz = (m12 + m21) / s;
        } else {
            double s = 2.0 * std::sqrt(1.0 + m22 - m00 - m11);
            qw = (m10 - m01) / s;
            qx = (m02 + m20) / s;
            qy = (m12 + m21) / s;
            qz = 0.25 * s;
        }
    }

    // 保持四元数顺序（x, y, z, w）
    output[3] = qx;
    output[4] = qy;
    output[5] = qz;
    output[6] = qw;
}


void print_pose_quaternion(const double* pq, 
                          std::ostream& os,
                          int precision) {
    // 保存原始格式状态
    std::ios oldState(nullptr);
    oldState.copyfmt(os);

    // 设置固定小数格式和精度
    os << std::fixed << std::setprecision(precision);
    
    // 打印位置部分
    os << "Position (m): [ "
       << pq[0] << ", "
       << pq[1] << ", "
       << pq[2] << " ]\n";

    // 打印四元数部分
    os << "Quaternion (x,y,z,w): [ "
       << pq[3] << ", "
       << pq[4] << ", "
       << pq[5] << ", "
       << pq[6] << " ]\n";

    // 验证四元数单位化
    const double norm = std::sqrt(
        pq[3]*pq[3] + 
        pq[4]*pq[4] + 
        pq[5]*pq[5] + 
        pq[6]*pq[6]
    );
    
    os << "Quaternion norm: " << norm 
       << " (" << (std::abs(norm - 1.0) < 0.001 ? "OK" : "WARNING: non-unit quaternion") 
       << ")\n";

    // 恢复原始格式
    os.copyfmt(oldState);
}

void print_serial_joints(const SerialJoints& joints) {
    std::cout << "关节值: ("
              << std::get<0>(joints) << ", "
              << std::get<1>(joints) << ", "
              << std::get<2>(joints) << ", "
              << std::get<3>(joints) << ", "
              << std::get<4>(joints) << ", "
              << std::get<5>(joints) << ", "
              << std::get<6>(joints) << ")" << std::endl;
}

Vector7d serial_joints_to_vec7d(const SerialJoints& joints_tuple) {
    Vector7d vec;
    // 逐个元素赋值
    vec[0] = std::get<0>(joints_tuple);
    vec[1] = std::get<1>(joints_tuple);
    vec[2] = std::get<2>(joints_tuple);
    vec[3] = std::get<3>(joints_tuple);
    vec[4] = std::get<4>(joints_tuple);
    vec[5] = std::get<5>(joints_tuple);
    vec[6] = std::get<6>(joints_tuple);
    return vec;
}

SerialJoints vec7d_to_serial_joints(const Vector7d& joints_vec) {
    SerialJoints joints_tuple;
    // 逐个元素赋值
    std::get<0>(joints_tuple) = joints_vec[0];
    std::get<1>(joints_tuple) = joints_vec[1];
    std::get<2>(joints_tuple) = joints_vec[2];
    std::get<3>(joints_tuple) = joints_vec[3];
    std::get<4>(joints_tuple) = joints_vec[4];
    std::get<5>(joints_tuple) = joints_vec[5];
    std::get<6>(joints_tuple) = joints_vec[6];
    return joints_tuple;
}    

void serial_joints_to_double_array(const SerialJoints& joints_tuple, double* out_ptr) {
    // 检查 out_ptr 是否为空指针，虽然这并不能检查内存是否足够大，
    // 但至少可以避免空指针解引用。
    if (out_ptr == nullptr) {
        std::cerr << "错误：传入的 out_ptr 为空指针，无法写入数据。\n";
        return;
    }

    // 依次将元组元素赋值到指针指向的内存位置
    // out_ptr[i] 是指针解引用和偏移量的语法糖，等同于 *(out_ptr + i)
    out_ptr[0] = std::get<0>(joints_tuple);
    out_ptr[1] = std::get<1>(joints_tuple);
    out_ptr[2] = std::get<2>(joints_tuple);
    out_ptr[3] = std::get<3>(joints_tuple);
    out_ptr[4] = std::get<4>(joints_tuple);
    out_ptr[5] = std::get<5>(joints_tuple);
    out_ptr[6] = std::get<6>(joints_tuple);
}

void double_array_to_serial_joints(const double* current_joints_array, SerialJoints& current_joints_tuple) {
    // **重要提示：**
    // 在这里，我们无法在运行时可靠地检查 current_joints_array 的实际大小。
    // 如果调用者传入的数组小于 7 个元素，下面的访问将导致越界，引发未定义行为。
    // 如果你坚持不传入大小且不使用模板，那么这种风险是无法避免的。

    std::get<0>(current_joints_tuple) = current_joints_array[0];
    std::get<1>(current_joints_tuple) = current_joints_array[1];
    std::get<2>(current_joints_tuple) = current_joints_array[2];
    std::get<3>(current_joints_tuple) = current_joints_array[3];
    std::get<4>(current_joints_tuple) = current_joints_array[4];
    std::get<5>(current_joints_tuple) = current_joints_array[5];
    std::get<6>(current_joints_tuple) = current_joints_array[6];
}

//计算两个向量夹角,传入向量无需单位化
double cal_vec_angle(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2){
    // Check if either vector is a zero vector to avoid division by zero.
    double norm1 = vec1.norm();
    double norm2 = vec2.norm();

    if (norm1 < std::numeric_limits<double>::epsilon() || norm2 < std::numeric_limits<double>::epsilon()) {
        std::cerr << "Warning: One or both vectors are zero vectors. Angle is undefined." << std::endl;
        return 0.0; // Return 0 or handle as an error.
    }

    // Calculate the dot product.
    double dot_product = vec1.dot(vec2);

    // Calculate the cosine of the angle.
    double cos_theta = dot_product / (norm1 * norm2);

    // Clamp the value to the valid range [-1, 1] to prevent a domain error
    // with std::acos due to floating-point inaccuracies.
    if (cos_theta > 1.0) {
        cos_theta = 1.0;
    } else if (cos_theta < -1.0) {
        cos_theta = -1.0;
    }

    // Calculate the angle in radians.
    double angle_rad = std::acos(cos_theta);

    // Convert the angle to degrees.
    // double angle_deg = angle_rad * 180.0 / M_PI;

    return angle_rad;
} 



/**
 * @brief 计算 n1 到 n2 绕轴 axis (l1) 的有向夹角 (弧度)。
 * * 步骤包括：轴归一化，向量投影，投影向量归一化，atan2计算。
 * 鲁棒性：检查输入零向量和输入向量与轴平行的情况。
 * * @param n1 起始向量 (Vector3d)
 * @param n2 终点向量 (Vector3d)
 * @param l1 旋转轴 (Vector3d)
 * @return double 角度 (弧度), 范围 (-PI, PI]
 * @throws std::runtime_error 如果输入向量或轴为零向量，或 n1/n2 平行于 l1。
 */
double cal_signed_angle(const Vector3d& n1, const Vector3d& n2, const Vector3d& l1) {
    // 定义一个小的阈值 epsilon 用于浮点数比较
    const double EPSILON = 1e-9;
    
    // --- 鲁棒性检查 1: 零向量输入 ---
    if (n1.norm() < EPSILON) {
        throw std::runtime_error("Error: Starting vector n1 is a zero vector.");
    }
    if (n2.norm() < EPSILON) {
        throw std::runtime_error("Error: Ending vector n2 is a zero vector.");
    }
    if (l1.norm() < EPSILON) {
        throw std::runtime_error("Error: Rotation axis l1 is a zero vector.");
    }

    // 1. 轴归一化 (Normalize Axis)
    Vector3d axis_norm = l1.normalized();

    // 2. 计算垂直分量 (投影)
    // 从 n1/n2 中减去它们在轴上的分量，得到垂直于 l1 的投影向量 (p1, p2)。
    // 公式: p = v - (v . axis) * axis
    
    // n1 投影
    double n1_dot_axis = n1.dot(axis_norm);
    Vector3d p1 = n1 - n1_dot_axis * axis_norm; // 垂直于l1的分量
    
    // n2 投影
    double n2_dot_axis = n2.dot(axis_norm);
    Vector3d p2 = n2 - n2_dot_axis * axis_norm; // 垂直于l1的分量

    // --- 鲁棒性检查 2: 向量平行于轴 ---
    // 如果投影后的向量模长接近 0，说明原始向量几乎平行于旋转轴。
    // 在这种情况下，旋转角度在几何上是未定义的。
    if (p1.norm() < EPSILON) {
        throw std::runtime_error("Error: Vector n1 is parallel (or anti-parallel) to the axis l1.");
    }
    if (p2.norm() < EPSILON) {
        throw std::runtime_error("Error: Vector n2 is parallel (or anti-parallel) to the axis l1.");
    }

    // 3. 归一化投影向量
    // 确保 atan2 的输入 x 和 y 都是单位向量的点积和叉积，以提高数值稳定性。
    Vector3d u_p1 = p1.normalized();
    Vector3d u_p2 = p2.normalized();

    // 4. 计算 atan2 的分量
    // x (Cosine part): 使用归一化投影向量的点积 -> cos(theta)
    double x = u_p1.dot(u_p2);

    // y (Sine part): 使用归一化投影向量的叉积与轴的点积 -> sin(theta)
    // (u_p1 x u_p2) 结果是一个垂直于旋转平面的向量，点乘 axis_norm 提取出带符号的 sin(theta)。
    double y = u_p1.cross(u_p2).dot(axis_norm);

    // 5. 返回带符号角度
    return std::atan2(y, x);
}