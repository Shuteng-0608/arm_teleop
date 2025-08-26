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