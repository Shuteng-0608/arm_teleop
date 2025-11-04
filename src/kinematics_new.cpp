#include "kinematics_new.h"
#include "tools.h"
#include "kinematics_cal_tools.h"
#include <cmath>
#include <iostream>



const SerialJoints SerialZeros = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};



// KinematicsConfig 构造函数的实现
KineConfig::KineConfig(const std::string& filepath) {
    try {
        YAML::Node config = YAML::LoadFile(filepath);

        // DH Parameters
        d_bs = config["dh_parameters"]["d_bs"].as<double>();
        d_se = config["dh_parameters"]["d_se"].as<double>();
        d_ew = config["dh_parameters"]["d_ew"].as<double>();
        a_wf = config["dh_parameters"]["a_wf"].as<double>();
        a_se = config["dh_parameters"]["a_se"].as<double>();
        a_ee = config["dh_parameters"]["a_ee"].as<double>();

        // Conversion Drive Parameters
        l_bx = config["Conversion_drive_params"]["l_bx"].as<double>();
        l_by = config["Conversion_drive_params"]["l_by"].as<double>();
        l_ofsx = config["Conversion_drive_params"]["l_ofsx"].as<double>();
        l_ofsy = config["Conversion_drive_params"]["l_ofsy"].as<double>();
        double beta40_degrees = config["Conversion_drive_params"]["beta40_degrees"].as<double>();
        beta40_radians = beta40_degrees / 180.0 * M_PI; // 转换为弧度

        // lb = std::sqrt(l_bx*l_bx + l_by*l_by); // 在代码中计算
        // lofs = std::sqrt(l_ofsx*l_ofsx + l_ofsy*l_ofsy); // 在代码中计算

        // Rod Init Lengths
        l_m10 = config["motor_init_lengths"]["l_m10"].as<double>();
        l_m20 = config["motor_init_lengths"]["l_m20"].as<double>();
        l_m30 = config["motor_init_lengths"]["l_m30"].as<double>();

        // Wrist Joint Parameters
        d_cx = config["wrist_joint_params"]["d_cx"].as<double>();
        d_cy = config["wrist_joint_params"]["d_cy"].as<double>();
        d_cz = config["wrist_joint_params"]["d_cz"].as<double>();
        d_ax = config["wrist_joint_params"]["d_ax"].as<double>();
        d_ay = config["wrist_joint_params"]["d_ay"].as<double>();
        d_az = config["wrist_joint_params"]["d_az"].as<double>();


        // Joint Limits
        if (config["joint_limits"]) {
            for (const auto& limit_node : config["joint_limits"]) {
                double min_val = limit_node["min"].as<double>();
                double max_val = limit_node["max"].as<double>();
                joint_limits.push_back({min_val, max_val});
            }
        }
        std::cout << "Configuration loaded successfully from: " << filepath << std::endl;

    } catch (const YAML::BadFile& e) {
        std::cerr << "Error: Could not open config file: " << filepath << ". " << e.what() << std::endl;
        // 可以在这里抛出异常或设置默认值
        throw std::runtime_error("Failed to open kinematics config file.");
    } catch (const YAML::Exception& e) {
        std::cerr << "Error parsing config file: " << filepath << ". " << e.what() << std::endl;
        throw std::runtime_error("Failed to parse kinematics config file.");
    }
}







// 基类函数 构造函数
ArmKineBase::ArmKineBase(const std::string& config_filepath)
    : params_(config_filepath) {}// 在初始化列表中调用 KinematicsConfig 的构造函数

FKResult ArmKineBase::calculateFK(const Vector7d& theta) {
    FKResult result;

    // 获取差异参数，通过调用受保护的虚函数
    const double d_ew_val = get_A5_d_param(); // 调用派生类实现的函数
    const double a_wf_val = get_A7_a_param(); // 调用派生类实现的函数

    // 关节角度直接映射 (无拷贝)
    const double& t1 = theta[0];
    const double& t2 = theta[1];
    const double& t3 = theta[2];
    const double& t4 = theta[3];
    const double& t5 = theta[4];
    const double& t6 = theta[5];
    const double& t7 = theta[6];

    // 链式变换矩阵计算 (现在通用了)
    const Matrix4d A1 = modified_DH_transform(t1 + M_PI_2, params_.d_bs, 0.0, 0.0);
    const Matrix4d A2 = modified_DH_transform(t2 - M_PI_2, 0.0, 0.0, M_PI_2);
    const Matrix4d A3 = modified_DH_transform(t3 + M_PI_2, params_.d_se, 0.0, M_PI_2);
    const Matrix4d A4 = modified_DH_transform(t4, 0.0, params_.a_se, -M_PI_2);
    const Matrix4d A5 = modified_DH_transform(t5, d_ew_val, 0.0, M_PI_2); // 使用差异参数
    const Matrix4d A6 = modified_DH_transform(t6 + M_PI_2, 0.0, 0.0, M_PI_2);
    const Matrix4d A7 = modified_DH_transform(t7, 0.0, a_wf_val, M_PI_2); // 使用差异参数
    const Matrix4d A8 = modified_DH_transform(0.0, 0.0, params_.a_ee, 0.0);

    result.T_08 = A1 * A2 * A3 * A4 * A5 * A6 * A7 * A8;
    return result;
}


// 派生类函数 构造函数
ArmKineStd::ArmKineStd(const std::string& config_filepath)
    : ArmKineBase(config_filepath) {} // 需要传回基类，因为config 函数存在基类中


ArmKineOfst::ArmKineOfst(const std::string& config_filepath)
    : ArmKineBase(config_filepath) {} // 需要传回基类，因为config 函数存在基类中






/*
这里对解分支进行说明
    这里的写入顺序
    1. cos_theta_orig >0, cos_theta2 >0, cos_theta6 >0
    2. cos_theta_orig >0, cos_theta2 >0, cos_theta6 <0
    3. cos_theta_orig >0, cos_theta2 <0, cos_theta6 >0
    4. cos_theta_orig >0, cos_theta2 <0, cos_theta6 <0
    5. cos_theta_orig <0, cos_theta2 >0, cos_theta6 >0
    6. cos_theta_orig <0, cos_theta2 >0, cos_theta6 <0
    7. cos_theta_orig <0, cos_theta2 <0, cos_theta6 >0
    8. cos_theta_orig <0, cos_theta2 <0, cos_theta6 <0

这里的错误码展开说明
    error_code =-1 : 结构体初始化的默认值
    error_code = 1 : 求解初始参考平面时候错误， <<< sin^2+cos^2 != 1 >>>
    error_code = 2 : 求解初始参考平面时候错误，可能是奇异 <<< 行列式 D 接近零。 >>>
    error_code = 3 : 臂角无法计算，超出范围
    error_code = 4 : final sin(theta2) 值超出范围。跳过此解
    error_code = 5 : Delta < 0, 无法求解臂角
    error_code = 6 : theta2选择为theta2 = +- pi/2, 发生了奇异
    error_code = 7 : sin_theta6 值超出范围
    error_code = 8 : theta6 = +- pi/2 , 发生奇异！！！
    error_code = 9 : IK 计算未找到任何满足条件的解
    error_code = 10 : 超出了工作空间
    error_code = 11 : 无法计算sin_theta2_orig
    error_code = 12 : sin_theta2_orig 超出范围

*/

// 7DOF SRS Std IK 主函数
IKResult ArmKineStd::calculateIK(
    const Matrix4d& target_pose,
    double current_joints_array[],
    std::optional<double> arm_angle,
    std::optional<double> theta7
){

   IKResult result;
   result.is_valid = false;
    if (!arm_angle.has_value()) // 检查是否没有值 (即为 std::nullopt)
    {
        std::cerr << "函数输入错误， 未输入臂角 arm_angle " << std::endl;
        return result;
    }
    else // 有值
    {
        result.arm_angle = arm_angle.value(); // 使用 .value() 来获取内部的 double 值
    }

   

    // 打印所需参数定义
    Eigen::IOFormat fmt_p(Eigen::StreamPrecision, 0, ", ", "\n", "[", "]");
    Eigen::IOFormat fmt_t(4, 0, ", ", "\n", "|", "|");

    //======打印按钮=========//
    constexpr bool ENABLE_LOG = false; 

    // Step 1: 计算机械臂末端位姿,从末端法兰，到第七关节
    Matrix3d R_08 = target_pose.block<3,3>(0,0);

    // TODO 这个地方对应的偏置应该是d_ew_new = d_ew + a_wf
    Vector4d P_8_wa(-params_.a_ee, 0, 0, 1);
    // P_0_wa是腕关节在0坐标系下的位置, 这个腕关节相当于以带偏置的构型的腕关节的实际位置
    Vector4d P_0_wa = target_pose * P_8_wa;
    Matrix4d T_07 = Matrix4d::Identity();
    T_07.block<3,3>(0,0) = R_08;
    T_07.block<3,1>(0,3) = P_0_wa.head<3>();

    // TODO 为了匹配实际构型，修改d_ew_new, 以关节7 作为腕关节
    const double d_ew_new = params_.d_ew +params_.a_wf;

    // Step 2: 计算theta4， 只考虑theta4_up
    const double len_vec_SE = sqrt(pow(params_.d_se,2) + pow(params_.a_se,2));
    const double angle_ESEv = atan2(params_.a_se, params_.d_se);
    Vector4d P_0_S(0, 0, params_.d_bs, 1);
    //vec_0_sw 是0坐标系下的肩腕向量
    Vector3d vec_0_sw = T_07.block<3,1>(0,3) - P_0_S.head<3>();
    const double len_vec_0_sw = vec_0_sw.norm();
    const double angle_SEW = acos(
        std::clamp((pow(len_vec_SE,2) + pow(d_ew_new,2) - pow(len_vec_0_sw,2)) / 
             (2*len_vec_SE*d_ew_new), -1.0, 1.0)
    );
    const double angle_ESW = std::acos(
        std::clamp(
        (std::pow(len_vec_SE, 2) + std::pow(len_vec_0_sw, 2) - std::pow(len_vec_0_sw, 2)) / 
        (2 * len_vec_SE * len_vec_0_sw),-1.0, 1.0)
        );
    const double angle_EvSW = angle_ESW + angle_ESEv;
    // Elbow up
    double theta4 = angle_ESEv + pi - angle_SEW;

    //判断臂长
    const double max_arm_lengeth = len_vec_SE + d_ew_new; //虚拟机械臂的最大臂长，常数不会变
    double current_arm_length = len_vec_0_sw; //虚拟机械臂的肩腕关节向量的模长
    if (current_arm_length>=max_arm_lengeth ){
        std::cout << " >>>>>  IK Std 超出了工作空间 <<<<<" << std::endl;
        result.error_code = 10;
        return result;
    }

    // step 3 :计算参考平面下的前三个关节的初始值（参考平面为theta3 = 0 的手臂平面，参考08 Tro）

    // step 3.1 : 计算theta2_orig
    //提取肩腕关节向量元素进行运算
    double c1 = vec_0_sw(0);
    double c2 = vec_0_sw(1);
    double c3 = vec_0_sw(2);
    double denominator = params_.d_se + d_ew_new * cos(theta4);

    // 避免除以零或非常小的数
    if (std::abs(denominator) < std::numeric_limits<double>::epsilon()) {
        std::cerr << "IK Std 错误：分母接近零，无法计算 sin_theta2。" << std::endl;
        result.error_code = 11;
    }
    double sin_theta2_orig = -c3 / denominator;

    // --- 验证 sin_theta2_orig 的范围 ---
    // 由于浮点数精度问题，允许一点误差
    if (sin_theta2_orig > 1.0 + std::numeric_limits<double>::epsilon() ||
        sin_theta2_orig < -1.0 - std::numeric_limits<double>::epsilon()) {
        std::cerr << "IK Std 错误：sin_theta2_orig 的值超出 [-1, 1] 范围。无实数解 theta2。" << std::endl;
        result.error_code = 12;
    }

    // TODO 这里做了范围约束
    // 限制在 [-1, 1] 范围内，以防微小的浮点误差导致 sqrt(负数)
    if (sin_theta2_orig > 1.0) sin_theta2_orig = 1.0;
    if (sin_theta2_orig < -1.0) sin_theta2_orig = -1.0;

    // --- 计算 cos_theta2_orig 的两种可能解 ---
    //TODO >>>>> 解分支1 <<<<<
    double cos_theta2__orig_base = sqrt(1.0 - sin_theta2_orig * sin_theta2_orig);

    // --- 计算 theta2_orig 的两种可能解 ---
    // -- 两者的关系： theta2_1_orig + theta2_2_orig =  +- pi (两者符号相同)
    double theta2_1_orig = std::atan2(sin_theta2_orig, cos_theta2__orig_base);
    double theta2_2_orig = std::atan2(sin_theta2_orig,-cos_theta2__orig_base);

    std::vector<std::tuple<double>> possible_theta2_orig_values;
    possible_theta2_orig_values.push_back({theta2_1_orig});
    // 只有当 cos_theta2_orig_base 不接近零时，才有第二个独立的解
    if (cos_theta2__orig_base > EPSILON) { 
        possible_theta2_orig_values.push_back({theta2_2_orig});
    }
    else { //这个是最外层的循环
        std::cout << "只记录了1个theta2_orig,因此少一次方" << std::endl;
    }

    std::vector<SerialJoints> possible_serial_joints;
    // step 3.2 :遍历 theta2_orig 的每种情况， 求theta1_orig，并检查是否合理
    std::vector<std::tuple<double,double>> possible_theta_12_orig; // 存储theta1_orig, theta2_orig
    for (auto current_theta2_orig_temp : possible_theta2_orig_values) { //一次循环

        // --- Part 2: 求解 theta1_orig (基于线性方程组) ---
        double current_theta2_orig = std::get<0>(current_theta2_orig_temp);
        double cos_theta2_val_orig = std::cos(current_theta2_orig); // 不同的theta2_orig_case 结果相差一个负号
        double sin_theta4_val = std::sin(theta4);
        double cos_theta4_val = std::cos(theta4);
        
        double a1 = cos_theta2_val_orig * (params_.d_se+d_ew_new*cos_theta4_val);
        double b1 = params_.a_se + d_ew_new * sin_theta4_val;
        double a2 = params_.a_se + d_ew_new * sin_theta4_val;
        double b2 = -cos_theta2_val_orig * (params_.d_se+d_ew_new*cos_theta4_val);
 
        double D = a1 * b2 - a2 * b1;

        if (ENABLE_LOG == true){
            print_vector(vec_0_sw);
            std::cout << " a1: " << a1 << std::endl;
            std::cout << " b1: " << b1 << std::endl;
            std::cout << " a2: " << a2 << std::endl;
            std::cout << " b2: " << b2 << std::endl;
            std::cout << " D: " << D << std::endl;
        }


        double current_theta1_orig = std::nan(""); // 初始化为 NaN

        if (std::abs(D) > EPSILON) { // D 不为零，有唯一解  
            double x_sin_theta1_sol = (c1 * b2 - c2 * b1) / D;
            double y_cos_theta1_sol = (a1 * c2 - a2 * c1) / D;
            // 验证 x_sin_theta1_sol^2 + y_cos_theta1_sol^2 = 1,满足此条件，此解才有效
            double check_sum_sq = x_sin_theta1_sol * x_sin_theta1_sol + y_cos_theta1_sol * y_cos_theta1_sol;
            if (std::abs(check_sum_sq - 1.0) < EPSILON * 10.0) {
                current_theta1_orig = std::atan2(x_sin_theta1_sol, y_cos_theta1_sol);
                possible_theta_12_orig.push_back({current_theta1_orig,current_theta2_orig});
            } else {
                std::cout << "IK Std 警告 : 对于 theta2_orig=" << current_theta2_orig * 180.0 / M_PI
                          << "度，sin^2+cos^2 != 1。跳过此解。" << std::endl;
                result.error_code = 1;
                return result;
            }
        } else {
            std::cout << "IK Std 警告: 对于 theta2_orig=" << current_theta2_orig * 180.0 / M_PI
                      << "度，行列式 D 接近零。跳过此解。" << std::endl;
            result.error_code = 2;
            return result;
        }
    } // 一次循环结束

    // 在上面的计算中，得到了至多两组 theta1_orig, theta2_orig, theta3_orig=0,然后循环求解 
    // TODO 分支点1， theta2_orig 多解带来的分支 （可能可以消除掉）
    // step 4 : 基于theta1_orig  theta2_orig 和 original theta3 = 0，计算前三个过节的实际值，计算经过臂角作用后的R_03

    std::vector<std::tuple<double, double, double>> possible_theta_123;

    for (auto current_theta_12_orig : possible_theta_12_orig){ // 一次循环

        //计算变换矩阵
        const Matrix4d A1_orig_transform= modified_DH_transform(std::get<0>(current_theta_12_orig) + M_PI / 2.0, params_.d_bs, 0, 0);
        const Matrix4d A2_orig_transform= modified_DH_transform(std::get<1>(current_theta_12_orig) - M_PI / 2.0, 0, 0, M_PI / 2.0);
        const Matrix4d A3_orig_transform= modified_DH_transform(0.0 + M_PI / 2.0, params_.d_se, 0, M_PI / 2.0);
        const Matrix4d A4_orig_transform = modified_DH_transform(theta4, 0, params_.a_se, -M_PI / 2.0);

        Matrix4d T_03_orig_calc = A1_orig_transform * A2_orig_transform * A3_orig_transform;
        Matrix3d R_03_orig_calc = T_03_orig_calc.block<3, 3>(0, 0);

        // step 4.1 :计算真实 arm_angle 上的旋转矩阵
        const Vector3d vec_0_sw_hat = vec_0_sw.normalized();
        const Matrix3d As = vec_to_skew_matrix(vec_0_sw_hat) * R_03_orig_calc;
        const Matrix3d Bs = -vec_to_skew_matrix(vec_0_sw_hat) * As;
        const Matrix3d Cs = vec_0_sw_hat * vec_0_sw_hat.transpose() * R_03_orig_calc;

        const Matrix3d R_03_final = As * std::sin(arm_angle.value()) + Bs * std::cos(arm_angle.value()) + Cs;

        // step 4.2 从 R_03_final 计算 theta1, theta2, theta3
        // 计算theta2
        double sin_theta2 = -R_03_final(2, 2);
        // 验证 sin_final_theta2 范围
        if (sin_theta2 > 1.0 + EPSILON || sin_theta2 < -1.0 - EPSILON) {
            std::cout << " IK Std 警告: final sin(theta2) 值超出范围。跳过此解。" << std::endl;
            result.error_code = 4;
            return result;
        }
        // 安全处理
        sin_theta2 = std::max(-1.0, std::min(1.0, sin_theta2));
        double cos_theta2_base = std::sqrt(1.0 - sin_theta2 * sin_theta2);
        std::vector<double> possible_theta2_values;
        // TODO  >>>>> 解分支2 <<<<<
        // TODO 分支点2 theta2多解带来的分支 , 增加了normalize_angle_to_open_interval 使得其成为一个开区间
        possible_theta2_values.push_back(normalize_angle_to_open_interval(std::atan2(sin_theta2, cos_theta2_base)));
        if (cos_theta2_base > EPSILON) {
            possible_theta2_values.push_back(normalize_angle_to_open_interval(std::atan2(sin_theta2, -cos_theta2_base)));
        }
        else {
            std::cout << ">>>>>>>> IK Std cos_theta2_base == 0 <<<<<<<<" << std::endl;
        }

        // step 4.3 ：从矩阵中分离出theta2，从而求出不同组的解，但是这里要考虑cos_theta2=0 (theta2 = +- pi/2)的情况
        for (double theta2_choice : possible_theta2_values){ // 二次循环

            double cos_theta2 = cos(theta2_choice);
            double theta1, theta3;
            if (cos_theta2 == 0 ){
                std::cout << "IK Std 警告: 此时theta2选择为theta2 = +- pi/2, 发生了奇异" << std::endl;
                result.error_code = 6;
                theta1 = current_joints_array[0];
                theta3 = current_joints_array[2];

            }
            else {
                theta1 = std::atan2(R_03_final(0, 2)/cos_theta2, -R_03_final(1, 2)/cos_theta2);
                theta3 = std::atan2(R_03_final(2, 0)/cos_theta2, R_03_final(2, 1)/cos_theta2);
            }
            possible_theta_123.push_back({theta1, theta2_choice, theta3});   
        } //二次循环结束
    }// 一次循环结束

    // 至此， （单次循环中）至多产生了4组解 possible_theta_123,由theta2 的多解带来          
    // 遍历最终 possible_theta_123 的每种情况,

    for (const auto current_theta_123 : possible_theta_123) { //一次循环
        // 检查最终 theta2 的约束 (theta2 < 0)
        double current_theta1 = std::get<0>(current_theta_123);
        double current_theta2 = std::get<1>(current_theta_123);
        double current_theta3 = std::get<2>(current_theta_123);


        // --- Part 4: 求解 theta5, theta6, theta7 ---
        // T_07 是总的 target_pose
        const Matrix4d A1_final_transform = modified_DH_transform(current_theta1 + M_PI / 2.0, params_.d_bs, 0, 0);
        const Matrix4d A2_final_transform = modified_DH_transform(current_theta2 - M_PI / 2.0, 0, 0, M_PI / 2.0);
        const Matrix4d A3_final_transform = modified_DH_transform(current_theta3 + M_PI / 2.0, params_.d_se, 0, M_PI / 2.0); // 这里的 theta3 是 final_theta3

        const Matrix4d T_04 = A1_final_transform * A2_final_transform * A3_final_transform *
                                modified_DH_transform(theta4, 0, params_.a_se, -M_PI / 2.0); // 重新使用 fixed_theta4_param
        const Matrix4d T_47 = T_04.inverse() * T_07;
        const Matrix3d R_47 = T_47.block<3,3>(0,0);

        // 先求theta6
        double sin_theta6 = -R_47(1, 2);
        // 验证 sin_theta6 范围
        if (sin_theta6 > 1.0 + EPSILON || sin_theta6 < -1.0 - EPSILON) {
            std::cout << "警告 (IK Std): sin_theta6 值超出范围。跳过此解。" << std::endl;
            result.error_code = 7;
            continue;
        }

        sin_theta6 = std::max(-1.0, std::min(1.0, sin_theta6));
        double cos_theta6_base = std::sqrt(1.0 - sin_theta6 * sin_theta6);
        std::vector<double> possible_theta6_values;
        possible_theta6_values.push_back(normalize_angle_to_open_interval(std::atan2(sin_theta6, cos_theta6_base)));
        if (cos_theta6_base > EPSILON) {
            possible_theta6_values.push_back(normalize_angle_to_open_interval(std::atan2(sin_theta6, -cos_theta6_base)));
        }
        double theta5,theta7;
        std::vector<std::tuple<double, double, double>> possible_theta_567;
        // TODO  >>>>> 解分支3 <<<<<
        // TODO theta6 的多解带来的不同的解分支
        for (double current_theta6 : possible_theta6_values){ //二次循环
            double cos_theta6 = cos(current_theta6);
            if (cos_theta6 == 0) {
                std::cout << "theta6 = +- pi/2  , 发生奇异！！！" << std::endl;
                result.error_code = 8;
                theta5 = current_joints_array[4];
                theta7 = current_joints_array[6];

            }
            else{
                theta5 = std::atan2(R_47(2, 2)/cos_theta6, R_47(0, 2)/cos_theta6);
                theta7 = std::atan2(R_47(1, 1)/cos_theta6, -R_47(1, 0)/cos_theta6);
            }
            possible_theta_567.push_back({theta5,current_theta6,theta7});
            possible_serial_joints.push_back({current_theta1,current_theta2,current_theta3,
                                                        theta4,theta5,current_theta6,theta7});

        } // 二次循环结束
    } //一次循环结束


    if (ENABLE_LOG == true){
    std::cout << "possible_serial_joints size is : " <<
                     possible_serial_joints.size() << std::endl;

    std::cout << ">>>>>>>>> all possible solutions <<<<<<<<<<<<<" << std::endl;
    print_vec_of_tuples(possible_serial_joints);
    }

    // 到这里全部计算完了反解，得到至多8组反解
    result.all_solutions = possible_serial_joints;

    SerialJoints current_joints_tuple; 
    double_array_to_serial_joints(current_joints_array,current_joints_tuple);

    //检查关节范围合法性
    // 用于存储符合条件的解的向量
    std::vector<SerialJoints> checked_serial_joints;

    for (const auto& solution_tuple : possible_serial_joints) {
        // 1. 将 SerialJoints 转换为 Vector7d，因为 validate_solution 接受 Vector7d
        Vector7d solution_vector = serial_joints_to_vec7d(solution_tuple);

        // 2. 调用 validate_solution 进行验证
        auto validation_result = validate_solution(solution_vector, params_.joint_limits);

        // 3. 检查验证结果元组的第一个元素（布尔值），判断是否符合条件
        bool is_valid = std::get<0>(validation_result);


        if (is_valid) {
            // 4. 如果符合条件，加入到 checked_serial_joints
            checked_serial_joints.push_back(solution_tuple);

        } else {
            // 打印违规详情，仅用于调试
            // for (int violation_idx : std::get<2>(validation_result)) {
            //     std::cout << violation_idx << " ";
            // }
            // std::cout << "\n";
        }
    }


    if (checked_serial_joints.empty()) {
        std::cout << ">>> IK Std 计算未找到任何满足条件的解。<<<" << std::endl;
    }

    std::optional<SerialJoints> closest_solution =
        select_closest_ik_solution(checked_serial_joints, current_joints_tuple);

    if (closest_solution.has_value()) {
            // std::cout << "\n --- 找到最近的 IK 解 ---\n";
            // printSerialJointsManual(*closest_solution);
            // std::cout << "-------------------------------------------\n";

            result.is_valid = true;
            result.final_sol = closest_solution.value(); // 正确赋值
            result.theta7 = std::get<6>(result.final_sol);
    } else {
            std::cout << "\nIK Std : 未找到最近的 IK 解（可能在选择过程中发生内部错误）。\n";
            result.is_valid = false;
    }

    if (ENABLE_LOG == true){
        std::cout << ">>>>>>>>>>>> print area <<<<<<<<<<<<" << std::endl;
        std::cout << "计算得到的 sin_theta2_orig: " << sin_theta2_orig << std::endl;
        std::cout <<  "vec_0_sw :" << std::endl;
        print_vector(vec_0_sw);
        double denominator = params_.d_se - d_ew_new * cos(theta4);
        std::cout <<  "theta4 :" << theta4 << std::endl;
        // theta4 验算通过
        std::cout << "denominator: " << denominator << std::endl; 

        std::cout << "sin_theta2_orig" << sin_theta2_orig << std::endl;

        std::cout << "theta2_1_orig" << theta2_1_orig << std::endl;
        std::cout << "theta2_2_orig" << theta2_2_orig << std::endl;

        std::cout << "possible_theta_12_orig size is  :" << possible_theta_12_orig.size() << std::endl;

        std::cout << ">>>>>>>>>>>> print possible_theta2_orig_values <<<<<<<<<<<<" << std::endl;
        std::cout << "possible_theta2_orig_values size is : " << possible_theta2_orig_values.size() << std::endl;
        print_vec_of_tuples(possible_theta2_orig_values);


        std::cout << ">>>>>>>>>>>> print possible_theta_12_orig <<<<<<<<<<<<" << std::endl;
        std::cout << "possible_theta_12_orig size is : " << possible_theta_12_orig.size() << std::endl;
        print_vec_of_tuples(possible_theta_12_orig);

        std::cout << std::fixed << std::setprecision(4);

        // --- 打印每个 tuple 的值和维度 ---
        std::cout << "--- 打印 possible_theta2_orig_values (带维度信息) ---" << std::endl;
        std::cout << "Vector size: " << possible_theta2_orig_values.size() << std::endl;

        std::cout << std::fixed << std::setprecision(4); // 可选：设置浮点数精度
        print_vec_of_tuples(possible_theta2_orig_values);
        print_vec_of_tuples(possible_theta_12_orig);


    std::cout << ">>>>>>>>>>>> print completed <<<<<<<<<<<<" << std::endl;
    }

    return result;

}




// 7DOF SRS Ofst IK 主函数
IKResult ArmKineOfst::calculateIK(
    const Matrix4d& target_pose,
    double current_joints_array[],
    std::optional<double> arm_angle,
    std::optional<double> theta7
){
    IKResult result;
    result.is_valid =false;
    constexpr bool ENABLE_LOG = false; 

    // 如果取消偏置需要进行的参数设置(两个修改)
    const double a_wf_new = 0 ;
    const double d_ew_new = params_.d_ew + params_.a_wf;

    if (!theta7.has_value()) // 检查是否没有值 (即为 std::nullopt)
    {
        std::cerr << "IK Ofst: 函数输入错误， 未输入theta " << std::endl;
        return result;
    }
    else // 有值
    {
        result.theta7 = theta7.value(); // 使用 .value() 来获取内部的 double 值
    }

    // Step 1: 计算虚拟机械臂末端位姿
    Matrix4d A7 = modified_DH_transform(theta7.value(), 0, params_.a_wf, pi/2);
    Matrix3d R_08 = target_pose.block<3,3>(0,0);

    Vector4d P_8_wa(-params_.a_ee, 0, 0, 1);
    Vector4d P_0_wa = target_pose * P_8_wa;

    Matrix4d T_07a = Matrix4d::Identity();
    T_07a.block<3,3>(0,0) = R_08;
    T_07a.block<3,1>(0,3) = P_0_wa.head<3>();

    Matrix4d T_76_a = A7.inverse();
    Vector4d P_76_a = T_76_a.col(3);

    Vector4d P_07_v = T_07a * P_76_a;
    
    Matrix4d T_07_v = Matrix4d::Identity();
    T_07_v.block<3,3>(0,0) = R_08;
    T_07_v.block<3,1>(0,3) = P_07_v.head<3>();
    // 到这里已经全部换成了虚拟机械臂, T_07_v 是虚拟机械臂的末端姿态矩阵

    // Step 2: 计算theta4
    const double len_vec_SE = sqrt(pow(params_.d_se,2) + pow(params_.a_se,2));
    const double angle_ESEv = atan2(params_.a_se, params_.d_se);

    Vector4d P_0_S(0, 0, params_.d_bs, 1);
    Vector3d vec_0_sw = T_07_v.block<3,1>(0,3) - P_0_S.head<3>();
    const double len_vec_0_sw = vec_0_sw.norm();

    const double angle_SEW = acos(
        std::clamp((pow(len_vec_SE,2) + pow(params_.d_ew,2) - pow(len_vec_0_sw,2)) / 
             (2*len_vec_SE*params_.d_ew), -1.0, 1.0)
    );

    const double len_vec_EW = params_.d_ew;
    const double angle_ESW = std::acos(
        std::clamp(
        (std::pow(len_vec_SE, 2) + std::pow(len_vec_0_sw, 2) - std::pow(len_vec_EW, 2)) / 
        (2 * len_vec_SE * len_vec_0_sw),-1.0, 1.0)
        );
    const double angle_EvSW = angle_ESW + angle_ESEv;

    //判断臂长
    const double max_arm_lengeth = len_vec_SE + len_vec_EW; //虚拟机械臂的最大臂长，常数不会变
    double current_arm_length = len_vec_0_sw; //虚拟机械臂的肩腕关节向量的模长
    if (current_arm_length>=max_arm_lengeth ){
        std::cout << " >>>>>  IK Ofst 超出了工作空间 <<<<<" << std::endl;
        result.error_code = 10;
        return result;
    }

    // TODO  >>>>> 解分支0 : 肘关节分支 （忽视）<<<<<
    // 此处仅按照肘关节向上的构态来处理
    double theta4 = angle_ESEv + pi - angle_SEW;
    // 到此处theta4求解完成

    /*
    step 3 :计算参考平面下的前三个关节的初始值
    参考平面为theta3 = 0 的手臂平面，参考08 T-Ro
    */

   // step 3.1 : 计算theta2_orig
   //提取肩腕关节向量元素进行运算

    double c1 = vec_0_sw(0);
    double c2 = vec_0_sw(1);
    double c3 = vec_0_sw(2);
    double denominator = params_.d_se + params_.d_ew * cos(theta4);

    // 避免除以零或非常小的数
    if (std::abs(denominator) < std::numeric_limits<double>::epsilon()) {
        std::cerr << "IK Ofst 错误：分母接近零，无法计算 sin_theta2。" << std::endl;
    }
    double sin_theta2_orig = -c3 / denominator;

    // --- 验证 sin_theta2_orig 的范围 ---
    // 由于浮点数精度问题，允许一点误差
    if (sin_theta2_orig > 1.0 + std::numeric_limits<double>::epsilon() ||
        sin_theta2_orig < -1.0 - std::numeric_limits<double>::epsilon()) {
        std::cerr << "IK Ofst 错误：sin_theta2 的值超出 [-1, 1] 范围。无实数解 theta2。" << std::endl;
    }

    // TODO 这里做了范围约束
    // 限制在 [-1, 1] 范围内，以防微小的浮点误差导致 sqrt(负数)
    if (sin_theta2_orig > 1.0) sin_theta2_orig = 1.0;
    if (sin_theta2_orig < -1.0) sin_theta2_orig = -1.0;

    // --- 计算 cos_theta2_orig 的两种可能解 ---
    //TODO >>>>> 解分支1 <<<<< 
    double cos_theta2__orig_base = sqrt(1.0 - sin_theta2_orig * sin_theta2_orig);
    // std::cout << "cos_theta2__orig_base: " << cos_theta2__orig_base << std::endl;

    // --- 计算 theta2_orig 的两种可能解 ---
    // -- 两者的关系： theta2_1_orig + theta2_2_orig =  +- pi (两者符号相同)
    double theta2_1_orig = std::atan2(sin_theta2_orig, cos_theta2__orig_base);
    double theta2_2_orig = std::atan2(sin_theta2_orig,-cos_theta2__orig_base);

    std::vector<std::tuple<double>> possible_theta2_orig_values;
    possible_theta2_orig_values.push_back({theta2_1_orig});
    // 只有当 cos_theta2_orig_base 不接近零时，才有第二个独立的解
    if (cos_theta2__orig_base > EPSILON) { 
        possible_theta2_orig_values.push_back({theta2_2_orig});
    }
    else { //这个是最外层的循环
        std::cout << "IK Ofst 只记录了1个theta2_orig,因此少一次方" << std::endl;
    }

    std::vector<SerialJoints> possible_serial_joints;
    // step 3.2 :遍历 theta2_orig 的每种情况， 求theta1_orig，并检查是否合理
    std::vector<std::tuple<double,double>> possible_theta_12_orig; // 存储theta1_orig, theta2_orig
    for (auto current_theta2_orig_temp : possible_theta2_orig_values) { //一次循环

        // --- Part 2: 求解 theta1_orig (基于线性方程组) ---
        double current_theta2_orig = std::get<0>(current_theta2_orig_temp);
        double cos_theta2_val_orig = std::cos(current_theta2_orig); // 不同的theta2_orig_case 结果相差一个负号
        double sin_theta4_val = std::sin(theta4);
        double cos_theta4_val = std::cos(theta4);
   
        double a1 = cos_theta2_val_orig * (params_.d_se+params_.d_ew*cos_theta4_val);
        double b1 = params_.a_se + params_.d_ew * sin_theta4_val;
        double a2 = params_.a_se + params_.d_ew * sin_theta4_val;
        double b2 = -cos_theta2_val_orig * (params_.d_se+params_.d_ew*cos_theta4_val);
 
        double D = a1 * b2 - a2 * b1;

        double current_theta1_orig = std::nan(""); // 初始化为 NaN

        if (std::abs(D) > EPSILON) { // D 不为零，有唯一解  
            double x_sin_theta1_sol = (c1 * b2 - c2 * b1) / D;
            double y_cos_theta1_sol = (a1 * c2 - a2 * c1) / D;
            // 验证 x_sin_theta1_sol^2 + y_cos_theta1_sol^2 = 1,满足此条件，此解才有效
            double check_sum_sq = x_sin_theta1_sol * x_sin_theta1_sol + y_cos_theta1_sol * y_cos_theta1_sol;
            if (std::abs(check_sum_sq - 1.0) < EPSILON * 10.0) {
                current_theta1_orig = std::atan2(x_sin_theta1_sol, y_cos_theta1_sol);
                possible_theta_12_orig.push_back({current_theta1_orig,current_theta2_orig});
            } else {
                std::cout << "IK Ofst 警告 : 对于 theta2_orig=" << current_theta2_orig * 180.0 / M_PI
                          << "度，sin^2+cos^2 != 1。跳过此解。" << std::endl;
                std::cout << "x_sin_theta1_sol :" << x_sin_theta1_sol << std::endl;
                std::cout << "y_cos_theta1_sol :" << y_cos_theta1_sol << std::endl;
                result.error_code = 1;
                return result;
                continue; // 解不一致，跳过
            }
        } else {
            std::cout << "IK Ofst 警告: 对于 theta2_orig=" << current_theta2_orig * 180.0 / M_PI
                      << "度，行列式 D 接近零。跳过此解。" << std::endl;
            result.error_code = 2;
            return result;
            continue; // 奇异点，跳过此解
        }
    }// 一次循环结束

    /*
    在上面的计算中，得到了至多两组 theta1_orig, theta2_orig, theta3_orig=0,然后循环求解 
    TODO 分支点1， theta2_orig 多解带来的分支
    step 4 : 基于theta1_orig  theta2_orig 和 original theta3 = 0，计算前三个过节的实际值，计算经过臂角作用后的R_03
    */
    std::vector<std::tuple<double, double, double, double>> possible_theta_123_and_phis; // theta1,theta2,theta3,phi
    std::vector<SerialJointsWithPhi> possible_serial_joints_and_phis;

    for (auto current_theta_12_orig : possible_theta_12_orig){ // 一次循环
        std::vector<double> possible_phis;
        
        //计算变换矩阵
        const Matrix4d A1_orig_transform= modified_DH_transform(std::get<0>(current_theta_12_orig) + M_PI / 2.0, params_.d_bs, 0, 0);
        const Matrix4d A2_orig_transform= modified_DH_transform(std::get<1>(current_theta_12_orig) - M_PI / 2.0, 0, 0, M_PI / 2.0);
        const Matrix4d A3_orig_transform= modified_DH_transform(0.0 + M_PI / 2.0, params_.d_se, 0, M_PI / 2.0);
        const Matrix4d A4_orig_transform = modified_DH_transform(theta4, 0, params_.a_se, -M_PI / 2.0);

        Matrix4d T_03_orig_calc = A1_orig_transform * A2_orig_transform * A3_orig_transform;
        Matrix3d R_03_orig_calc = T_03_orig_calc.block<3, 3>(0, 0);

        // step 4.1 : 计算phi角
        // TODO 解分支2 ： phi 多解 （可能可以消掉）
        const Vector3d vec_0_sw_hat = vec_0_sw.normalized();
        const Matrix3d As = vec_to_skew_matrix(vec_0_sw_hat) * R_03_orig_calc;
        const Matrix3d Bs = -vec_to_skew_matrix(vec_0_sw_hat) * As;
        const Matrix3d Cs = vec_0_sw_hat * vec_0_sw_hat.transpose() * R_03_orig_calc;

        const Matrix4d A4 = modified_DH_transform(theta4, 0, params_.a_se, -pi/2);
        const Matrix3d R4 = A4.block<3,3>(0,0);
        const Matrix3d Aw = R4.transpose() * As.transpose() * R_08;
        const Matrix3d Bw = R4.transpose() * Bs.transpose() * R_08;
        const Matrix3d Cw = R4.transpose() * Cs.transpose() * R_08;

        const double tan_theta7 = tan(theta7.value());

        const double P =  tan_theta7 * Aw(1,0) + Aw(1,1);
        const double Q =  tan_theta7 * Bw(1,0) + Bw(1,1);
        const double R = -tan_theta7 * Cw(1,0) - Cw(1,1);

        const double Z = sqrt(pow(P,2)+pow(Q,2)); // 振幅


        
        if (R<=Z){ // 这个地方的判据开展要修改
            double Delta = pow(P,2)+pow(Q,2)-pow(R,2);
            double temp_item1= 0;
            if (Delta < 0 ) {
                if (ENABLE_LOG == true){
                    print_matrix(Aw);
                    print_matrix(Bw);
                    print_matrix(Cw);
                    std::cout << " P = " << P << std::endl;
                    std::cout << " Q = " << Q << std::endl;
                    std::cout << " R = " << R << std::endl;
                    std::cout << " Z = " << Z << std::endl;
                }
                std::cerr << "IK Ofst 错误：Delta < 0 ,无法求解臂角" << std::endl;
                result.error_code = 5;
                // return result;
            }
            else{
                temp_item1 = sqrt(Delta);
            }
            // std::cout << "temp_item1 = "  << temp_item1 << std::endl;
            double phi_1 = atan2(R, temp_item1)-atan2(Q,P);
            double phi_2 = atan2(R,-temp_item1)-atan2(Q,P);
            double normalized_phi_1 = normalize_angle_to_neg_pi_to_pi(phi_1);
            double normalized_phi_2 = normalize_angle_to_neg_pi_to_pi(phi_2);
            possible_phis.push_back(normalized_phi_1);
            possible_phis.push_back(normalized_phi_2);
            // std::cout << ">>>>> phi from triangle : " << std::endl;
            // std::cout << "phi_1 ： " << normalized_phi_1 << std::endl;
            // std::cout << "phi_2 ： " << normalized_phi_2 << std::endl;
        }
        else{
            std::cout << "IK Ofst 臂角无法计算，超出范围" << std::endl;
            result.error_code = 3;
            return result;
        }

        // step 4.2 : 计算arm_angle 上的旋转矩阵

        for (const auto& current_phi : possible_phis){// 二次循环
            double arm_angle = current_phi;
            const Matrix3d R_03_final = As * std::sin(arm_angle) + Bs * std::cos(arm_angle) + Cs;

            // step 4.3 从 R_03_final 计算 theta1, theta2, theta3
            double sin_theta2 = -R_03_final(2, 2);
            // std::cout << "sin_theta2 : " << sin_theta2 << std::endl;
            // 验证 sin_final_theta2 范围
            if (sin_theta2 > 1.0 + EPSILON || sin_theta2 < -1.0 - EPSILON) {
                std::cout << "IK Ofst 警告: final sin(theta2) 值超出范围。跳过此解。" << std::endl;
                result.error_code = 4;
                return result;
                continue;
            }
            // 安全处理
            sin_theta2 = std::max(-1.0, std::min(1.0, sin_theta2));
            double cos_theta2_base = std::sqrt(1.0 - sin_theta2 * sin_theta2);
            // std::cout << "cos_theta2_base = " << cos_theta2_base << std::endl;
            std::vector<double> possible_theta2_values;
            // TODO  >>>>> 解分支3 <<<<<
            // TODO 分支点3 theta2多解带来的分支 , 增加了normalize_angle_to_open_interval 使得其成为一个开区间
            possible_theta2_values.push_back(normalize_angle_to_open_interval(std::atan2(sin_theta2, cos_theta2_base)));
            if (cos_theta2_base >= EPSILON) {
                possible_theta2_values.push_back(normalize_angle_to_open_interval(std::atan2(sin_theta2, -cos_theta2_base)));
            }
            else {
                std::cout << ">>>>>>>> IK Ofst  cos_theta2_base == 0, 此时发生了奇异 <<<<<<<<" << std::endl;
                std::cout << "cos_theta2_base = " << cos_theta2_base << std::endl;
                std::cout << "sin_theta2 = " << sin_theta2 << std::endl;

                // result.error_code = 5;
                // return result;
            }
            // std::cout << "possible_theta2_values size is : " << possible_theta2_values.size() << std::endl;
            // step 4.4 ：从矩阵中分离出theta2，从而求出不同组的解，但是这里要考虑cos_theta2=0 (theta2 = +- pi/2)的情况
            for (double theta2_choice : possible_theta2_values){ // 三次循环

                double cos_theta2 = cos(theta2_choice);
                double theta1, theta3;
                if (fabs(cos_theta2) <= EPSILON ){
                    std::cout << "IK Ofst 警告，此时theta2选择为theta2 = +- pi/2, 发生了奇异" << std::endl;
                    theta1 = current_joints_array[0];
                    theta3 = current_joints_array[2];
                    // result.error_code = 6;
                    // return result;
                }
                else {
                    theta1 = std::atan2(R_03_final(0, 2)/cos_theta2, -R_03_final(1, 2)/cos_theta2);
                    theta3 = std::atan2(R_03_final(2, 0)/cos_theta2, R_03_final(2, 1)/cos_theta2);
                }
                possible_theta_123_and_phis.push_back({theta1, theta2_choice, theta3, current_phi});  
                // temp_phis_sequence.push_back(current_phi);
            } //三次循环结束
        }//二次循环结束
    } // 一次循环结束，到此 possible_theta_123，temp_phis_sequence应该存放了8组解
    // std::cout << "possible_theta_123_and_phis size is : " << possible_theta_123_and_phis.size() << std::endl;
    // printVectorOfTuples(possible_theta_123_and_phis);

    // std::vector<double> all_phis_sequence;
    // 遍历最终 possible_theta_123_and_phis 的每种情况

    for (const auto current_theta_123_and_phi : possible_theta_123_and_phis) { //一次循环
        // 检查最终 theta2 的约束 (theta2 < 0)
        double current_theta1 = std::get<0>(current_theta_123_and_phi);
        double current_theta2 = std::get<1>(current_theta_123_and_phi);
        double current_theta3 = std::get<2>(current_theta_123_and_phi);
        double current_phi = std::get<3>(current_theta_123_and_phi);


        // --- Part 4: 求解 theta5, theta6, theta7 ---
        // T_07 是总的 target_pose
        const Matrix4d A1_final_transform = modified_DH_transform(current_theta1 + M_PI / 2.0, params_.d_bs, 0, 0);
        const Matrix4d A2_final_transform = modified_DH_transform(current_theta2 - M_PI / 2.0, 0, 0, M_PI / 2.0);
        const Matrix4d A3_final_transform = modified_DH_transform(current_theta3 + M_PI / 2.0, params_.d_se, 0, M_PI / 2.0); // 这里的 theta3 是 final_theta3

        const Matrix4d T_04 = A1_final_transform * A2_final_transform * A3_final_transform *
                                modified_DH_transform(theta4, 0, params_.a_se, -M_PI / 2.0); // 重新使用 fixed_theta4_param
        const Matrix4d T_47 = T_04.inverse() * T_07_v;
        const Matrix3d R_47 = T_47.block<3,3>(0,0);

        // 先求theta6
        double sin_theta6 = -R_47(1, 2);
        // 验证 sin_theta6 范围
        if (sin_theta6 > 1.0 + EPSILON || sin_theta6 < -1.0 - EPSILON) {
            std::cout << "IK Ofst 警告 (IK): sin_theta6 值超出范围。跳过此解。" << std::endl;
            result.error_code = 7;
            return result;
            continue;
        }
        sin_theta6 = std::max(-1.0, std::min(1.0, sin_theta6));
        double cos_theta6_base = std::sqrt(1.0 - sin_theta6 * sin_theta6);
        std::vector<double> possible_theta6_values;
        possible_theta6_values.push_back(normalize_angle_to_open_interval(std::atan2(sin_theta6, cos_theta6_base)));
        if (cos_theta6_base > EPSILON) {
            possible_theta6_values.push_back(normalize_angle_to_open_interval(std::atan2(sin_theta6, -cos_theta6_base)));
        }
        double theta5,theta7_cal;
        std::vector<std::tuple<double, double, double, double>> possible_theta_567_and_phi; // theta5, theta6, theta7, phi

        // int cnt_phi = 0;
        // TODO  >>>>> 解分支4 <<<<<
        // TODO theta6 的多解带来的不同的解分支
        
        for (double current_theta6 : possible_theta6_values){ //二次循环
            double cos_theta6 = cos(current_theta6);
            if (fabs(cos_theta6) < EPSILON) {
                std::cout << "IK Ofst theta6 =0 +- pi/2 , 发生奇异！！！" << std::endl;
                theta5 = current_joints_array[4];
                theta7_cal = theta7.value();
                // result.error_code = 8;
                // return result;
            }
            else{
                theta5 = std::atan2(R_47(2, 2)/cos_theta6, R_47(0, 2)/cos_theta6);
                theta7_cal = std::atan2(R_47(1, 1)/cos_theta6, -R_47(1, 0)/cos_theta6);
            }
            possible_theta_567_and_phi.push_back({theta5,current_theta6,theta7_cal, current_phi});
            // all_phis_sequence.push_back(possible_phis[cnt_phi]);
            // cnt_phi++;

        } // 二次循环结束

        // TODO 根据theta7 筛掉一个分支
        // possible_theta_567 size is 2 (2* 8)
        for (const auto& current_theta_567_and_phi : possible_theta_567_and_phi){
            double current_theta7 = std::get<2>(current_theta_567_and_phi);
  

            if (std::abs(current_theta7 - theta7.value()) <= EPSILON*100){
                double current_theta5 = std::get<0>(current_theta_567_and_phi);
                double current_theta6 = std::get<1>(current_theta_567_and_phi);
                possible_serial_joints.push_back({current_theta1,current_theta2,current_theta3,
                                                        theta4,current_theta5,current_theta6,current_theta7});
                possible_serial_joints_and_phis.push_back({current_theta1,current_theta2,current_theta3,
                                                        theta4,current_theta5,current_theta6,current_theta7, current_phi});

            }
            else {
                continue;
            }
        }

    } //一次循环结束

    // 到这里应该计算全部完成，得到了至多16组解
    if (ENABLE_LOG == true){
        std::cout << "possible_serial_joints size is : " <<
                     possible_serial_joints.size() << std::endl;
        std::cout << ">>>>>>>>> all possible solutions <<<<<<<<<<<<<" << std::endl;
        print_vec_of_tuples(possible_serial_joints_and_phis);
    }

    result.all_solutions = possible_serial_joints;

    //进行筛选
    // 用于存储符合条件的解的向量
    std::vector<SerialJoints> checked_serial_joints;
    std::vector<std::tuple<double,double,double,double,double,double,double,double>> checked_serial_joints_with_phi;
    for (const auto& solution_tuple : possible_serial_joints_and_phis){
        solWithPhi sol_temp_with_phi;
        seperate_serial_joints_with_arm_angle(sol_temp_with_phi,solution_tuple);
        SerialJoints serial_tuple = sol_temp_with_phi.sol;
        double current_phi = sol_temp_with_phi.arm_angle;
        Vector7d solution_vector = serial_joints_to_vec7d(serial_tuple);
        auto validation_result = validate_solution(solution_vector, params_.joint_limits);
        bool is_valid = std::get<0>(validation_result);
        if (is_valid) {
            // 4. 如果符合条件，加入到 checked_serial_joints
            checked_serial_joints_with_phi.push_back(solution_tuple);
        } else {
            // std::cout << " 不符合条件，已过滤。违规关节索引：";
            // 打印违规详情，仅用于调试
            // for (int violation_idx : std::get<2>(validation_result)) {
            //     std::cout << violation_idx << " ";
            // }
            // std::cout << "\n";
        }

    }



    if (checked_serial_joints_with_phi.empty()) {
        result.error_code = 9;
        return result;
        std::cout << " IK 计算未找到任何满足条件的解。" << std::endl;
    }

    SerialJoints current_joints_serial; 
    double_array_to_serial_joints(current_joints_array,current_joints_serial);
    std::optional<std::tuple<double, double, double, double, double, double, double, double>> 
        closest_solution_with_phi =
            select_closest_ik_solution_with_phi(checked_serial_joints_with_phi, current_joints_serial);

    if (result.error_code != -1) {result.is_valid = false;}

    if (closest_solution_with_phi.has_value()) {


            result.is_valid = true;
            solWithPhi temp_result; 
            seperate_serial_joints_with_arm_angle(temp_result,closest_solution_with_phi.value());
            result.final_sol = temp_result.sol;
            result.arm_angle = temp_result.arm_angle;
               
            
    } else {
            std::cout << "\n未找到最近的 IK 解（可能在选择过程中发生内部错误）。\n";
            result.is_valid = false;
    }

    return result;
}


// --- 组合 IK 的实现 ---
IKResult ArmKineComb::calculateIK(
    const Matrix4d& target_pose,
    double current_joints_array[],
    std::optional<double> arm_angle,
    std::optional<double> theta7) // 注意：这里不需要默认参数
{
    IKResult combined_result;
    combined_result.is_valid = false;


    // 先用 Standard 反解求一个初始解,利用std 求出的theta7 传给Ofst， 从而接近指定ofst 的臂角
    IKResult std_ik_result = std_solver_->calculateIK(
        target_pose,
        current_joints_array,
        arm_angle,
        std::nullopt 
    );

    if (std_ik_result.is_valid) {

        // 假设 Offset IK 需要 theta7，而 Standard IK 不需要
        IKResult ofst_ik_result = ofst_solver_->calculateIK(
            target_pose,
            current_joints_array, // 传入 std 的结果作为 Ofst 的初始猜测
            std::nullopt,
            std_ik_result.theta7 // 将 std 算出的 theta7 传给 ofst_solver
        );

        if (ofst_ik_result.is_valid) {
            combined_result = ofst_ik_result; // 使用 Offset 的结果
        } else {
            std::cerr << "ArmKineComb: Ofst IK failed to Calculate the result." << std::endl;
            combined_result = std_ik_result; // 或者返回 Std 的结果，近似处理
        }
    } else {
        std::cerr << "ArmKineComb: Std IK failed. Cannot proceed with Tee." << std::endl;

    }

    return combined_result;
}

// --- ArmKine FK 的实现 (沿用 ArmKineOfst 的正解) ---
FKResult ArmKineComb::calculateFK(const Vector7d& theta) {
    FKResult result;
    // 直接委托给 ofst_solver_ 的 calculateFK 方法
    if (ofst_solver_) { // 确保 ofst_solver_ 有效
        return ofst_solver_->calculateFK(theta);
    } else {
        std::cerr << "Error: ArmKineComb::calculateFK called but offset solver is not initialized!" << std::endl;
        return result;
    }
}




 