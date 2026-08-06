#include "kinematics_new.h"
#include "tools.h"
#include "kinematics_cal_tools.h"
#include <cmath>
#include <iostream>

constexpr bool ENABLE_STD_LOG = false; 
constexpr bool ENABLE_OFST_LOG = false; 
const bool SELECT_SINGLE_SOL_LOG = false; 
const int SELECT_STD_ID = 0; // 选择的解的索引，从0开始;
const int SELECT_OFST_ID = 0; // 选择的解的索引，从0开始;


const Vector3d vec_ref(0,1,0);



// 7DOF SRS Std IK 主函数
IKResult ArmKineStd::calculateIK_vec_ref(
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

    /*
    现在有一些地方可能要分类讨论
    但是现在还不清楚如何区分
    <似乎不用区分，只需要区分z4_r就好>
    暂时用case_id来标记
    0：适合大多数区域，肘关节偏转大
    1：适合某些特殊区域，肘关节偏转小
    case_id 影响的值：
    angle_EvSW； theta4; z4_r
    z4_r 用vec_0_sw_hat.x()的符号进行判断
    */ 

    int case_id;



    // 打印所需参数定义
    Eigen::IOFormat fmt_p(Eigen::StreamPrecision, 0, ", ", "\n", "[", "]");
    Eigen::IOFormat fmt_t(4, 0, ", ", "\n", "|", "|");



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
    const double d_ew_new = this->get_A5_d_param();

    // 新增考虑a_ew
    const double len_ew = std::hypot(d_ew_new, params_.a_ew);


    // Step 2: 计算theta4， 只考虑theta4_up 
    // hypot计算平方和的平方根，更加稳定
    const double len_se = std::hypot(params_.d_se, params_.a_se);
    const double angle_ESEv = atan2(params_.a_se, params_.d_se);
    Vector4d P_0_S(0, 0, params_.d_bs, 1);
    //vec_0_sw 是0坐标系下的肩腕向量
    Vector3d vec_0_sw = T_07.block<3,1>(0,3) - P_0_S.head<3>();
    const double len_sw = vec_0_sw.norm();
    const double angle_SEW = acos(
        std::clamp((pow(len_se,2) + pow(len_ew,2) - pow(len_sw,2)) / 
                (2*len_se*len_ew), -1.0, 1.0)
        );
    const double angle_ESW = acos(
        std::clamp(
        (std::pow(len_se, 2) + std::pow(len_sw, 2) - std::pow(len_ew, 2)) / 
        (2 * len_se * len_sw),-1.0, 1.0)
        );


    //根据vec_0_sw_hat.x()的符号判断case_id
    // 测试是否可以用vec_0_sw_hat.x()来判断case_id
    // if (vec_0_sw.x() >=0){
    //     case_id = 0;
    // }
    // else{
    //     case_id = 1;
    // }
    case_id = 0; // 先固定为0，测试效果
    std::cout << " IK Std case_id = " << case_id << std::endl;

    const double gamma2 = atan2(d_ew_new, -params_.a_ew);

    // TODO 分情况讨论 angle_EvSW
    const double angle_EvSW_v1 = angle_ESEv + angle_ESW;
    const double angle_EvSW_v2 = angle_ESEv - angle_ESW;
    
    // Elbow up TODO
    // TODO 分情况讨论 theta4
    double theta4_v1 = angle_ESEv + 3.0/2.0*pi - angle_SEW - gamma2;
    double theta4_v2 = angle_SEW - 0.5*pi + angle_ESEv - gamma2;

    double theta4;
    double angle_EvSW;
    if (case_id ==0){
        theta4 = theta4_v1;
        angle_EvSW = angle_EvSW_v1;
    }
    else {
        theta4 = theta4_v2;
        angle_EvSW = angle_EvSW_v2;
    }



    //判断臂长
    const double max_arm_lengeth = len_se + len_ew; //虚拟机械臂的最大臂长，常数不会变
    double current_arm_length = len_sw; //虚拟机械臂的肩腕关节向量的模长


    if (current_arm_length>=max_arm_lengeth ){
        std::cout << " >>>>>  IK Std 超出了工作空间 <<<<<" << std::endl;
        result.error_code = 10;
        return result;
    }



    //------- 核心改动： 改动参考平面的定义 ----------------
    const Vector3d vec_0_sw_hat = vec_0_sw.normalized();
    // z4_r 需要检查定义
    Vector3d z4_r = (vec_0_sw_hat.x() >= 0) ?  
        vec_0_sw_hat.cross(vec_ref) : 
        vec_ref.cross(vec_0_sw_hat);
    // Vector3d z4_r = vec_0_sw_hat.cross(vec_ref);
    Vector3d z4_r_hat = z4_r.normalized();


    const double gamma = angle_EvSW;
    const double Theta = pi/2 - gamma;
    const Matrix3d R_sup = Matrix3d::Identity() + 
            sin(Theta) * vec_to_skew_matrix(z4_r_hat) +
            (1-cos(Theta)) * vec_to_skew_matrix(z4_r_hat) * vec_to_skew_matrix(z4_r_hat);
    const Vector3d x4_r_hat = R_sup * vec_0_sw_hat;
    const Vector3d y4_r_hat = z4_r_hat.cross(x4_r_hat);

    Matrix3d R_03_r;
    R_03_r.col(0) =  x4_r_hat;
    R_03_r.col(1) =  z4_r_hat; 
    R_03_r.col(2) = -y4_r_hat; 


    //------- 核心改动结束 ----------------


    // 
    const Matrix3d As = vec_to_skew_matrix(vec_0_sw_hat) * R_03_r;
    const Matrix3d Bs = -vec_to_skew_matrix(vec_0_sw_hat) * As;
    const Matrix3d Cs = vec_0_sw_hat * vec_0_sw_hat.transpose() * R_03_r;


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
    
    Vector3d n_ref = vec_0_sw.cross(vec_ref); // 参考平面的法向量；

    std::vector<std::tuple<double, double, double, Vector3d>> possible_theta_123_and_refplane;
    // step 4.3 ：从矩阵中分离出theta2，从而求出不同组的解，但是这里要考虑cos_theta2=0 (theta2 = +- pi/2)的情况
    for (double theta2_choice : possible_theta2_values){ 

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
        possible_theta_123_and_refplane.push_back({theta1, theta2_choice, theta3, n_ref});   
    } 

    std::vector<SerialJointsWithPlane> possible_serial_joints_and_refplane;


    for (const auto current_theta_123 : possible_theta_123_and_refplane) { //一次循环
        // 检查最终 theta2 的约束 (theta2 < 0)
        double current_theta1 = std::get<0>(current_theta_123);
        double current_theta2 = std::get<1>(current_theta_123);
        double current_theta3 = std::get<2>(current_theta_123);
        const auto current_ref_plane = std::get<3>(current_theta_123);


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
            possible_serial_joints_and_refplane.push_back({current_theta1,current_theta2,current_theta3,
                                                        theta4,theta5,current_theta6,theta7, current_ref_plane});

        } // 二次循环结束
    } //一次循环结束

    std::vector<SerialJoints> possible_serial_joints;
    std::vector<Vector3d> possible_n_ref;
    // separate_vec_serial_joints_with_plane(possible_serial_joints_and_refplane, 
    //                                         possible_serial_joints, 
    //                                         possible_n_ref);
    separate_vector_single(possible_serial_joints_and_refplane, 
                                            possible_serial_joints, 
                                            possible_n_ref);





    // 到这里全部计算完了反解，得到至多8组反解
    result.all_solutions = possible_serial_joints;



    SerialJoints current_joints_tuple; 
    double_array_to_serial_joints(current_joints_array,current_joints_tuple);

    //检查关节范围合法性
    // 用于存储符合条件的解的向量
    std::vector<SerialJoints> checked_serial_joints;


    // 存储合法之后的解的在possible_serial_joints之中的索引
    std::vector<size_t> valid_solution_indices;


    for (size_t i = 0; i < possible_serial_joints.size(); ++i){
        const auto& solution_tuple = possible_serial_joints[i];

        // 1. 将 SerialJoints 转换为 Vector7d，因为 validate_solution 接受 Vector7d
        Vector7d solution_vector = serial_joints_to_vec7d(solution_tuple);

        // 2. 调用 validate_solution 进行验证
        auto validation_result = validate_solution(solution_vector, params_.joint_limits);

        // 3. 检查验证结果元组的第一个元素（布尔值），判断是否符合条件
        bool is_valid = std::get<0>(validation_result);


        if (is_valid) {
            // 4. 如果符合条件，加入到 checked_serial_joints
            checked_serial_joints.push_back(solution_tuple);
            valid_solution_indices.push_back(i);

        } else {
            // 打印违规详情，仅用于调试
            // for (int violation_idx : std::get<2>(validation_result)) {
            //     std::cout << violation_idx << " ";
            // }
            // std::cout << "\n";
        }
    }


    // std::cout << "Valid indices: ";
    // for (size_t index : valid_solution_indices) {
    //     std::cout << index << " ";
    // }
    // std::cout << std::endl;

    if (checked_serial_joints.empty()) {
        std::cout << ">>> IK Std 计算未找到任何满足条件的解。<<<" << std::endl;
    }



    // std::optional<SerialJoints> closest_solution =
    //     select_closest_ik_solution(checked_serial_joints, current_joints_tuple);

    std::optional<SerialJointsWithIndex> closest_solution_with_index =
        select_closest_ik_solution_with_index(checked_serial_joints, current_joints_tuple);

    if (closest_solution_with_index.has_value()) {
            // std::cout << "\n --- 找到最近的 IK 解 ---\n";
            // printSerialJointsManual(*closest_solution);
            // std::cout << "-------------------------------------------\n";
            
            SerialJoints closest_solution = extract_serialjoints_from_tuple(closest_solution_with_index.value());
            // 存储最近的解的在 checked_serial_joints 之中的索引
            const auto index_in_clos_sol = std::get<7>(closest_solution_with_index.value());
            int n_ref_index = valid_solution_indices[index_in_clos_sol];
            const auto clos_sol_n_ref = possible_n_ref[n_ref_index];


            result.is_valid = true;
            result.final_sol = closest_solution; // 正确赋值
            result.n_ref = clos_sol_n_ref;
            result.n_ref = n_ref;

    } else {
            std::cout << "\nIK Std : 未找到最近的 IK 解（可能在选择过程中发生内部错误）。\n";
            result.is_valid = false;
    }




// --------------------------打印区域 --------------------------//
    if (ENABLE_STD_LOG == true){
        std::cout << "\n\n\n>>>>>>>>>>>> STD  print area <<<<<<<<<<<<" << std::endl;
        std::cout << std::fixed << std::setprecision(4); // 可选：设置浮点数精度

        // std::cout << "current_arm_length : " << current_arm_length << 
        //             ", max_arm_lengeth : " << max_arm_lengeth << std::endl; 

        // std::cout <<  "vec_0_sw :" << std::endl;
        // print_vector(vec_0_sw);
        // std::cout <<  "theta4 :" << theta4 << std::endl;
        // theta4 验算通过

        // std::cout << "d_ew_new in IK Std = " << d_ew_new << std::endl;


        // std::cout << " R_03_final = " <<  std::endl;
        // print_matrix(R_03_final);

        std::cout << "possible_serial_joints_and_refplane size is : " <<
                            possible_serial_joints_and_refplane.size() << std::endl;

        std::cout << ">>>>>>>>> all possible solutions <<<<<<<<<<<<<" << std::endl;
        // auto 
        print_vec_of_tuples(possible_serial_joints);




    std::cout << ">>>>>>>>>>>>STD  print completed <<<<<<<<<<<<\n\n\n" << std::endl;
    }

// --------------------------打印区域 --------------------------//

    // 临时修改区域
    if (SELECT_SINGLE_SOL_LOG == true){
        std::optional<SerialJoints> selected_solution = 
            select_sol_from_possible(SELECT_STD_ID, possible_serial_joints);
        if (selected_solution.has_value()){
            // std::cout << "\n >>>>> 选择了第 " << SELECT_STD_ID << " 个解  <<<<< \n";
        } else {
            std::cout << "\n >>>>> 选择解失败，索引超出范围  <<<<< \n";
        }
        result.final_sol = selected_solution.value();
    }

    result.theta7 = std::get<6>(result.final_sol);



    return result;

}


IKResult ArmKineOfst::calculateIK_vec_ref(
    const Matrix4d& target_pose,
    double current_joints_array[],
    std::optional<double> arm_angle,
    std::optional<double> theta7
){
    IKResult result;
    result.is_valid =false;

    int case_id;

    // (两个修改)
    //  ********        修改1        ***********
    // 下面进行的参数设置 为 取消偏置， 注释情况下 就是正常带偏置 
    // const double a_wf_new = 0 ;
    // const double d_ew_new = params_.d_ew + params_.a_wf;

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
    const double len_se = std::hypot(params_.d_se, params_.a_se);
    const double angle_ESEv = atan2(params_.a_se, params_.d_se);
    const double len_ew = std::hypot(params_.d_ew, params_.a_ew);

    Vector4d P_0_S(0, 0, params_.d_bs, 1);
    Vector3d vec_0_sw = T_07_v.block<3,1>(0,3) - P_0_S.head<3>();
    const double len_sw = vec_0_sw.norm();

    //判断臂长
    const double max_arm_lengeth = len_se + len_ew; //虚拟机械臂的最大臂长，常数不会变
    double current_arm_length = len_sw; //虚拟机械臂的肩腕关节向量的模长
    std::cout << "current_arm_length in IK Ofst = " << current_arm_length << std::endl;
    std::cout << "max_arm_lengeth in IK Ofst = " << max_arm_lengeth << std::endl;
    if (current_arm_length - max_arm_lengeth > EPSILON*1e5 ){
        std::cout << " >>>>>  IK Ofst 超出了工作空间 <<<<<" << std::endl;
        std::cout << "--- IK ofst 超出长度 = " << current_arm_length - max_arm_lengeth << std::endl;
        result.error_code = 10;
        return result;
    }


    const double angle_SEW = acos(
        std::clamp((pow(len_se,2) + pow(len_ew,2) - pow(len_sw,2)) / 
             (2*len_se*len_ew), -1.0, 1.0)
    );

    const double angle_ESW = std::acos(
        std::clamp(
        (std::pow(len_se, 2) + std::pow(len_sw, 2) - std::pow(len_ew, 2)) / 
        (2 * len_se * len_sw),-1.0, 1.0)
        );

    const double gamma2 = atan2(params_.d_ew, -params_.a_ew);

    //根据vec_0_sw_hat.x()的符号判断case_id
    // 测试是否可以用vec_0_sw_hat.x()来判断case_id
    // if (vec_0_sw.x() >=0){
    //     case_id = 0;
    // }
    // else{
    //     case_id = 1;
    // }
    case_id = 0; // 先固定为0，测试效果
    std::cout << " IK Std case_id = " << case_id << std::endl;

        // TODO 分情况讨论 angle_EvSW
    const double angle_EvSW_v1 = angle_ESEv + angle_ESW;
    const double angle_EvSW_v2 = angle_ESEv - angle_ESW;
    
    // Elbow up TODO
    // TODO 分情况讨论 theta4
    double theta4_v1 = angle_ESEv + 3.0/2.0*pi - angle_SEW - gamma2;
    double theta4_v2 = angle_SEW - 0.5*pi + angle_ESEv - gamma2;


    double theta4;
    double angle_EvSW;
    if (case_id ==0){
        theta4 = theta4_v1;
        angle_EvSW = angle_EvSW_v1;
    }
    else {
        theta4 = theta4_v2;
        angle_EvSW = angle_EvSW_v2;
    }    



    // TODO  >>>>> 解分支0 : 肘关节分支 （忽视）<<<<<
    // 此处仅按照肘关节向上的构态来处理
    // 到此处theta4求解完成

    /*
    step 3 :计算参考平面下的前三个关节的初始值
    参考平面为theta3 = 0 的手臂平面，参考08 T-Ro
    */

    //------- 核心改动： 改动参考平面的定义 ----------------
    const Vector3d vec_0_sw_hat = vec_0_sw.normalized();
    // z4_r 需要检查定义
    Vector3d z4_r = (vec_0_sw_hat.x() >= 0) ?  
        vec_0_sw_hat.cross(vec_ref) : 
        vec_ref.cross(vec_0_sw_hat);
    // Vector3d z4_r = vec_0_sw_hat.cross(vec_ref);
    Vector3d z4_r_hat = z4_r.normalized();


    const double gamma = angle_EvSW;
    const double Theta = pi/2 - gamma;
    const Matrix3d R_sup = Matrix3d::Identity() + 
            sin(Theta) * vec_to_skew_matrix(z4_r_hat) +
            (1-cos(Theta)) * vec_to_skew_matrix(z4_r_hat) * vec_to_skew_matrix(z4_r_hat);
    const Vector3d x4_r_hat = R_sup * vec_0_sw_hat;
    const Vector3d y4_r_hat = z4_r_hat.cross(x4_r_hat);

    Matrix3d R_03_orig_calc; // 参考平面的位姿矩阵
    R_03_orig_calc.col(0) =  x4_r_hat;
    R_03_orig_calc.col(1) =  z4_r_hat; 
    R_03_orig_calc.col(2) = -y4_r_hat; 


    //------- 核心改动结束 ----------------

    Vector3d n_ref = vec_0_sw.cross(vec_ref);

    std::vector<double> possible_phis;
    std::vector<double> log_phis;

    // step 4.1 : 计算phi角
    // TODO 解分支2 ： phi 多解 （可能可以消掉）
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

    // 此处记录Aw,Bw,Cw,P,Q,R,Z的值，便于调试
    std::vector<std::tuple<Matrix3d, Matrix3d, Matrix3d, double, double, double, double>> log_ofst_phi_params;
    log_ofst_phi_params.push_back({Aw, Bw, Cw, P, Q, R, Z});

    if (R<Z || std::abs(R-Z)<EPSILON){ // 这个地方的判据开展要修改
        double Delta = pow(P,2)+pow(Q,2)-pow(R,2);
        double temp_item1= 0;
        if (Delta < 0 ) {

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


        // 这个地方增加对arm_angle是否有值进行判断，如果有值进行下面的操作，如果没有值这一步跳过
        if (arm_angle.has_value()) {
            // 根据与arm_angle的差值重新排序
            double diff_1 = std::abs(normalized_phi_1 - arm_angle.value());
            double diff_2 = std::abs(normalized_phi_2 - arm_angle.value());
        
            // 确保 normalized_phi_1 是与 arm_angle 差值更小的那个
            if (diff_2 < diff_1) {
                std::swap(normalized_phi_1, normalized_phi_2);
            }
        }
        else {
            // std::cerr << "IK Ofst 警告： 未输入臂角 arm_angle , 跳过臂角匹配" << std::endl;
        } 

        
        possible_phis.push_back(normalized_phi_1);
        possible_phis.push_back(normalized_phi_2);
        // std::cout << ">>>>> phi from triangle : " << std::endl;
        // std::cout << "phi_1 ： " << normalized_phi_1 << std::endl;
        // std::cout << "phi_2 ： " << normalized_phi_2 << std::endl;
        
        log_phis.push_back(phi_1);
        log_phis.push_back(phi_2);
        log_phis.push_back(normalized_phi_1);
        log_phis.push_back(normalized_phi_2);

    }
    else{
        // std::cout << " P = " << P << std::endl;
        // std::cout << " Q = " << Q << std::endl;
        // std::cout << " R = " << R << std::endl;
        // std::cout << " Z = " << Z << std::endl;
        std::cout << "IK Ofst 臂角无法计算，超出范围" << std::endl;
        result.error_code = 3;
        // log_phis.push_back(0.000);
        // log_phis.push_back(0.000);
        // log_phis.push_back(0.000);
        // log_phis.push_back(0.000);

        return result;
    }

    std::vector<std::tuple<double, double, double, double, Vector3d>> possible_theta_123_and_phis_and_refplane; // theta1,theta2,theta3,phi
    for (const auto& current_phi : possible_phis){
        double angle_phi = current_phi;
        const Matrix3d R_03_final = As * std::sin(angle_phi) + Bs * std::cos(angle_phi) + Cs;

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
            possible_theta_123_and_phis_and_refplane.push_back({theta1, theta2_choice, theta3, current_phi, n_ref});  
        } //二次循环结束
    }//一次循环结束

    std::vector<SerialJoints> possible_serial_joints;
    std::vector<SerialJointsWithPhiAndPlane> possible_serial_joints_and_phis_and_refplane;
    for (const auto current_theta_123_and_phi_and_refplane : possible_theta_123_and_phis_and_refplane) { //一次循环
        // 检查最终 theta2 的约束 (theta2 < 0)
        double current_theta1 = std::get<0>(current_theta_123_and_phi_and_refplane);
        double current_theta2 = std::get<1>(current_theta_123_and_phi_and_refplane);
        double current_theta3 = std::get<2>(current_theta_123_and_phi_and_refplane);
        double current_phi = std::get<3>(current_theta_123_and_phi_and_refplane);
        Vector3d current_n_ref = std::get<4>(current_theta_123_and_phi_and_refplane);


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


        } // 二次循环结束

        // TODO 根据theta7 筛掉一个分支
        for (const auto& current_theta_567_and_phi : possible_theta_567_and_phi){
            double current_theta7 = std::get<2>(current_theta_567_and_phi);
  

            if (std::abs(current_theta7 - theta7.value()) <= EPSILON*100){
                double current_theta5 = std::get<0>(current_theta_567_and_phi);
                double current_theta6 = std::get<1>(current_theta_567_and_phi);
                possible_serial_joints.push_back({current_theta1,current_theta2,current_theta3,
                                                        theta4,current_theta5,current_theta6,current_theta7});
                possible_serial_joints_and_phis_and_refplane.push_back({current_theta1,current_theta2,current_theta3,
                                                        theta4,current_theta5,current_theta6,current_theta7, current_phi, current_n_ref});
            }
            else {
                continue;
            }
        }

    } //一次循环结束






    // 到这里应该计算全部完成，得到了至多8组解
    if (ENABLE_OFST_LOG == true){
        std::cout << "\n\n\n>>>>>>>>>>>>OFST  print area <<<<<<<<<<<<" << std::endl;
        std::cout << "Aw = " << std::endl;
        print_matrix(Aw);
        std::cout << "Bw = " << std::endl;
        print_matrix(Bw);
        std::cout << "Cw = " << std::endl;
        print_matrix(Cw);
        std::cout << " P = " << P << std::endl;
        std::cout << " Q = " << Q << std::endl;
        std::cout << " R = " << R << std::endl;
        std::cout << " Z = " << Z << std::endl;



        std::cout << "possible_serial_joints size is : " <<
                     possible_serial_joints.size() << std::endl;
        std::cout << ">>>>>>>>> all possible solutions <<<<<<<<<<<<<" << std::endl;
        print_vec_of_tuples(possible_serial_joints);
        std::cout << ">>>>>>>>>>>>OFST  print completed <<<<<<<<<<<<\n\n\n" << std::endl;
    }

    result.all_solutions = possible_serial_joints;

    //进行筛选
    // 用于存储符合条件的解的向量
    std::vector<SerialJoints> checked_serial_joints;
    std::vector<SerialJointsWithPhiAndPlane> checked_serial_joints_with_phi_and_plane;
    for (const auto& solution_tuple : possible_serial_joints_and_phis_and_refplane){
        solWithPhiAndPlane sol_temp_with_phi_and_refplane;
        seperate_serial_joints_with_arm_angle_and_plane(sol_temp_with_phi_and_refplane,solution_tuple);
        SerialJoints serial_tuple = sol_temp_with_phi_and_refplane.sol;
        double current_phi = sol_temp_with_phi_and_refplane.arm_angle;
        Vector3d current_refplane = sol_temp_with_phi_and_refplane.plane;
        Vector7d solution_vector = serial_joints_to_vec7d(serial_tuple);
        auto validation_result = validate_solution(solution_vector, params_.joint_limits);
        bool is_valid = std::get<0>(validation_result);
        if (is_valid) {
            // 4. 如果符合条件，加入到 checked_serial_joints
            checked_serial_joints_with_phi_and_plane.push_back(solution_tuple);
        } else {
            // std::cout << " 不符合条件，已过滤。违规关节索引：";
            // 打印违规详情，仅用于调试
            // for (int violation_idx : std::get<2>(validation_result)) {
            //     std::cout << violation_idx << " ";
            // }
            // std::cout << "\n";
        }

    }



    if (checked_serial_joints_with_phi_and_plane.empty()) {
        result.error_code = 9;
        return result;
        std::cout << " IK 计算未找到任何满足条件的解。" << std::endl;
    }

    SerialJoints current_joints_serial; 
    double_array_to_serial_joints(current_joints_array,current_joints_serial);
    std::optional<SerialJointsWithPhiAndPlane> 
        closest_solution_with_phi_and_plane =
            select_closest_ik_solution_with_phi_and_plane(checked_serial_joints_with_phi_and_plane, current_joints_serial);

    if (result.error_code != -1) {result.is_valid = false;}

    if (closest_solution_with_phi_and_plane.has_value()) {


            result.is_valid = true;
            solWithPhiAndPlane temp_result; 
            seperate_serial_joints_with_arm_angle_and_plane(temp_result,closest_solution_with_phi_and_plane.value());
            result.final_sol = temp_result.sol;
            result.arm_angle = temp_result.arm_angle;
            result.n_ref = temp_result.plane;
            result.log_phis = log_phis;
               
            
    } else {
            std::cout << "\n未找到最近的 IK 解（可能在选择过程中发生内部错误）。\n";
            result.is_valid = false;
    }

    // 临时修改区域
    if (SELECT_SINGLE_SOL_LOG == true){
        std::optional<SerialJoints> selected_solution = 
            select_sol_from_possible(SELECT_OFST_ID, possible_serial_joints);
        if (selected_solution.has_value()){
            // std::cout << "\n >>>>> 选择了第 " << SELECT_OFST_ID << " 个解  <<<<< \n";
        } else {
            std::cout << "\n >>>>> 选择解失败，索引超出范围  <<<<< \n";
        }
        result.final_sol = selected_solution.value();
    }

    result.log_ofst_phi_params = log_ofst_phi_params;
    result.vec_sw = vec_0_sw;
    // std::cout << "IK log phi: "  << result.log_phis[0] << "," <<
    //                                 result.log_phis[1] << "," <<
    //                                 result.log_phis[2] << "," <<
    //                                 result.log_phis[3]  <<std::endl;

    return result;
}





IKResult ArmKineComb::calculateIK_vec_ref(
    const Matrix4d& target_pose,
    double current_joints_array[],
    std::optional<double> arm_angle,
    std::optional<double> theta7) // 注意：这里不需要默认参数
{
    IKResult combined_result;
    combined_result.is_valid = false;


    // 先用 Standard 反解求一个初始解,利用std 求出的theta7 传给Ofst， 从而接近指定ofst 的臂角
    IKResult std_ik_result = std_solver_->calculateIK_vec_ref(
        target_pose,
        current_joints_array,
        arm_angle,
        std::nullopt 
    );

    if (std_ik_result.is_valid) {

        // 假设 Offset IK 需要 theta7，而 Standard IK 不需要
        IKResult ofst_ik_result = ofst_solver_->calculateIK_vec_ref(
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




/**
 * @brief 计算最佳臂角以找到一个有效的、且关节跳变最小的IK解。
 * * 逻辑：
 * 1. 尝试使用 current_arm_angle 进行计算。
 * 2. 如果失败，遍历偏差列表，尝试 new_arm_angle = current_arm_angle + deviation。
 * * @param target_pose 目标位姿。
 * @param current_joints_array 当前关节角度数组 (JOINT_COUNT个)。
 * @param current_arm_angle 初始尝试的臂角值。
 * @param arm_angle_deviation_list 臂角的偏差值列表 (Delta Arm Angle)，默认值: [-0.5, -0.25, 0, 0.25, 0.5]。
 * @param offset_ref 允许的最大关节跳变阈值。
 * @return 最佳的IKResult。如果所有尝试都失败，返回一个 is_valid=false 的结果。
 */
IKResult ArmKineComb::cal_IK_feasible_armAngle_vec_ref(
    const Matrix4d& target_pose,
    double current_joints_array[],
    double current_arm_angle,
    const std::vector<double>& arm_angle_deviation_list , // 默认臂角偏差列表
    double offset_ref  // 默认最大关节跳变阈值
) {
    IKResult feasible_res;
    feasible_res.is_valid = false;
    std::vector<IKResult> feasible_solutions; // 存储所有满足跳变要求的解
    int cnt=0;

    
    // --- 步骤 1: 尝试初始的 current_arm_angle ---
    
    double initial_arm_angle = current_arm_angle;
    
    IKResult initial_result = ArmKineComb::calculateIK_vec_ref(
        target_pose,
        current_joints_array,
        initial_arm_angle, // 初始臂角
        std::nullopt
    );

    // 检查偏差解是否有效，以及检查偏差距离
    if (is_solution_acceptable(initial_result, current_joints_array, offset_ref)) {
        // 初始臂角计算成功且解符合要求，直接返回
        // std::cout << "Optimal IK solution found with initial Arm Angle: " << current_arm_angle << std::endl;
        return initial_result;
    }
    
    // --- 步骤 2: 遍历偏差列表，尝试 new_arm_angle = current_arm_angle + deviation ---
    std::vector<double> tested_angles;
    tested_angles.push_back(initial_arm_angle);
    // 检查偏差列表是否为空
    if (arm_angle_deviation_list.empty()) {
        std::cerr << "Warning: Initial arm angle failed and deviation list is empty." << std::endl;
        return IKResult{}; // 返回一个无效结果
    }

    // 遍历臂角偏差列表
    for (double deviation : arm_angle_deviation_list) {
        cnt ++;
        // 跳过偏差为0的情况，因为已经在步骤1中计算过了
        if (std::abs(deviation) < 1e-6) { // 使用一个小的阈值判断是否接近0
            continue; 
        }
        
        // 计算新的臂角
        double new_arm_angle = current_arm_angle + deviation;

        // double clamp_arm_angle = std::clamp(new_arm_angle, -100.0/180.0*M_PI, 0.0); 
        double clamp_arm_angle = new_arm_angle;
        
        // 检查该臂角是否已经计算过（例如多次钳位到边缘值）
        bool is_calculated = false;
        for (double angle : tested_angles) {
            if (std::abs(angle - clamp_arm_angle) < 1e-6) {
                is_calculated = true;
                break;
            }
        }

        if (is_calculated) {
            continue;
        }
        tested_angles.push_back(clamp_arm_angle);




        IKResult result = calculateIK(
            target_pose,
            current_joints_array,
            clamp_arm_angle,
            std::nullopt
        );

        if (is_solution_acceptable(result, current_joints_array, offset_ref)) {
            result.arm_angle = clamp_arm_angle;
            feasible_solutions.push_back(result);
        }
    }


        // // 调用原始的 IK 计算函数
        // IKResult result = calculateIK_vec_ref(
        //     target_pose,
        //     current_joints_array,
        //     new_arm_angle, // 使用计算出的新臂角
        //     std::nullopt
        // );

        // // 检查 IK 解是否有效且符合跳变要求
        // if (is_solution_acceptable(result, current_joints_array, offset_ref)) {
        //     // 找到一个符合所有条件的解，立即返回
        //     // std::cout << "Optimal IK solution found with deviated Arm Angle: " << new_arm_angle 
        //     //           << " (Deviation: " << deviation << ")" << std::endl;
        //     std::cout << "寻找次数： cnt = " << cnt << std::endl;
        //     result.arm_angle = new_arm_angle; // 记录使用的臂角
        //     return result;
        // }
    std::cout << "寻找次数：cnt = " << cnt << std::endl;
    std::cout << "找到满足跳变要求的解数量：" << feasible_solutions.size() << std::endl;

    // --- 步骤 3: 从所有可行解中选择距离最近的一个 ---
    if (feasible_solutions.empty()) {
        std::cerr << "Warning: No valid IK solution found within deviation limit for any arm angle attempt." << std::endl;
        return IKResult{}; // 返回一个无效结果
    }

    // 如果只有一个解，直接返回
    if (feasible_solutions.size() == 1) {
        return feasible_solutions[0];
    }

    // 多个解的情况，选择距离最近的一个
    IKResult best_solution;
    double min_distance = std::numeric_limits<double>::max();
    // double min_Delta = std::numeric_limits<double>::max();

    for (const auto& solution : feasible_solutions) {
        double distance = calculate_max_joint_deviation(solution.final_sol, current_joints_array);
        // double current_arm_angle = solution.arm_angle;
        // double Delta = distance;
    
        if (distance < min_distance) {
            min_distance = distance;
            best_solution = solution;
        }
        // if (Delta < min_Delta) {
        //     min_Delta = Delta;
        //     best_solution = solution;
        // }
        
        // std::cout << "臂角 " << solution.arm_angle << " 的距离: " << distance << std::endl;
    }
    

    return best_solution;    

}



