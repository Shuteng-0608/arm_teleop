#include <ros/ros.h>
#include <Eigen/Dense>
#include "arm_teleop/ArmIK.h"  // 替换为实际服务头文件
// #include "/home/pangu/pangu/src/arm_teleop/lib/kInematics_advanced/Arm_kinematics_cal_cpp-test_refactor/include/arm_kinematics/arm_kinematics.h"
// #include "tools.h"
#include "arm_kinematics/arm_kinematics.h"
#include <ros/package.h>
using namespace arm_kinematics;

class ArmKinematicsServer {

private:
    
    ros::ServiceServer service_;
    CombinedSolver solver_;

public:
    ArmKinematicsServer(ros::NodeHandle& nh, const std::string& config_path) : solver_(config_path) {
        try {      
            ROS_INFO("[right_arm_teleop] Inverse Kinematics solvers initialized successfully");
        } catch (const std::exception& e) {
            ROS_FATAL("[right_arm_teleop] Solver initialization failed: %s", e.what());
            throw;
        }

        // 注册服务
        service_ = nh.advertiseService("/arm_teleop/right_arm_ik_srv", &ArmKinematicsServer::handleRequest, this);
    }

    /**
     * @brief 验证IK解的精度
     */
    bool verifyIKSolution(CombinedSolver& solver, 
                        const Matrix4d& target_pose, 
                        const IKResult& result,
                        double trans_tol = 1e-5,
                        double rot_tol = 1e-4) {
        if (!result.is_valid) {
            return false;
        }
        
        Vector7d sol_vector = jointAnglesToVector(result.final_solution);
        FKResult fk_result = solver.computeFK(sol_vector);
        
        PoseComparisonResult comparison = comparePosesDetailed(
            target_pose, fk_result.T_08, trans_tol, rot_tol);
        
        return comparison.is_approximate;
    }

    bool handleRequest(arm_teleop::ArmIK::Request& req, arm_teleop::ArmIK::Response& res) {
        // 转换初始关节角
        // ROS_INFO("Received IK request using method: %s", req.method.c_str());
        // ROS_INFO("Current arm angle: %f", req.current_arm_angle);
        // ROS_INFO("Initial joints: [%f, %f, %f, %f, %f, %f, %f]",
        //         req.init_joints[0], req.init_joints[1], req.init_joints[2],
        //         req.init_joints[3], req.init_joints[4], req.init_joints[5],
        //         req.init_joints[6]);
        // ROS_INFO("Target pose position: [%f, %f, %f]", 
        //         req.target_pose.position.x,
        //         req.target_pose.position.y,
        //         req.target_pose.position.z);
        // ROS_INFO("Target pose orientation: [%f, %f, %f, %f]", 
        //         req.target_pose.orientation.w,
        //         req.target_pose.orientation.x, 
        //         req.target_pose.orientation.y,
        //         req.target_pose.orientation.z);
        double init_joints_array[7];
        for (int i = 0; i < 7; ++i) {
            init_joints_array[i] = req.init_joints[i];
        }

        // 转换目标位姿为Eigen矩阵
        Eigen::Matrix4d Tee = Eigen::Matrix4d::Identity();
        Tee.block<3,1>(0,3) = Eigen::Vector3d(
            req.target_pose.position.x,
            req.target_pose.position.y,
            req.target_pose.position.z);
        
        Eigen::Quaterniond q(
            req.target_pose.orientation.w,
            req.target_pose.orientation.x,
            req.target_pose.orientation.y,
            req.target_pose.orientation.z);
        // ROS_INFO("Target quaternion: [w: %f, x: %f, y: %f, z: %f]", 
        //         q.w(), q.x(), q.y(), q.z());
        Tee.block<3,3>(0,0) = q.normalized().toRotationMatrix();
        // ROS_INFO("Transformation Matrix Tee:");
        // for (int i = 0; i < 4; ++i) {
        //     ROS_INFO("[%6.3f, %6.3f, %6.3f, %6.3f]", 
        //             Tee(i,0), Tee(i,1), Tee(i,2), Tee(i,3));
        // }

        // 选择求解方法
        IKResult ik_res;
        
        try {
            ROS_INFO("Calculating IK...");
            if (req.method == "feasible_ref") {
                ik_res = solver_.computeIKWithSearch(Tee, 
                                                    init_joints_array, 
                                                    req.current_arm_angle,
                                                    req.offset_list,
                                                    req.offset_refer,
                                                    IKSolveMode::kFeasible,
                                                    IKMethodType::kVecRef);
            } else if (req.method == "feasible_std") {
                ik_res = solver_.computeIKWithSearch(Tee, 
                                                    init_joints_array, 
                                                    req.current_arm_angle,
                                                    req.offset_list,
                                                    req.offset_refer,
                                                    IKSolveMode::kFeasible,
                                                    IKMethodType::kStandard);
            } else if (req.method == "optimal_ref") {
                ik_res = solver_.computeIKWithSearch(Tee, 
                                                    init_joints_array, 
                                                    req.current_arm_angle,
                                                    req.offset_list,
                                                    req.offset_refer,
                                                    IKSolveMode::kOptimal,
                                                    IKMethodType::kVecRef);
            } else if (req.method == "optimal_std") {
                ik_res = solver_.computeIKWithSearch(Tee, 
                                                    init_joints_array, 
                                                    req.current_arm_angle,
                                                    req.offset_list,
                                                    req.offset_refer,
                                                    IKSolveMode::kOptimal,
                                                    IKMethodType::kStandard);
            } else {
                throw std::invalid_argument("Invalid method. Valid options: std, ofst, comb, feasible");
            }
        } catch (const std::exception& e) {
            res.success = false;
            res.message = "IK calculation failed: " + std::string(e.what());
            return true;
        }

        // 处理结果
        if (ik_res.is_valid) {
            // 返回关节角度
            res.solution[0] = std::get<0>(ik_res.final_solution);
            res.solution[1] = std::get<1>(ik_res.final_solution);
            res.solution[2] = std::get<2>(ik_res.final_solution);
            res.solution[3] = std::get<3>(ik_res.final_solution);
            res.solution[4] = std::get<4>(ik_res.final_solution);
            res.solution[5] = std::get<5>(ik_res.final_solution);
            res.solution[6] = std::get<6>(ik_res.final_solution);
            res.new_arm_angle = ik_res.arm_angle;
            
            // 验证结果
            bool verified = verifyIKSolution(solver_, Tee, ik_res);
            std::cout << "  FK验证: " << (verified ? "通过" : "失败") << "\n\n";
            
            res.success = true;
            res.message = "Success! ovo";
        } else {
            res.success = false;
            res.message = "Failed qwq";
        
        }
        return true; // 服务处理成功
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "right_arm_kinematics_server");
    ros::NodeHandle nh;
    
    // 获取配置文件路径
    // std::string config_path;
    // config_path = "~arm_teleop/config/kinematics_params.yaml";
    std::string config_path = ros::package::getPath("arm_teleop") + "/config/kinematics_params.yaml";
    // config_path = nh.param<std::string>("config_path", "config/arm_kinematics.yaml");
    ROS_INFO("[right_arm_teleop] Using config file: %s", config_path.c_str());

    try {
        ArmKinematicsServer server(nh, config_path);
        ROS_INFO("[right_arm_teleop] Arm IK Service ready");
        ros::spin();
    } catch (const std::exception& e) {
        ROS_FATAL("[right_arm_teleop] Service initialization failed: %s", e.what());
        return 2;
    }
    
    return 0;
}