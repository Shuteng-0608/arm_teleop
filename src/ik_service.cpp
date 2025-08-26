#include <ros/ros.h>
#include <Eigen/Dense>
#include "arm_teleop/ArmIK.h"  // 替换为实际服务头文件
#include "kinematics_new.h"
#include "tools.h"
#include <ros/package.h>

class ArmKinematicsServer {
private:
    ros::ServiceServer service_;
    std::unique_ptr<ArmKineStd> std_solver;
    std::unique_ptr<ArmKineOfst> ofst_solver;
    std::unique_ptr<ArmKineComb> comb_solver;

public:
    ArmKinematicsServer(ros::NodeHandle& nh, const std::string& config_path) {
        // 初始化三种求解器
        try {
            std_solver = std::make_unique<ArmKineStd>(config_path);
            ofst_solver = std::make_unique<ArmKineOfst>(config_path);
            
            auto std_for_comb = std::make_unique<ArmKineStd>(config_path);
            auto ofst_for_comb = std::make_unique<ArmKineOfst>(config_path);
            comb_solver = std::make_unique<ArmKineComb>(std::move(std_for_comb), std::move(ofst_for_comb));
                
            ROS_INFO("Kinematics solvers initialized successfully");
        } catch (const std::exception& e) {
            ROS_FATAL("Solver initialization failed: %s", e.what());
            throw;
        }

        // 注册服务
        service_ = nh.advertiseService("arm_ik_service", &ArmKinematicsServer::handleRequest, this);
    }

    bool handleRequest(arm_teleop::ArmIK::Request& req, arm_teleop::ArmIK::Response& res) {
        // 转换初始关节角
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
        Tee.block<3,3>(0,0) = q.normalized().toRotationMatrix();

        // 选择求解方法
        IKResult ik_res;
        try {
            if (req.method == "std") {
                ik_res = std_solver->calculateIK(Tee, init_joints_array, 0.1, std::nullopt);
            } else if (req.method == "ofst") {
                ik_res = ofst_solver->calculateIK(Tee, init_joints_array, std::nullopt, 0.14);
            } else if (req.method == "comb") {
                ik_res = comb_solver->calculateIK(Tee, init_joints_array, 0.1, std::nullopt);
            } else {
                throw std::invalid_argument("Invalid method. Valid options: std, ofst, comb");
            }
        } catch (const std::exception& e) {
            res.success = false;
            res.message = "IK calculation failed: " + std::string(e.what());
            return true;
        }

        // 处理结果
        if (ik_res.is_valid) {
            // 返回关节角度
            res.solution[0] = std::get<0>(ik_res.final_sol);
            res.solution[1] = std::get<1>(ik_res.final_sol);
            res.solution[2] = std::get<2>(ik_res.final_sol);
            res.solution[3] = std::get<3>(ik_res.final_sol);
            res.solution[4] = std::get<4>(ik_res.final_sol);
            res.solution[5] = std::get<5>(ik_res.final_sol);
            res.solution[6] = std::get<6>(ik_res.final_sol);
            
            // 验证结果
            Vector7d sol_vec = serial_joints_to_vec7d(ik_res.final_sol);
            FKResult fk_res;
            if (req.method == "std") fk_res = std_solver->calculateFK(sol_vec);
            else if (req.method == "ofst") fk_res = ofst_solver->calculateFK(sol_vec);
            else fk_res = comb_solver->calculateFK(sol_vec);
            
            PoseComparisonResult cmp = compare_poses_detailed(Tee, fk_res.T_08, 1e-5, 1e-4);
            
            res.success = true;
            res.message = "Success! Trans error: " + std::to_string(cmp.translation_error) +
                        "m, Rot error: " + std::to_string(cmp.rotation_error) + "rad";
        } else {
            res.success = false;
    
            switch(ik_res.error_code) {
                case -1:
                    res.message = "No valid solution found (general failure)";
                    break;
                case -2:
                    res.message = "No valid solution found (elbow position)";
                    break;
                case -3:
                    res.message = "No valid solution found (wrist position)";
                    break;
                case -4:
                    res.message = "No valid solution found (singularity)";
                    break;
                case -5:
                    res.message = "No valid solution found (joint limit violation)";
                    break;
                default:
                    res.message = "No valid IK solution found (error code: " + 
                                std::to_string(ik_res.error_code) + ")";
            }
        
            return true;
        }
        return true; // 服务处理成功
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_kinematics_server");
    ros::NodeHandle nh;
    
    // 获取配置文件路径
    // std::string config_path;
    // config_path = "~arm_teleop/config/kinematics_params.yaml";
    std::string config_path = ros::package::getPath("arm_teleop") + "/config/kinematics_params.yaml";
    // config_path = nh.param<std::string>("config_path", "config/arm_kinematics.yaml");
    ROS_INFO("Using config file: %s", config_path.c_str());

    try {
        ArmKinematicsServer server(nh, config_path);
        ROS_INFO("Arm IK Service ready");
        ros::spin();
    } catch (const std::exception& e) {
        ROS_FATAL("Service initialization failed: %s", e.what());
        return 2;
    }
    
    return 0;
}