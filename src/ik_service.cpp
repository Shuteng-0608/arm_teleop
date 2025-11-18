#include <ros/ros.h>
#include <Eigen/Dense>
#include "arm_teleop/ArmIK.h"  // 替换为实际服务头文件
#include "kinematics_new.h"
#include "tools.h"
#include <ros/package.h>

class ArmKinematicsServer {
private:
    ros::ServiceServer service_;
    std::shared_ptr<ArmKineStd> std_solver;
    std::shared_ptr<ArmKineOfst> ofst_solver;
    std::shared_ptr<ArmKineComb> comb_solver;

public:
    ArmKinematicsServer(ros::NodeHandle& nh, const std::string& config_path) {
        // 初始化三种求解器
        try {
            std_solver = std::make_shared<ArmKineStd>(config_path);
            ofst_solver = std::make_shared<ArmKineOfst>(config_path);
            
            auto std_for_comb = std::make_shared<ArmKineStd>(config_path);
            auto ofst_for_comb = std::make_shared<ArmKineOfst>(config_path);
            comb_solver = std::make_shared<ArmKineComb>((std_for_comb), (ofst_for_comb));
                
            ROS_INFO("[right_arm_teleop] Inverse Kinematics solvers initialized successfully");
        } catch (const std::exception& e) {
            ROS_FATAL("[right_arm_teleop] Solver initialization failed: %s", e.what());
            throw;
        }

        // 注册服务
        service_ = nh.advertiseService("/arm_teleop/right_arm_ik_srv", &ArmKinematicsServer::handleRequest, this);
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
            if (req.method == "std") {
                ik_res = std_solver->calculateIK(Tee, init_joints_array, 0.1, std::nullopt);
            } else if (req.method == "ofst") {
                ik_res = ofst_solver->calculateIK(Tee, init_joints_array, std::nullopt, 0.14);
            } else if (req.method == "comb") {
                ik_res = comb_solver->calculateIK(Tee, init_joints_array, 0.1, std::nullopt);
            } else if (req.method == "feasible") {
                ik_res = comb_solver->cal_IK_feasible_armAngle(Tee, init_joints_array, req.current_arm_angle, req.offset_list, req.offset_refer);
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
            res.solution[0] = std::get<0>(ik_res.final_sol);
            res.solution[1] = std::get<1>(ik_res.final_sol);
            res.solution[2] = std::get<2>(ik_res.final_sol);
            res.solution[3] = std::get<3>(ik_res.final_sol);
            res.solution[4] = std::get<4>(ik_res.final_sol);
            res.solution[5] = std::get<5>(ik_res.final_sol);
            res.solution[6] = std::get<6>(ik_res.final_sol);
            res.new_arm_angle = ik_res.arm_angle;
            
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