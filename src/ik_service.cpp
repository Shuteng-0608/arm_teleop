#include <ros/ros.h>
#include <Eigen/Dense>
#include "arm_teleop/ArmIK.h"  // 替换为实际服务头文件
// #include "/home/pangu/pangu/src/arm_teleop/lib/kInematics_advanced/Arm_kinematics_cal_cpp-test_refactor/include/arm_kinematics/arm_kinematics.h"
// #include "tools.h"
#include "arm_kinematics/arm_kinematics.h"
#include "arm_kinematics/redundancy/redundancy_configuration_selector.h"
#include "arm_kinematics/redundancy/redundancy_selector_config.h"
#include <ros/package.h>
#include <cmath>
#include <stdexcept>
#include <string>
using namespace arm_kinematics;

#ifndef ARM_KINEMATICS_SELECTOR_CONFIG_PATH
#define ARM_KINEMATICS_SELECTOR_CONFIG_PATH \
    "/usr/local/share/ArmKinematics/config/redundancy_selector.yaml"
#endif

namespace {

using arm_kinematics::redundancy::RedundancySelectorSettings;
using arm_kinematics::redundancy::SelectorInput;
using arm_kinematics::redundancy::SelectorRuntimeMode;
using arm_kinematics::redundancy::SrsBranch;
using arm_kinematics::redundancy::TemporalWristState;

const char* kA1Method = "A1_minimum_jv";
const char* kRefinedMethod = "minimum_sufficient_continuity_refined";

RedundancySelectorSettings selectorSettingsForMethod(
    const std::string& selector_config_path,
    const std::string& method) {
    RedundancySelectorSettings settings =
        arm_kinematics::redundancy::loadRedundancySelectorSettings(
            selector_config_path);
    settings.runtime_mode = SelectorRuntimeMode::kExecution;

    if (method == kA1Method) {
        settings.enable_wrist_objective = false;
        settings.enable_guard = false;
        settings.minimum_sufficient_budget.enabled = false;
        settings.minimum_sufficient_budget.continuity_enabled = false;
        settings.minimum_sufficient_budget.offset_refinement_enabled = false;
        settings.minimum_sufficient_budget.stateful.enabled = false;
        settings.minimum_sufficient_budget.offset_aware.enabled = false;
        settings.continuity_first_wrist.enabled = false;
        settings.temporal_wrist_policy.enabled = false;
        return settings;
    }

    if (method == kRefinedMethod) {
        settings.enable_wrist_objective = true;
        settings.enable_guard = true;
        settings.minimum_wrist_gap_improvement = 0.0005;

        auto& sufficient = settings.minimum_sufficient_budget;
        sufficient.enabled = true;
        sufficient.continuity_enabled = true;
        sufficient.redundant_step_cap = 0.15;
        sufficient.entry_redundant_step_cap = 0.075;
        sufficient.sufficient_jv_tie_tolerance = 0.005;
        sufficient.reversal_deadband_rad_s = 0.01;
        sufficient.offset_refinement_enabled = true;
        sufficient.offset_refinement_phi_tolerance_rad = 1.0e-4;
        sufficient.offset_refinement_gain_tolerance = 1.0e-5;
        sufficient.offset_refinement_maximum_iterations = 8;
        sufficient.stateful.enabled = false;
        sufficient.offset_aware.enabled = false;

        settings.continuity_first_wrist.enabled = false;
        auto& temporal = settings.temporal_wrist_policy;
        temporal.enabled = true;
        temporal.enter_gain = 0.0005;
        temporal.enter_consecutive_cycles = 3;
        temporal.exit_consecutive_cycles = 1;
        temporal.cooldown_cycles = 3;
        temporal.near_opt_gap_tolerance = 0.00025;
        temporal.maintain_gain_floor = 1.0e-9;
        temporal.redundancy_bias_deadband_rad = 0.0002;
        temporal.lock_branch_while_improving = true;
        temporal.lock_bias_direction_while_improving = true;
        return settings;
    }

    throw std::invalid_argument(
        "Invalid redundancy selector method: " + method);
}

SrsBranch inferBranch(const Vector7d& joints) {
    const std::size_t branch_index =
        (std::cos(joints[1]) >= 0.0 ? 0U : 2U) +
        (std::cos(joints[5]) >= 0.0 ? 0U : 1U);
    return arm_kinematics::redundancy::branchFromIndex(branch_index);
}

Vector7d wrappedDifference(const Vector7d& current, const Vector7d& previous) {
    Vector7d difference = Vector7d::Zero();
    for (int joint = 0; joint < difference.size(); ++joint) {
        difference[joint] = std::atan2(
            std::sin(current[joint] - previous[joint]),
            std::cos(current[joint] - previous[joint]));
    }
    return difference;
}

struct SelectorHistory {
    bool valid = false;
    Matrix4d accepted_target_pose = Matrix4d::Identity();
    Vector7d previous_joints = Vector7d::Zero();
    bool previous_standard_joints_valid = false;
    Vector7d previous_standard_joints = Vector7d::Zero();
    bool previous_baseline_standard_joints_valid = false;
    Vector7d previous_baseline_standard_joints = Vector7d::Zero();
    bool previous_standard_redundancy_displacement_valid = false;
    Vector7d previous_standard_redundancy_displacement = Vector7d::Zero();
    bool previous_baseline_joints_valid = false;
    Vector7d previous_baseline_joints = Vector7d::Zero();
    bool previous_redundancy_displacement_valid = false;
    Vector7d previous_redundancy_displacement = Vector7d::Zero();
    bool previous_redundancy_velocity_valid = false;
    Vector7d previous_redundancy_velocity = Vector7d::Zero();
    double previous_arm_angle = 0.0;
    SrsBranch previous_branch = SrsBranch::kCos2PositiveCos6Positive;
    bool previous_guard_active = false;
    bool previous_command_velocity_valid = false;
    Vector7d previous_command_velocity = Vector7d::Zero();
    TemporalWristState temporal_wrist_state;
};

const char* selectionStatusName(
    arm_kinematics::redundancy::SelectionStatus status) {
    using arm_kinematics::redundancy::SelectionStatus;
    switch (status) {
        case SelectionStatus::kSelected:
            return "selected";
        case SelectionStatus::kFallbackBaseline:
            return "fallback_baseline";
        case SelectionStatus::kHoldPrevious:
            return "hold_previous";
        case SelectionStatus::kInvalidInput:
            return "invalid_input";
        case SelectionStatus::kNumericalFailure:
            return "numerical_failure";
    }
    return "unknown";
}

}  // namespace

class ArmKinematicsServer {

private:
    
    ros::ServiceServer service_;
    CombinedSolver solver_;
    arm_kinematics::redundancy::RedundancyConfigurationSelector
        redundancy_selector_a1_;
    arm_kinematics::redundancy::RedundancyConfigurationSelector
        redundancy_selector_refined_;
    SelectorHistory selector_history_a1_;
    SelectorHistory selector_history_refined_;

public:
    ArmKinematicsServer(
        ros::NodeHandle& nh,
        const std::string& config_path,
        const std::string& selector_config_path)
        : solver_(config_path),
          redundancy_selector_a1_(
              config_path,
              selectorSettingsForMethod(selector_config_path, kA1Method)),
          redundancy_selector_refined_(
              config_path,
              selectorSettingsForMethod(selector_config_path, kRefinedMethod)) {
        try {      
            ROS_INFO("[right_arm_teleop] Inverse Kinematics solvers initialized successfully");
        } catch (const std::exception& e) {
            ROS_FATAL("[right_arm_teleop] Solver initialization failed: %s", e.what());
            throw;
        }

        // 注册服务
        service_ = nh.advertiseService("/arm_teleop/right_arm_ik_srv", &ArmKinematicsServer::handleRequest, this);
    }

    void resetSelectorHistory(
        SelectorHistory& history,
        const Matrix4d& target_pose,
        const double* initial_joints,
        double arm_angle) {
        history = SelectorHistory();
        history.valid = true;
        history.accepted_target_pose = target_pose;
        for (int joint = 0; joint < history.previous_joints.size(); ++joint) {
            history.previous_joints[joint] = initial_joints[joint];
        }
        history.previous_arm_angle = arm_angle;
        history.previous_branch = inferBranch(history.previous_joints);
    }

    void prepareSelectorHistory(
        SelectorHistory& history,
        const Matrix4d& target_pose,
        const double* initial_joints,
        double arm_angle) {
        bool reset = !history.valid;
        if (!reset) {
            for (int joint = 0; joint < history.previous_joints.size(); ++joint) {
                if (std::abs(initial_joints[joint] -
                             history.previous_joints[joint]) > 0.01) {
                    reset = true;
                    break;
                }
            }
        }
        if (reset) {
            resetSelectorHistory(history, target_pose, initial_joints, arm_angle);
        }
    }

    SelectorInput makeSelectorInput(
        const SelectorHistory& history,
        const Matrix4d& target_pose) const {
        SelectorInput input;
        input.target_pose = target_pose;
        input.accepted_target_pose = history.accepted_target_pose;
        input.previous_joints = history.previous_joints;
        input.previous_standard_joints_valid =
            history.previous_standard_joints_valid;
        input.previous_standard_joints = history.previous_standard_joints;
        input.previous_baseline_standard_joints_valid =
            history.previous_baseline_standard_joints_valid;
        input.previous_baseline_standard_joints =
            history.previous_baseline_standard_joints;
        input.previous_standard_redundancy_displacement_valid =
            history.previous_standard_redundancy_displacement_valid;
        input.previous_standard_redundancy_displacement =
            history.previous_standard_redundancy_displacement;
        input.previous_baseline_joints_valid =
            history.previous_baseline_joints_valid;
        input.previous_baseline_joints = history.previous_baseline_joints;
        input.previous_redundancy_displacement_valid =
            history.previous_redundancy_displacement_valid;
        input.previous_redundancy_displacement =
            history.previous_redundancy_displacement;
        input.previous_redundancy_velocity_valid =
            history.previous_redundancy_velocity_valid;
        input.previous_redundancy_velocity = history.previous_redundancy_velocity;
        input.previous_arm_angle = history.previous_arm_angle;
        input.previous_branch = history.previous_branch;
        input.previous_guard_active = history.previous_guard_active;
        input.history_valid = history.valid;
        input.source_period_s = 1.0 / 30.0;
        input.previous_command_velocity_valid =
            history.previous_command_velocity_valid;
        input.previous_command_velocity = history.previous_command_velocity;
        input.temporal_wrist_state = history.temporal_wrist_state;
        return input;
    }

    void acceptSelectorResult(
        SelectorHistory& history,
        const Matrix4d& target_pose,
        const arm_kinematics::redundancy::SelectorResult& result) {
        // Temporal policy state is acknowledged independently of command
        // history; the selector may advance or reset it on a HOLD result.
        if (result.updates_temporal_state_if_acknowledged) {
            history.temporal_wrist_state = result.next_temporal_wrist_state;
        }
        if (!result.has_executable_solution ||
            !result.updates_history_if_accepted) {
            return;
        }

        const Vector7d previous_joints = history.previous_joints;
        const Vector7d previous_redundancy =
            history.previous_redundancy_displacement;
        const bool previous_redundancy_valid =
            history.previous_redundancy_displacement_valid;
        history.previous_joints = result.executable_joints;
        history.previous_standard_joints = result.selected_standard_joints;
        history.previous_standard_joints_valid = true;
        history.previous_arm_angle = result.selected_arm_angle;
        history.previous_branch = result.standard_branch;
        history.previous_guard_active = result.guard_active;
        history.accepted_target_pose = target_pose;

        if (result.diagnostics.final_baseline_joints_valid) {
            history.previous_baseline_joints =
                result.diagnostics.baseline_joints;
            history.previous_baseline_joints_valid = true;
            history.previous_redundancy_displacement =
                wrappedDifference(
                    result.executable_joints,
                    result.diagnostics.baseline_joints);
            history.previous_redundancy_displacement_valid = true;
            if (previous_redundancy_valid) {
                history.previous_redundancy_velocity =
                    (history.previous_redundancy_displacement -
                     previous_redundancy) / (1.0 / 30.0);
                history.previous_redundancy_velocity_valid = true;
            } else {
                history.previous_redundancy_velocity.setZero();
                history.previous_redundancy_velocity_valid = false;
            }
        }
        if (result.diagnostics.final_baseline_standard_joints_valid) {
            history.previous_baseline_standard_joints =
                result.diagnostics.baseline_standard_joints;
            history.previous_baseline_standard_joints_valid = true;
            history.previous_standard_redundancy_displacement =
                wrappedDifference(
                    result.selected_standard_joints,
                    result.diagnostics.baseline_standard_joints);
            history.previous_standard_redundancy_displacement_valid =
                true;
        }
        history.previous_command_velocity =
            result.diagnostics.command_velocity_rad_s;
        history.previous_command_velocity_valid = true;
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
        res.success = false;
        res.search_cnt = -1;
        res.new_arm_angle = req.current_arm_angle;
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

        const bool legacy_selector_alias = req.method == "redundancy_selector";
        const std::string selector_method = legacy_selector_alias
            ? kA1Method : req.method;
        if (selector_method == kA1Method || selector_method == kRefinedMethod) {
            SelectorHistory& selector_history = selector_method == kA1Method
                ? selector_history_a1_ : selector_history_refined_;
            prepareSelectorHistory(
                selector_history, Tee, init_joints_array, req.current_arm_angle);
            const SelectorInput selector_input = makeSelectorInput(
                selector_history, Tee);
            const auto selector_result = selector_method == kA1Method
                ? redundancy_selector_a1_.select(selector_input)
                : redundancy_selector_refined_.select(selector_input);
            acceptSelectorResult(selector_history, Tee, selector_result);
            res.success = selector_result.has_executable_solution;
            res.message = std::string(legacy_selector_alias
                                          ? "redundancy_selector:"
                                          : selector_method + ":") +
                selectionStatusName(selector_result.status);
            if (res.success) {
                for (int joint = 0; joint < 7; ++joint) {
                    res.solution[joint] =
                        selector_result.executable_joints[joint];
                }
                res.new_arm_angle = selector_result.selected_arm_angle;
            }
            return true;
        }

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
                throw std::invalid_argument(
                    "Invalid method. Valid options: A1_minimum_jv, "
                    "minimum_sufficient_continuity_refined, feasible_ref, "
                    "feasible_std, optimal_ref, optimal_std");
            }
        } catch (const std::exception& e) {
            res.success = false;
            res.message = "IK calculation failed: " + std::string(e.what());
            res.search_cnt = -1;  // ✅ 异常时明确标记为-1
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
            res.search_cnt= ik_res.search_cnt;  // ✅ 新增这一行
            // 验证结果
            bool verified = verifyIKSolution(solver_, Tee, ik_res);
            std::cout << "  FK验证: " << (verified ? "通过" : "失败") << "\n\n";
            res.search_cnt = ik_res.search_cnt;  // ✅ 传递真实搜索次数
            res.success = true;
            res.message = "Success! ovo";
        } else {
            res.success = false;
            res.message = "Failed qwq";
            res.search_cnt = -1;  // ✅ 失败时明确标记为-1
        }
        return true; // 服务处理成功
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "right_arm_kinematics_server");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    // 获取配置文件路径
    // std::string config_path;
    // config_path = "~arm_teleop/config/kinematics_params.yaml";
    const std::string default_config_path =
        ros::package::getPath("arm_teleop") + "/config/kinematics_params.yaml";
    std::string config_path;
    private_nh.param<std::string>(
        "kinematics_config_path", config_path, default_config_path);
    std::string selector_config_path;
    private_nh.param<std::string>(
        "selector_config_path",
        selector_config_path,
        ARM_KINEMATICS_SELECTOR_CONFIG_PATH);
    // config_path = nh.param<std::string>("config_path", "config/arm_kinematics.yaml");
    ROS_INFO("[right_arm_teleop] Using config file: %s", config_path.c_str());
    ROS_INFO(
        "[right_arm_teleop] Using selector config file: %s",
        selector_config_path.c_str());

    try {
        ArmKinematicsServer server(nh, config_path, selector_config_path);
        ROS_INFO("[right_arm_teleop] Arm IK Service ready");
        ros::spin();
    } catch (const std::exception& e) {
        ROS_FATAL("[right_arm_teleop] Service initialization failed: %s", e.what());
        return 2;
    }
    
    return 0;
}
