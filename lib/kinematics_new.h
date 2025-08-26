#ifndef KINEMATICS_NEW_H
#define KINEMATICS_NEW_H

// kinematics.h
#include <Eigen/Dense>  
#include <vector>
#include <tuple>
#include <iostream>
#include <iomanip>  // 添加setprecision需要
#include <ceres/ceres.h> // 这个要安装和配置
#include <optional>     // For std::optional (C++17+)
#include <yaml-cpp/yaml.h>
// #include <tools.h>

// 类型别名
using Matrix3d = Eigen::Matrix3d;
using Matrix4d = Eigen::Matrix4d;
using Vector3d = Eigen::Vector3d;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Vector4d = Eigen::Vector4d;

using SerialJoints = std::tuple<double,double,double,double,double,double,double>;
using SerialJointsWithPhi = std::tuple<double,double,double,double,double,double,double,double>;
using JointsArray = std::array<double, 7>;
extern const SerialJoints SerialZeros;
const double pi = M_PI;
const double EPSILON = std::numeric_limits<double>::epsilon() * 1e2; // 适当放大以应对累积误差




// 读取配置参数
struct KineConfig {
    // DH Parameters
    double d_bs, d_se, d_ew, a_wf, a_se, a_ee;
    
    // Conversion Drive Parameters
    double l_bx, l_by, l_ofsx, l_ofsy, beta40_radians;
    
    // Rod Init Lengths
    double l_m10, l_m20, l_m30;

    // Wrist Joint Parameters
    double d_cx, d_cy, d_cz, d_ax, d_ay, d_az;

    // Joint Limits
    std::vector<std::pair<double, double>> joint_limits;

    // 构造函数，用于从文件加载配置
    KineConfig(const std::string& filepath); // 声明构造函数

    // 添加默认构造函数以允许在不提供文件路径的情况下实例化
    KineConfig() = default;

};


// 通用7DOF 手臂反解结果结构体
struct IKResult {
    SerialJoints final_sol;     // 7个关节角度 (rad)
    bool is_valid;             // 解的有效性标志
    std::vector<SerialJoints> all_solutions;
    double theta7;
    double arm_angle;
    int error_code;

    IKResult() :
        final_sol(SerialZeros),
        is_valid(false),
        all_solutions(8,SerialZeros),
        theta7(0.0),
        arm_angle(0.0),
        error_code (-1)
        {}
};

// 通用7 DOF 手臂 正运动学计算结果结构体
struct FKResult {
    Matrix4d T_08;       // 末端执行器位姿

    // 默认构造函数初始化矩阵
    FKResult() : 
        T_08(Matrix4d::Identity())
    {}
};

class ArmKineBase{
    public:
        virtual ~ArmKineBase() = default; // 虚析构函数，确保正确释放内存
        ArmKineBase(const std::string& config_filepath);

        // 纯虚函数：声明接口，强制派生类实现
        // 每个派生类都必须提供自己的 IK 计算方法,基类纯虚函数包含所有可能的参数
        virtual IKResult calculateIK(
            const Matrix4d& target_pose, // 目标位姿
            double current_joints_array[], // 上一时刻关节向量，选取方案关节距离最小
            std::optional<double> arm_angle = std::nullopt, // 臂角
            std::optional<double> theta7 = std::nullopt   //指定theta7
        ) = 0; // "= 0" 表示这是一个纯虚函数

        virtual FKResult calculateFK(const Vector7d& theta);
    protected:
        // 受保护的成员变量，派生类可以直接访问
        KineConfig params_; // 配置参数
        // 定义提供差异参数的受保护虚函数
        virtual double get_A5_d_param() const = 0; // 纯虚函数，强制派生类提供
        virtual double get_A7_a_param() const = 0; // 纯虚函数，强制派生类提供

};


class ArmKineStd : public ArmKineBase{
    public:
        ArmKineStd(const std::string& config_filepath);
        // 覆盖并实现基类中的纯虚函数
        IKResult calculateIK(
            const Matrix4d& target_pose, 
            double current_joints_array[], 
            std::optional<double> arm_angle = std::nullopt, 
            std::optional<double> theta7 = std::nullopt  // std 下这个传入空
        ) override; // 'override' 关键字是 C++11 的特性，用于明确表示覆盖基类虚函数

        // 覆盖并实现基类中的纯虚函数
        // FKResult calculateFK(const Vector7d& theta);

    protected:

        double get_A5_d_param() const override {
            return params_.d_ew + params_.a_wf; // ArmKineStd 的特定逻辑
        }
        double get_A7_a_param() const override {
            return 0.0; // ArmKineStd 的特定逻辑
        }

};

class ArmKineOfst : public ArmKineBase{
    public:
        ArmKineOfst(const std::string& config_filepath);
        // 覆盖并实现基类中的纯虚函数
        IKResult calculateIK(
            const Matrix4d& target_pose,
            double current_joints_array[],
            std::optional<double> arm_angle = std::nullopt, // ofst 下这个传入空
            std::optional<double> theta7 = std::nullopt
        ) override; // 'override' 关键字是 C++11 的特性，用于明确表示覆盖基类虚函数

        // 覆盖并实现基类中的纯虚函数
        // FKResult calculateFK(const Vector7d& theta);

    protected:
        double get_A5_d_param() const override {
            return params_.d_ew; 
        }
        double get_A7_a_param() const override {
            return params_.a_wf; 
        }
};


class ArmKineComb{
    public:
        ArmKineComb(std::unique_ptr<ArmKineStd> std_solver,
                    std::unique_ptr<ArmKineOfst> ofst_solver)
            :   std_solver_(std::move(std_solver)),
                ofst_solver_(std::move(ofst_solver)){}

        IKResult calculateIK(
            const Matrix4d& target_pose, 
            double current_joints_array[], 
            std::optional<double> arm_angle = std::nullopt, 
            std::optional<double> theta7 = std::nullopt  // std 下这个传入空
        );

        FKResult calculateFK(const Vector7d& theta);

    private:
        std::unique_ptr<ArmKineStd> std_solver_;
        std::unique_ptr<ArmKineOfst> ofst_solver_;

};














#endif