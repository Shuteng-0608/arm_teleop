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
using SerialJointsWithPlane = std::tuple<double,double,double,double,double,double,double,Vector3d>;
using SerialJointsWithIndex = std::tuple<double,double,double,double,double,double,double,int>;
using SerialJointsWithPhiAndPlane = std::tuple<double,double,double,double,double,double,double,double,Vector3d>;

using JointsArray = std::array<double, 7>;
extern const SerialJoints SerialZeros;
const double pi = M_PI;
const double EPSILON = std::numeric_limits<double>::epsilon() * 1e2; // 适当放大以应对累积误差



// 此部分新加的内容用于参考向量法的反解计算
extern const Vector3d vec_ref;
extern const bool SELECT_SINGLE_SOL_LOG; 
extern const int SELECT_STD_ID;// 选择的解的索引，从0开始;
extern const int SELECT_OFST_ID;// 选择的解的索引，从0开始;





// 读取配置参数
struct KineConfig {
    // DH Parameters
    double d_bs, d_se, d_ew, a_wf, a_se, a_ee;

    // 新增的参数
    double a_ew; // Elbow to Wrist x-offset
    
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
// 在IKReuslt中的arm_angle 是指虚拟机械臂的arm_angle
struct IKResult {
    SerialJoints final_sol;     // 7个关节角度 (rad)
    bool is_valid;             // 解的有效性标志
    std::vector<SerialJoints> all_solutions;
    double theta7;
    double arm_angle;
    int error_code;
    Vector3d n_ref;
    std::vector<double> log_phis;
    std::vector<std::tuple<Matrix3d, Matrix3d, Matrix3d, double, double, double, double>> log_ofst_phi_params;
    Vector3d vec_sw;



    IKResult() :
        final_sol(SerialZeros),
        is_valid(false),
        all_solutions(8,SerialZeros),
        theta7(0.0),
        arm_angle(0.0),
        error_code (-1),
        n_ref(0,0,0),
        log_phis({0.0, 0.0, 0.0, 0.0}),
        log_ofst_phi_params(),
        vec_sw(0,0,0)
        {}
};

// 通用7 DOF 手臂 正运动学计算结果结构体
struct FKResult {
    Matrix4d T_08;       // 末端执行器位姿
    Vector3d n_arm_act;
    Vector3d n_arm_vir;
    Vector3d P_0_elbow_act;
    Vector3d P_0_shoulder_act;
    Vector3d P_0_wrist_act;
    Matrix4d T_04; 
    Matrix4d T_03; 
    Vector3d vec_SW;
    double arm_angle_vec_ref;



    // 默认构造函数初始化矩阵
    FKResult() : 
        T_08(Matrix4d::Identity()),
        n_arm_act(0,0,0),
        n_arm_vir(0,0,0),
        P_0_elbow_act(0,0,0),
        P_0_shoulder_act(0,0,0),
        P_0_wrist_act(0,0,0),
        T_04(Matrix4d::Identity()),
        T_03(Matrix4d::Identity()),
        vec_SW(0,0,0),
        arm_angle_vec_ref(0.0)

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

        // 声明一个虚函数，用于修改 DH 参数
        /*
            set_a_wf 函数说明：
            在创造对象的时候，已经初始化参数了，按照config内容进行初始化
            然后在最开始set_a_wf 相当于修改的是params_.a_wf，会对正反解都产生影响，是正确的作用
        */
        virtual void set_a_wf(double new_a_wf) {
            // 默认实现，可以直接修改基类的 protected 成员
            params_.a_wf = new_a_wf;
        }

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

        IKResult calculateIK_vec_ref(
            const Matrix4d& target_pose, 
            double current_joints_array[], 
            std::optional<double> arm_angle = std::nullopt, 
            std::optional<double> theta7 = std::nullopt  // std 下这个传入空
        );  // 'override' 关键字是 C++11 的特性，用于明确表示覆盖基类虚函数

        // 覆盖并实现基类中的纯虚函数
        // FKResult calculateFK(const Vector7d& theta);

    protected:

        double get_A5_d_param() const override {
            return params_.d_ew + params_.a_wf; // ArmKineStd 的特定逻辑
            // 腕关节认为是在第七关节那里，把机械臂的小臂长补足到带偏置的地方
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

        IKResult calculateIK_vec_ref(
            const Matrix4d& target_pose,
            double current_joints_array[],
            std::optional<double> arm_angle = std::nullopt, // ofst 下这个传入空
            std::optional<double> theta7 = std::nullopt
        ); // 'override' 关键字是 C++11 的特性，用于明确表示覆盖基类虚函数

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
        ArmKineComb(std::shared_ptr<ArmKineStd> std_solver,
                    std::shared_ptr<ArmKineOfst> ofst_solver)
            :   std_solver_(std_solver),
                ofst_solver_(ofst_solver){}

        IKResult calculateIK(
            const Matrix4d& target_pose, 
            double current_joints_array[], 
            std::optional<double> arm_angle = std::nullopt, 
            std::optional<double> theta7 = std::nullopt  // std 下这个传入空
        );

        IKResult calculateIK_vec_ref(
            const Matrix4d& target_pose, 
            double current_joints_array[], 
            std::optional<double> arm_angle = std::nullopt, 
            std::optional<double> theta7 = std::nullopt 
        );

        // 从参数列表中寻找可用的臂角
        IKResult cal_IK_feasible_armAngle_vec_ref(
            const Matrix4d& target_pose,
            double current_joints_array[],
            double current_arm_angle,
            const std::vector<double>& arm_angle_deviation_list = {-0.5, -0.25, 0.0, 0.25, 0.5}, // 默认臂角偏差列表
            double offset_ref = 3.0 // 默认最大关节跳变阈值
        );

        FKResult calculateFK(const Vector7d& theta);

    private:
        std::shared_ptr<ArmKineStd> std_solver_;
        std::shared_ptr<ArmKineOfst> ofst_solver_;

};


// ------- 手腕计算 --------
struct WristConfig {
    // 原始参数，直接从 YAML 读取
    double a_wf, ax, ay, az;
    double cx, cy, cz;
    double dx, dy, dz;
    double l_m10, l_m20, l_m30;
    double d_cx, d_cy, d_cz;
    double d_ax, d_ay, d_az;
    
    // 构造函数声明
    explicit WristConfig(const std::string& filepath);
    // explicit WristConfig(const YAML::Node& wrist_params_node);
    
    // 显式声明一个默认构造函数
    WristConfig() = default;
    
};


// --- 2. 手腕角度计算结果结构体   ---
struct WristAngleResult {
    bool success;
    std::string error_message;
    std::vector<double> angles;
};


// --- 3. Ceres 残差类 (只接收最终预计算值) ---
class WristResidual : public ceres::SizedCostFunction<2, 2> { // 2个残差，2个参数
public:
    // 构造函数只接收最终的预计算系数，这是它真正需要的
    explicit WristResidual(
        double in_l1_sq, double in_l2_sq,
        double in_G1, double in_G2, double in_G3,
        double in_G5, double in_G6, double in_H4, double in_H8
    );
                           
    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const override;
private:
    // 存储所有最终的预计算系数，这些是 const 的
    const double l1_sq_, l2_sq_;
    const double G1_, G2_, G3_, G5_, G6_, H4_, H8_;
};



// --- 4. 手腕运动学求解器类 ---
class WristKinematicsSolver {
public:
    // 构造函数：只接受配置文件路径。
    explicit WristKinematicsSolver(const std::string& config_filepath);

    // 求解手腕角度的核心方法：现在接收 l1 和 l2 作为参数,内部执行预计算，然后进行求解。
    WristAngleResult CalculateWristFK(double l1, double l2);
    std::pair<double, double> CalculateWristIK (double theta6, double theta7);

private:
    // WristKinematicsSolver 只存储从 YAML 读取的原始配置。
    WristConfig raw_config_; 

};


/*
这里采用镜像法求解左臂运动学。
建立全局坐标系，和右臂坐标系朝向一致，原点位于躯干中心
左右臂关于全局坐标系的xoy平面对称，z相反
这里需要注意，
反解计算过程中，target_pose 是左臂末端在全局坐标系下的位姿
正解过程中，算出的也是左臂末端在全局坐标系下的位姿
因为没有建立左手坐标系

*/

// 左臂运动学计算
class ArmLeftKine{
    public:
        ArmLeftKine(std::shared_ptr<ArmKineStd> std_solver,
                std::shared_ptr<ArmKineOfst> ofst_solver);

        //
        IKResult cal_left_arm_IK(
            const Matrix4d& target_pose,  // 左臂末端在全局坐标系下的位姿
            double current_joints_array[], 
            double arm_angle
        );

        // 这里直接传入左臂的关节坐标（等效的串联关节坐标值，无须坐标变换），得到左臂末端在全局坐标系下的位姿
        FKResult calculateFK(const Vector7d& theta); //左臂末端在全局坐标系下的位姿

        static const double length_LR; // 定义左右臂的坐标系之间的距离
        static const Eigen::Matrix4d M_mirror;
        static const Eigen::Matrix4d T_GR;
        static const Eigen::Matrix4d T_RG;

        static const Vector7d sign_vector; // 定义符号向量

    private:

        ArmKineComb right_arm;




};








#endif