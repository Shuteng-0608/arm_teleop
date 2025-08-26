#ifndef TOOLS_H
#define TOOLS_H

#include "kinematics_new.h"

// PQ 指pose quaternion



//末端位姿比较结果, Tee
struct PoseComparisonResult {
    bool is_approximate;
    double translation_error;
    double rotation_error;
    std::string error_type;
};

//比较末端位姿函数
PoseComparisonResult compare_poses_detailed(
    const Matrix4d& ref_pose,
    const Matrix4d& test_pose,
    double trans_tol,
    double rot_tol);

//向量比较函数
void compare_vector7d(const Vector7d& vec1,
                    const Vector7d& vec2,
                    double tolerance) ;


void convert_Hmatrix_to_PQarray(const Matrix4d& matrix, double output[7]);

void print_pose_quaternion(const double* pq, 
                          std::ostream& os = std::cout,
                          int precision = 3);


// 打印矩阵
template<typename Derived>
void print_matrix(const Eigen::MatrixBase<Derived>& mat, 
                    int precision = 4, 
                    int col_width = 10) 
{
    // 直接将实现代码放在这里
    using namespace Eigen;
    const int rows = mat.rows();
    const int cols = mat.cols();
    
    std::ios state(nullptr);
    state.copyfmt(std::cout);
    
    std::cout << std::fixed << std::right 
                << std::setprecision(precision);
    
    std::cout << "Matrix [" << rows << "x" << cols << "]:\n";
    for(int i = 0; i < rows; ++i) {
        std::cout << "| ";
        for(int j = 0; j < cols; ++j) {
            if(mat(i,j) >= 0) std::cout << " ";
            std::cout << std::setw(col_width) << mat(i,j) << " ";
        }
        std::cout << "|\n";
    }
    std::cout.copyfmt(state);
}

// 打印向量
template<typename Derived>
void print_vector(const Eigen::MatrixBase<Derived>& vec,
                 const std::string& name = "Vector",
                 int precision = 4,
                 int col_width = 10) 
{
    static_assert(Derived::ColsAtCompileTime == 1 || Derived::RowsAtCompileTime == 1,
                  "Only vectors (1D matrices) are supported.");

    using namespace Eigen;
    
    // 保存原始输出格式
    std::ios state(nullptr);
    state.copyfmt(std::cout);

    // 设置输出参数
    std::cout << std::fixed << std::right
              << std::setprecision(precision);

    // 判断行列方向
    const bool is_row = (Derived::RowsAtCompileTime == 1);
    const std::string dim_str = (vec.size() <= 8) ? 
        "(" + std::to_string(vec.size()) + ")" :
        "";

    // 打印标题
    std::cout << name << dim_str << " [" 
              << (is_row ? "row" : "column") 
              << " vector]:\n";

    // 打印元素
    const std::string prefix = is_row ? "[ " : "| ";
    const std::string suffix = is_row ? " ]" : " |";
    
    std::cout << prefix;
    for(int i = 0; i < vec.size(); ++i) {
        // 正数前填充空格对齐负号
        if(vec[i] >= 0) std::cout << " ";
        std::cout << std::setw(col_width) << vec[i] << " ";
        
        // 换行逻辑（每8个元素换行）
        if(!is_row && (i+1) % 8 == 0 && (i+1 != vec.size())) {
            std::cout << "\n| ";
        }
    }
    std::cout << suffix << "\n\n";

    // 恢复原始格式
    std::cout.copyfmt(state);
}

// 打印tuple 向量 函数
// 泛型打印函数
template <typename... Args> // Args 代表 tuple 中所有元素的类型
void print_vec_of_tuples(const std::vector<std::tuple<Args...>>& vec) {
    std::cout << "Printing vector of tuples:" << std::endl;
    for (const auto& current_tuple : vec) {
        std::cout << "{ ";
        // 使用 std::apply 将 tuple 的每个元素传递给 lambda
        std::apply([](const auto&... elems) {
            // 使用逗号表达式展开参数包，打印每个元素
            // (std::cout << elems << ", ")... 会在每个元素后加逗号和空格
            // 最后一个元素后面会多一个逗号，可以通过一些技巧避免，这里简化处理
            size_t count = 0;
            ((std::cout << std::fixed << std::setprecision(4) << elems << (++count < sizeof...(elems) ? ", " : "")), ...);
        }, current_tuple);
        std::cout << " }" << std::endl;
    }
}


void print_serial_joints(const SerialJoints& joints);

Vector7d serial_joints_to_vec7d(const SerialJoints& joints_tuple);

SerialJoints vec7d_to_serial_joints(const Vector7d& joints_vec);

void serial_joints_to_double_array(const SerialJoints& joints_tuple, double* out_ptr);

void double_array_to_serial_joints(const double* current_joints_array, SerialJoints& current_joints_tuple);







#endif