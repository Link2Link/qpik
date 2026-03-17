#pragma once
#include "qpik/tasks/task.hpp"

namespace qpik {

//
// 用户自定义任务（UserDefinedTask）
//
// 此结构体允许用户通过外部赋值的雅可比矩阵J和误差向量e，将任务灵活接入QP（Quadratic
// Programming）优化框架。
// 适用于无法用预定义任务类型表达的自定义优化目标；用户只需设定与任务相关联的J、e即可。
// ◇ J: 任务的雅可比矩阵，维度为 (task_dim, robot_dof)
// ◇ e: 任务的误差向量，维度为 (task_dim)
// ◇ weight: 权重向量（每个任务维度的权重），各元素非负
// ◇ gain: 阻尼系数，范围[0,1]
// ◇ lm_damping: Levenberg-Marquardt型阻尼系数
// ◇ dim: 任务维数
/**
 * 数学解释：
 *
 * UserDefinedTask允许用户自定义任务的误差向量 e 与雅可比矩阵
 * J，从而灵活地将各种优化目标接入QP框架。
 *
 * 在线性任务描述下，每一个任务可抽象为以下形式：
 *
 *     min_{dq}   0.5 * || W (J dq + e) ||^2 + 0.5 * mu * ||dq||^2
 *
 * 其中：
 *   - dq          : 机器人配置的增量（优化变量）
 *   - J           : 任务对于dq的雅可比矩阵（task_dim × robot_dof），由用户设定
 *   - e           : 当前任务误差（task_dim），由用户设定
 *   - W           : 对角权重矩阵，对每个误差分量赋权（由weight向量构造）
 *   - mu          : LM正则化系数（随优化过程动态调整）
 *
 * 实际优化时，系统会自动根据
 * J、e、weight、gain等参数计算二次规划问题（QP）中的目标函数的二次项（H）和一次项（c），
 * 即经过加权和正则化的：
 *     H = (WJ)^T (WJ) + mu * I
 *     c = (We)^T (WJ)
 * 以便下游QP求解。
 *
 * 用户在使用本任务类型时，仅需通过set_J/set_e等接口设置自己需要的J/e，系统会自动处理QP模型的构建。
 */
struct UserDefinedTask : public Task {
  /**
   * @brief 构造函数
   * @param name         任务名称
   * @param weight       权重向量，长度=任务维度
   * @param gain         增益（默认1.0，取值范围[0,1]）
   * @param lm_damping   阻尼系数（默认1e-4）
   */
  UserDefinedTask(std::string name, Eigen::VectorXd weight, double gain = 1.0,
                  double lm_damping = 1e-4);

  /**
   * @brief 设置权重向量（需与dim一致，所有元素须非负）
   * @param weight 权重， 内部检查尺寸和非负性
   */
  void set_weight(const Eigen::VectorXd &weight);

  /**
   * @brief 设置任务增益，范围[0,1]
   * @param gain 增益参数， 内部检查范围
   */
  void set_gain(double gain);

  /**
   * @brief 设置任务雅可比矩阵J
   * @param J 任务Jacobian，尺寸为(task_dim, robot_dof)
   */
  void set_J(const Eigen::MatrixXd &J);

  /**
   * @brief 设置任务误差向量e
   * @param e 误差向量，长度=任务维度
   */
  void set_e(const Eigen::VectorXd &e);

  /**
   * @brief 计算任务误差（直接返回用户缓存的e）
   * @param config 当前机器人配置
   * @param dt     步长（未使用）
   * @return 任务误差向量
   */
  Eigen::VectorXd compute_error(Configuration &config, float dt) override;

  /**
   * @brief 计算任务雅可比矩阵（直接返回用户缓存的J）
   * @param config 当前机器人配置
   * @param dt     步长（未使用）
   * @return 任务雅可比矩阵
   */
  Eigen::MatrixXd compute_jacobian(Configuration &config, float dt) override;

  Eigen::MatrixXd J; ///< 任务雅可比矩阵（task_dim x robot_dof）
  Eigen::VectorXd e; ///< 任务误差向量（task_dim）
  int dim;           ///< 任务维度
};

} // namespace qpik