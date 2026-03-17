#include "qpik/tasks/JointTraTask.hpp"

namespace qpik {

// 构造函数：极简初始化（仅继承Task基础参数）
JointTraTask::JointTraTask(std::string name, double gain, double lm_damping)
    : Task(name, Eigen::VectorXd::Zero(0), gain, lm_damping) {}

// 误差计算：仅计算目标关节的（当前值 - 期望值），无任何平滑/权重
Eigen::VectorXd JointTraTask::compute_error(Configuration &config, float dt) {
  // 初始化全零误差向量（维度=机器人总关节自由度）
  Eigen::VectorXd err = Eigen::VectorXd::Zero(config.model_.nv);

  // 遍历所有目标关节，计算误差
  for (const auto &param : joint_targets) {
    // 通过名称查关节索引
    auto it = config.joint_ids_map.find(param.joint_name);
    if (it == config.joint_ids_map.end()) {
      throw std::invalid_argument("Joint name '" + param.joint_name +
                                  "' not found in config!");
    }
    int j_idx = it->second;
    // 误差 = 增益 * (当前关节值 - 期望值)
    // 若需要速度级误差，可改为：err(j_idx) = gain * (config.q(j_idx) -
    // param.q_des) / dt;
    err(j_idx) = gain * (config.q(j_idx) - param.q_des);
  }

  return err;
}

// 雅可比矩阵：仅目标关节对角线位置为1，其余全0
Eigen::MatrixXd JointTraTask::compute_jacobian(Configuration &config,
                                               float dt) {
  // 初始化全零矩阵（维度=总关节自由度×总关节自由度）
  Eigen::MatrixXd jac =
      Eigen::MatrixXd::Zero(config.model_.nv, config.model_.nv);

  // 遍历目标关节，对应索引对角线设为1
  for (const auto &param : joint_targets) {
    auto it = config.joint_ids_map.find(param.joint_name);
    if (it != config.joint_ids_map.end()) {
      int j_idx = it->second;
      jac(j_idx, j_idx) = 1.0; // 仅目标关节位置为1
    }
  }

  return jac;
}

} // namespace qpik
