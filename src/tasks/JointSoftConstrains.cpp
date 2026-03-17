#include "qpik/tasks/JointSoftConstrains.hpp"

namespace qpik {

// 构造函数：简化，无轨迹跟踪相关初始化
JointSoftConstrains::JointSoftConstrains(std::string name, double gain,
                                         double lm_damping)
    : Task(name, Eigen::VectorXd::Zero(0), gain, lm_damping) {}

// 内部函数：计算单关节避伸直的误差贡献（梯度）
void JointSoftConstrains::compute_joint_avoid_error(
    const JointSoftConstraintParam &param, Configuration &config, float dt,
    const double band_, const double k_gate, const double w_cost,
    const double w_cost_max, const double gain, Eigen::VectorXd &err_vec) {
  // 1. 通过名称查关节索引
  auto it = config.joint_ids_map.find(param.joint_name);
  if (it == config.joint_ids_map.end()) {
    throw std::invalid_argument("Joint name '" + param.joint_name +
                                "' not found!");
  }
  int j_idx = it->second;
  double qj = config.q(config.joint_ids_map[param.joint_name]);

  // 2. 判断是否触发约束
  const double err_threshold =
      (param.towards > 0) ? (qj - param.q_thr) : (param.q_thr - qj);
  if (err_threshold <= 0.0)
    return;

  // 3. 平滑激活 + 权重计算
  const double z = std::min(1.0, err_threshold / std::max(1e-9, band_));
  const double S = smoothstep5(std::min(1.0, k_gate * z));
  double w = w_cost * S;
  w = std::min(w, w_cost_max);
  w_ = std::sqrt(w);
  // 4. 避伸直误差（梯度）：拉回安全角的速度级误差
  err_vec(j_idx) = -gain * w_ * (qj - param.q_safe);
  // std::cout<< "err_vec:::"<< (qj - param.q_safe)<<std::endl;
}

// 仅计算避伸直的误差（无轨迹跟踪）
Eigen::VectorXd JointSoftConstrains::compute_error(Configuration &config,
                                                   float dt) {
  // 初始化全零误差向量（维度=关节自由度）
  Eigen::VectorXd err = Eigen::VectorXd::Zero(config.model_.nv);

  // 遍历所有约束关节，计算避伸直误差
  for (const auto &param : constraint_joints) {
    compute_joint_avoid_error(param, config, dt, band_, k_gate, w_cost,
                              w_cost_max, gain, err);
  }

  return err;
}

// 雅可比矩阵：仅避伸直关节对角线为1，其余为0
Eigen::MatrixXd JointSoftConstrains::compute_jacobian(Configuration &config,
                                                      float dt) {
  // 初始化全零矩阵（维度=关节自由度×关节自由度）

  compute_error(config, dt);
  Eigen::MatrixXd jac =
      Eigen::MatrixXd::Zero(config.model_.nv, config.model_.nv);

  // 遍历约束关节，对应索引对角线设为1
  for (const auto &param : constraint_joints) {
    auto it = config.joint_ids_map.find(param.joint_name);
    if (it != config.joint_ids_map.end()) {
      int j_idx = it->second;
      jac(j_idx, j_idx) =
          1.0; // 仅约束关节位置为1
               // std::cout<< "jac:::"<< this->w_ * jac(j_idx,
               // j_idx)<<std::endl; // 仅约束关节位置为1<<std::endl;
    }
  }

  return this->w_ * jac;
}

} // namespace qpik
