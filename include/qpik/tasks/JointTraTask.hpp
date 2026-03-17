#pragma once
#include "qpik/configuration.hpp"
#include "qpik/tasks/task.hpp"
#include <Eigen/Dense>
#include <stdexcept>
#include <string>
#include <vector>

namespace qpik {

// 简化：仅保留关节名称和期望值
struct JointTargetParam {
  std::string joint_name; // 关节名称（通过名称查索引）
  double q_des = 0.0;     // 关节期望值
};

struct JointTraTask : public Task {
  // 构造函数：仅保留名称和基础增益（无多余参数）
  JointTraTask(std::string name, double gain = 1.0, double lm_damping = 1e-4);

  // 基础参数设置（仅保留增益）
  void set_gain(double gain) {
    // if (gain < 0 || gain > 1) {
    //   throw std::invalid_argument("gain must be between 0 and 1");
    // }
    this->gain = gain;
  }

  // 核心接口：添加需要控制的关节（名称+期望值），支持添加多个
  void add_joint_target(const std::string &joint_name, double q_des) {
    JointTargetParam param;
    param.joint_name = joint_name;
    param.q_des = q_des;
    joint_targets.push_back(param);
  }

  // 核心重载：计算误差（当前关节值 - 期望值）
  Eigen::VectorXd compute_error(Configuration &config, float dt) override;

  // 核心重载：雅可比矩阵（仅目标关节对角线为1，其余为0）
  Eigen::MatrixXd compute_jacobian(Configuration &config, float dt) override;

  // 成员变量：仅保留关节目标列表和增益
  std::vector<JointTargetParam> joint_targets;
  double gain = 1.0; // 误差增益（可选，默认1.0）
};

} // namespace qpik
