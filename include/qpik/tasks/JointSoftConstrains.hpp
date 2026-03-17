#pragma once
#include "qpik/configuration.hpp"
#include "qpik/tasks/task.hpp"
#include <Eigen/Dense>
#include <stdexcept>
#include <string>
#include <vector>

namespace qpik {

// 平滑激活函数
inline double smoothstep5(double z) {
  z = std::min(1.0, std::max(0.0, z));
  double z2 = z * z, z3 = z2 * z, z4 = z3 * z, z5 = z4 * z;
  return 6 * z5 - 15 * z4 + 10 * z3; // S(z)
}

// 单关节软约束参数
struct JointSoftConstraintParam {
  std::string joint_name; // 关节名称（通过名称查索引）
  double q_thr = -0.2;    // 介入阈值
  double q_safe = -0.25;  // 安全角度
  int towards = +1;       // 方向：+1限制上界，-1限制下界
};

struct JointSoftConstrains : public Task {
  // 构造函数：简化，仅保留基础Task参数（无轨迹跟踪相关）
  JointSoftConstrains(std::string name, double gain = 1.0,
                      double lm_damping = 1e-4);

  // 基础参数设置（仅保留必要的）
  void set_gain(double gain) {
    // if (gain < 0 || gain > 1) {
    //   throw std::invalid_argument("gain must be between 0 and 1");
    // }
    this->gain = gain;
  }
  void compute_joint_avoid_error(const JointSoftConstraintParam &param,
                                 Configuration &config, float dt,
                                 const double band_, const double k_gate,
                                 const double w_cost, const double w_cost_max,
                                 const double gain, Eigen::VectorXd &err_vec);
  // 通用接口：添加需要避伸直的关节（调用时传不同参数即可）
  void add_soft_constraint_joint(const std::string &joint_name, double q_thr,
                                 double q_safe, int towards) {
    if (towards != +1 && towards != -1) {
      throw std::invalid_argument("towards must be +1 or -1");
    }
    JointSoftConstraintParam param;
    param.joint_name = joint_name;
    param.q_thr = q_thr;
    param.q_safe = q_safe;
    param.towards = towards;
    constraint_joints.push_back(param);
  }

  // 设置通用平滑/权重参数
  void set_common_param(double band, double k_gate, double w_cost,
                        double w_cost_max) {
    if (band <= 0 || k_gate <= 0 || w_cost < 0 || w_cost_max < w_cost) {
      throw std::invalid_argument(
          "Invalid param: band/k_gate>0, w_cost_max>=w_cost>=0");
    }
    this->band_ = band;
    this->k_gate = k_gate;
    this->w_cost = w_cost;
    this->w_cost_max = w_cost_max;
  }

  // 核心重载：仅返回避伸直的误差（梯度）
  Eigen::VectorXd compute_error(Configuration &config, float dt) override;

  // 核心重载：仅避伸直关节对角线为1，其余为0
  Eigen::MatrixXd compute_jacobian(Configuration &config, float dt) override;

  // 成员变量（仅保留避伸直相关）
  std::vector<JointSoftConstraintParam> constraint_joints;
  double band_ = 0.15;     // 平滑激活带宽
  double k_gate = 80.0;    // 激活陡峭度
  double w_cost = 1e5;     // 基础权重
  double w_cost_max = 5e5; // 最大权重限制
  double w_ = 0.0;         // 开根号！！！！！！！！！！！！
};

} // namespace qpik
