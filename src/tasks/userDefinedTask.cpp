#include "qpik/tasks/userDefinedTask.hpp"
#include "qpik/configuration.hpp"
#include <stdexcept>

namespace qpik {

UserDefinedTask::UserDefinedTask(std::string name, Eigen::VectorXd weight,
                                 double gain, double lm_damping)
    : Task(name, weight, gain, lm_damping) {
  this->dim = this->weight.size();
  this->J = Eigen::MatrixXd::Zero(6, dim);
  this->e = Eigen::VectorXd::Zero(dim);
}

void UserDefinedTask::set_weight(const Eigen::VectorXd &weight) {
  for (auto &w : weight) {
    if (w < 0) {
      throw std::invalid_argument("weight must be non-negative");
    }
  }
  if (weight.size() != this->weight.size()) {
    throw std::invalid_argument("weight size mismatch");
  }
  this->weight = weight;
}

void UserDefinedTask::set_gain(double gain) {
  if (gain < 0 || gain > 1) {
    throw std::invalid_argument("gain must be between 0 and 1");
  }
  this->gain = gain;
}

void UserDefinedTask::set_J(const Eigen::MatrixXd &J) {
  if (J.cols() != this->dim) {
    throw std::invalid_argument("J cols must match task dimension");
  }
  this->J = J;
}

void UserDefinedTask::set_e(const Eigen::VectorXd &e) {
  if (e.size() != this->dim) {
    throw std::invalid_argument("e size must match task dimension");
  }
  this->e = e;
}

Eigen::VectorXd UserDefinedTask::compute_error(Configuration &config,
                                               float dt) {
  // 直接返回用户给定的误差向量
  return this->e;
}

Eigen::MatrixXd UserDefinedTask::compute_jacobian(Configuration &config,
                                                  float dt) {
  // 直接返回用户给定的雅可比矩阵
  return this->J;
}

} // namespace qpik