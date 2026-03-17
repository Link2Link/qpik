#pragma once
#include "qpik/configuration.hpp"
#include "qpik/tasks/task.hpp"
#include <Eigen/Dense>

namespace qpik {

struct FrameTraTask : public Task {
  FrameTraTask(std::string name, std::string frame_name,
               Eigen::VectorXd weight = Eigen::VectorXd::Ones(
                   6), // 权重 长度6 前三个为姿态， 后三个为位置
               double gain = 1.0, double lm_damping = 1e-4);

  void set_orientation_weight(const Eigen::Vector3d &weight);
  void set_orientation_weight(double weight);
  void set_position_weight(const Eigen::Vector3d &weight);
  void set_position_weight(double weight);
  void set_target(Eigen::Matrix<double, 4, 4> T_world_target);
  // offset 是对在目标上加的偏移，例如name 是 tcplink，则offset是
  // T_tcplink_realref 此参数使得可以用于更改相对于tcplink的参考点
  void set_offset(const Eigen::Matrix<double, 4, 4> &T_offset);

  bool is_SE3_matrix(const Eigen::Matrix<double, 4, 4> &T);
  void set_kp(const Eigen::VectorXd &kp);
  void set_wFunParameters(double b_gain);
  Eigen::VectorXd compute_error(Configuration &config, float dt) override;

  Eigen::MatrixXd compute_jacobian(Configuration &config, float dt) override;

  Eigen::Matrix<double, 4, 4> T_world_target;
  Eigen::Matrix<double, 4, 4> T_offset;
  Eigen::VectorXd kp;
  std::string frame_name;
  struct RMPMetricParams {
    double sigma_a = 0.1;
    double alpha_min = 0.50;
    double sigma_b = 0.9;   // 控制 beta 的尺度（越大衰减越慢）
    double b_gain = 1200.0; // d≈0 时放大倍数
    double mu_near = 3.0;
    double mu_far = 0.0;
    double eps_norm = 1e-9;
    double beta_p = 1.0; // β 的幂指数（等效于 1/(1+(d/σ)^p) 里 p）
    double gamma = 2.0;  // 强化小误差区（β^gamma）
    double min_scale = 1000.0; // 最小放大（>1 则远端也保留一定权重）
  };

  // 模仿 nvida 的 RMP
  // 设计（让误差小的拥有较高权重保持精度，误差大的拥有较低权重但不会低于阈值，大误差也不会由突变的速度）
  // 位置误差的 RMP 度量函数
  // err: 位置误差向量
  // p: RMPMetricParams 结构体，包含参数
  Eigen::MatrixXd rmpMetricPos(const Eigen::Ref<const Eigen::VectorXd> &err,
                               const RMPMetricParams &p) {
    const int N = static_cast<int>(err.size());
    const double d2 = err.squaredNorm();
    const double d = std::sqrt(d2);
    const double alpha = 1.0;
    const double denom =
        1.0 + std::pow(d / std::max(p.sigma_b, 1e-12), p.beta_p);
    double beta = 1.0 / denom;
    // ---- 改 2：γ 次幂，进一步凸显“小误差=大权重”
    beta = std::pow(beta, p.gamma);
    // 单位方向、投影
    Eigen::VectorXd n = err;
    if (d > p.eps_norm)
      n /= d;
    else {
      n.setZero(N);
      n(0) = 1.0;
    }
    const Eigen::MatrixXd S = n * n.transpose();
    const Eigen::MatrixXd M_near = p.mu_near * Eigen::MatrixXd::Identity(N, N);
    const Eigen::MatrixXd M_far = p.mu_far * S;
    const Eigen::MatrixXd M_mix = alpha * M_near + (1.0 - alpha) * M_far;
    // 原来是 1 与 b_gain 间插值；这里保留，但加入最小下限
    double scale = 1.0 + (p.b_gain - 1.0) * beta;
    scale = std::max(scale, p.min_scale);
    Eigen::MatrixXd M = scale * M_mix;
    M.diagonal().array() += 1e-12;
    return M;
  }

  RMPMetricParams params;
};

} // namespace qpik