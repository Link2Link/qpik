#pragma once
#include "qpik/configuration.hpp"
#include "qpik/tasks/task.hpp"
#include <Eigen/Dense>

namespace qpik {

struct FrameTask : public Task {
    FrameTask(
        std::string name,
        std::string frame_name,
        Eigen::VectorXd weight =
            Eigen::VectorXd::Ones(6), // 权重 长度6 前三个为姿态， 后三个为位置
        double gain = 1.0,
        double lm_damping = 1e-4);

    void set_orientation_weight(const Eigen::Vector3d &weight);
    void set_orientation_weight(double weight);
    void set_position_weight(const Eigen::Vector3d &weight);
    void set_position_weight(double weight);
    void set_target(Eigen::Matrix<double, 4, 4> T_world_target);
    // offset 是对在目标上加的偏移，例如name 是 tcplink，则offset是
    // T_tcplink_realref 此参数使得可以用于更改相对于tcplink的参考点
    void set_offset(const Eigen::Matrix<double, 4, 4> &T_offset);

    bool is_SE3_matrix(const Eigen::Matrix<double, 4, 4> &T);

    Eigen::VectorXd compute_error(Configuration &config, float dt) override;

    Eigen::MatrixXd compute_jacobian(Configuration &config, float dt) override;

    Eigen::Matrix<double, 4, 4> T_world_target;
    Eigen::Matrix<double, 4, 4> T_offset;
    std::string frame_name;
};

} // namespace qpik