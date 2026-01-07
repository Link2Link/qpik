#pragma once

#include <qpik/tasks/task.hpp>

namespace qpik {

class RelativeFrameTask : public Task {
  public:
    RelativeFrameTask(
        std::string name,
        std::string from_frame,
        std::string to_frame,
        Eigen::VectorXd weight = Eigen::VectorXd::Ones(6),
        double gain = 1.0,
        double lm_damping = 1e-4);

    Eigen::VectorXd compute_error(Configuration &config, float dt) override;
    Eigen::MatrixXd compute_jacobian(Configuration &config, float dt) override;

    void set_gain(double gain) { this->gain = gain; }
    void set_target(Eigen::Matrix4d target);
    void set_position_weight(const Eigen::Vector3d &weight);
    void set_position_weight(double weight);
    void set_orientation_weight(const Eigen::Vector3d &weight);
    void set_orientation_weight(double weight);

    Eigen::Matrix<double, 4, 4> Target;

  private:
    std::string from_frame;
    std::string to_frame;
};

} // namespace qpik