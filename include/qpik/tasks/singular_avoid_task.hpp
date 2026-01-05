#pragma once
#include "qpik/tasks/task.hpp"

namespace qpik {

struct SingularAvoidTask : public Task {
    SingularAvoidTask(
        std::string name,
        std::string frame_name,
        Eigen::VectorXd weight,
        double gain = 1.0,
        double lm_damping = 1e-4);

    void set_gain(double gain) { this->gain = gain; }

    Eigen::VectorXd compute_error(Configuration &config, float dt) override;

    Eigen::MatrixXd compute_jacobian(Configuration &config, float dt) override;

    std::string frame_name;
};

} // namespace qpik