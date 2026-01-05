#pragma once
#include "qpik/tasks/task.hpp"

namespace qpik {

struct DampingTask : public Task {
    DampingTask(
        std::string name,
        Eigen::VectorXd weight,
        double gain = 1.0,
        double lm_damping = 1e-4);

    void set_weight(const Eigen::VectorXd &weight) {
        for (auto &w : weight) {
            if (w < 0) {
                throw std::invalid_argument("weight must be non-negative");
            }
        }
        this->weight = weight;
    }

    Eigen::VectorXd compute_error(Configuration &config, float dt) override;
    Eigen::MatrixXd compute_jacobian(Configuration &config, float dt) override;
};

} // namespace qpik