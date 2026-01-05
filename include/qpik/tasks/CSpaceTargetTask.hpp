#pragma once
#include "qpik/configuration.hpp"
#include "qpik/tasks/task.hpp"
#include <Eigen/Dense>

namespace qpik {

struct CSpaceTargetTask : public Task {
    CSpaceTargetTask(
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

    void set_gain(double gain) {
        if (gain < 0 || gain > 1) {
            throw std::invalid_argument("gain must be between 0 and 1");
        }
        this->gain = gain;
    }

    void set_target(const Eigen::VectorXd &target) { this->target_q = target; }

    Eigen::VectorXd compute_error(Configuration &config, float dt) override;

    Eigen::MatrixXd compute_jacobian(Configuration &config, float dt) override;

    Eigen::VectorXd target_q;
};

} // namespace qpik