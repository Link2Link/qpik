#pragma once
#include "qpik/configuration.hpp"
#include "qpik/tasks/task.hpp"
#include <Eigen/Dense>

namespace qpik {

struct DOFFreezingTask : public Task {
    DOFFreezingTask(
        std::string name,
        Configuration &config,
        std::vector<std::string> freezing_dofs);

    Eigen::VectorXd compute_error(Configuration &config, float dt) override;

    Eigen::MatrixXd compute_jacobian(Configuration &config, float dt) override;

    int dim;
    std::vector<std::string> freezing_dofs;
    std::vector<int> indices;
};

} // namespace qpik