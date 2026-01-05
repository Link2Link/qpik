#include "qpik/tasks/damping_task.hpp"

namespace qpik {

DampingTask::DampingTask(
    std::string name, Eigen::VectorXd weight, double gain, double lm_damping)
    : Task(name, weight, gain, lm_damping) {
    this->weight = weight;
}

Eigen::VectorXd DampingTask::compute_error(Configuration &config, float dt) {
    return Eigen::VectorXd::Zero(config.model_.nv);
}

Eigen::MatrixXd DampingTask::compute_jacobian(Configuration &config, float dt) {
    return Eigen::MatrixXd::Identity(config.model_.nv, config.model_.nv);
}

} // namespace qpik