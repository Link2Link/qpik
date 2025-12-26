#include "qpik/tasks/CSpaceTargetTask.hpp"

namespace qpik {

    CSpaceTargetTask::CSpaceTargetTask(
        std::string name,
        Eigen::VectorXd weight,
        double gain,
        double lm_damping)
        : Task(name, weight, gain, lm_damping) {
        this->target_q = Eigen::VectorXd::Zero(weight.size());
    }

    Eigen::VectorXd CSpaceTargetTask::compute_error(Configuration &config, float dt) {
        return (config.q - target_q) / dt;
    }

    Eigen::MatrixXd CSpaceTargetTask::compute_jacobian(Configuration &config, float dt) {
        return Eigen::MatrixXd::Identity(config.model_.nv, config.model_.nv);
    }

} // namespace qpik