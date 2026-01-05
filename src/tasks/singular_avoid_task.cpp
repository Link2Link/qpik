#include "qpik/tasks/singular_avoid_task.hpp"

namespace qpik {

SingularAvoidTask::SingularAvoidTask(
    std::string name,
    std::string frame_name,
    Eigen::VectorXd weight,
    double gain,
    double lm_damping)
    : Task(name, weight, gain, lm_damping) {
    this->frame_name = frame_name;
}

Eigen::VectorXd
SingularAvoidTask::compute_error(Configuration &config, float dt) {
    Eigen::VectorXd e = Eigen::VectorXd::Zero(config.model_.nv);
    Eigen::MatrixXd Jb = config.Jb(this->frame_name);
    Eigen::MatrixXd A = Jb * Jb.transpose();
    Eigen::MatrixXd A_inv = qpik::utils::pinv(A);

    // 计算可操作度的梯度
    for (int i = 0; i < config.model_.nv; i++) {
        if (this->weight(i) < 1e-6) { continue; }
        Eigen::MatrixXd Mi =
            A_inv * config.Jb_dq(this->frame_name, i) * Jb.transpose();
        double gi = Mi.trace();
        e(i) = -gi;
    }

    return e;
}

Eigen::MatrixXd
SingularAvoidTask::compute_jacobian(Configuration &config, float dt) {
    return Eigen::MatrixXd::Identity(config.model_.nv, config.model_.nv);
}

} // namespace qpik