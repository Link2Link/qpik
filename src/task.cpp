#include "qpik/tasks/task.hpp"
#include "qpik/configuration.hpp"

namespace qpik {
double Objective::value(Eigen::VectorXd dq) {
    double qHq = 0.5 * dq.transpose() * H * dq;
    double cq = c.transpose() * dq;
    return qHq + cq;
}

Eigen::VectorXd Objective::gradient(Eigen::VectorXd dq) {
    return H * dq + c;
}

Objective Task::compute_qp_objective(Configuration &config, float dt) {
    Eigen::MatrixXd J = compute_jacobian(config, dt);
    Eigen::VectorXd e = compute_error(config, dt);

    auto minus_gain_error = -this->gain * e;

    // Apply weights to Jacobian and error
    Eigen::MatrixXd W = this->weight.asDiagonal();

    Eigen::MatrixXd weighted_jacobian = W * J;
    Eigen::VectorXd weighted_error = W * minus_gain_error;

    // LM型正则化系数，随误差变大而增大
    double mu = this->lm_damping * weighted_error.transpose() * weighted_error;
    Eigen::MatrixXd eye_tg =
        Eigen::MatrixXd::Identity(config.model_.nv, config.model_.nv);

    // Levenberg-Marquardt (damped least squares) Hessian approximation
    Eigen::MatrixXd H = weighted_jacobian.transpose() * weighted_jacobian
                        + mu * eye_tg; // QP二次项 (nv, nv)
    Eigen::VectorXd c =
        -weighted_error.transpose() * weighted_jacobian; // QP一次项 (nv, )

    Objective obj;
    obj.H = H;
    obj.c = c;
    return obj;
}

} // namespace qpik