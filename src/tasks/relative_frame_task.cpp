#include "qpik/tasks/relative_frame_task.hpp"

namespace qpik {

RelativeFrameTask::RelativeFrameTask(
    std::string name,
    std::string from_frame,
    std::string to_frame,
    Eigen::VectorXd weight,
    double gain,
    double lm_damping)
    : Task(name, weight, gain, lm_damping) {
    this->from_frame = from_frame;
    this->to_frame = to_frame;
    this->Target = Eigen::Matrix4d::Identity();
}

void RelativeFrameTask::set_target(Eigen::Matrix4d target) {
    if (utils::DistanceToSE3(target) > 1e-4) {
        std::cerr << "target must be a valid SE(3) matrix" << std::endl;
        return;
    }
    this->Target = target;
}

void RelativeFrameTask::set_position_weight(const Eigen::Vector3d &weight) {
    for (auto &w : weight) {
        if (w < 0.0) {
            throw std::invalid_argument("weight must be non-negative");
        }
    }
    this->weight.tail(3) = weight;
}

void RelativeFrameTask::set_position_weight(double weight) {
    if (weight < 0.0) {
        throw std::invalid_argument("weight must be non-negative");
    }
    this->weight.tail(3) = Eigen::Vector3d::Constant(weight);
}

void RelativeFrameTask::set_orientation_weight(const Eigen::Vector3d &weight) {
    for (auto &w : weight) {
        if (w < 0.0) {
            throw std::invalid_argument("weight must be non-negative");
        }
    }
    this->weight.head(3) = weight;
}

void RelativeFrameTask::set_orientation_weight(double weight) {
    if (weight < 0.0) {
        throw std::invalid_argument("weight must be non-negative");
    }
    this->weight.head(3) = Eigen::Vector3d::Constant(weight);
}

Eigen::VectorXd
RelativeFrameTask::compute_error(Configuration &config, float dt) {
    Eigen::Matrix4d T_from = config.FK(this->from_frame);
    Eigen::Matrix4d T_to = config.FK(this->to_frame);
    Eigen::Matrix4d T_from_to = utils::TransInv(T_from) * T_to;
    Eigen::VectorXd error = qpik::utils::right_minus(this->Target, T_from_to);
    return -error / dt;
}

Eigen::MatrixXd
RelativeFrameTask::compute_jacobian(Configuration &config, float dt) {
    Eigen::Matrix4d T_from = config.FK(this->from_frame);
    Eigen::Matrix4d T_to = config.FK(this->to_frame);
    Eigen::Matrix4d T_from_to = utils::TransInv(T_from) * T_to;
    Eigen::MatrixXd Jb_from = config.Jb(this->from_frame);
    Eigen::MatrixXd Jb_to = config.Jb(this->to_frame);

    Eigen::MatrixXd Ad_to_from = utils::Adjoint(utils::TransInv(T_from_to));

    Eigen::MatrixXd Jb_from_in_to = Ad_to_from * Jb_from;

    return Jb_to - Jb_from_in_to;
}

} // namespace qpik