#include "qpik/limits/acceleration_limit.hpp"

namespace qpik {

AccelerationLimit::AccelerationLimit(
    std::string name,
    Configuration &config,
    const std::map<std::string, double> &joint_acceleration_limits)
    : Limit(name), joint_acceleration_limits(joint_acceleration_limits) {
    this->joint_names = config.joint_names;
    this->joint_ids_map = config.joint_ids_map;
    this->set_joint_acceleration_limits(joint_acceleration_limits);
}

AccelerationLimit::AccelerationLimit(std::string name, Configuration &config)
    : Limit(name) {
    this->joint_names = config.joint_names;
    this->joint_ids_map = config.joint_ids_map;

    std::map<std::string, double> joint_acceleration_limits;
    joint_acceleration_limits.clear();
    int nv = config.model_.nv;
    for (int i = 0; i < nv; ++i) {
        std::string joint_name = config.joint_names[i];
        double max_velocity = config.model_.velocityLimit[i];
        joint_acceleration_limits[joint_name] = max_velocity * 50.0;
    }
    this->set_joint_acceleration_limits(joint_acceleration_limits);
}

AccelerationLimit::AccelerationLimit(
    std::string name, Configuration &config, double max_acceleration)
    : Limit(name) {
    this->joint_names = config.joint_names;
    this->joint_ids_map = config.joint_ids_map;

    std::map<std::string, double> joint_acceleration_limits;
    joint_acceleration_limits.clear();
    int nv = config.model_.nv;
    for (int i = 0; i < nv; ++i) {
        std::string joint_name = config.joint_names[i];
        joint_acceleration_limits[joint_name] = max_acceleration;
    }
    this->set_joint_acceleration_limits(joint_acceleration_limits);
}

void AccelerationLimit::set_joint_acceleration_limits(
    const std::map<std::string, double> &joint_acceleration_limits) {
    this->joint_acceleration_limits = joint_acceleration_limits;
    this->indices.clear();
    this->max_accelerations.clear();

    for (const auto &pair : joint_acceleration_limits) {
        const std::string &joint_name = pair.first;
        double max_acceleration = pair.second;
        int joint_index = this->joint_ids_map[joint_name];
        this->indices.push_back(joint_index);
        this->max_accelerations.push_back(max_acceleration);
    }

    int nb = this->indices.size();
    if (nb == 0) {
        throw std::invalid_argument("No joint acceleration limits provided");
    }
}

Constraint
AccelerationLimit::compute_qp_inequalities(Configuration &config, float dt) {
    Constraint constraint;

    int nb = this->indices.size();
    Eigen::MatrixXd G1 = Eigen::MatrixXd::Zero(nb, this->joint_names.size());
    Eigen::VectorXd a1 = Eigen::VectorXd::Zero(nb);
    // 正向加速度限制
    for (int i = 0; i < nb; i++) {
        G1(i, indices[i]) = 1.0;
        a1(i) = max_accelerations[i] * dt + config.v(indices[i]);
    }

    Eigen::MatrixXd G2 = Eigen::MatrixXd::Zero(nb, this->joint_names.size());
    Eigen::VectorXd a2 = Eigen::VectorXd::Zero(nb);
    // 反向加速度限制
    for (int i = 0; i < nb; i++) {
        G2(i, indices[i]) = -1.0;
        a2(i) = max_accelerations[i] * dt - config.v(indices[i]);
    }

    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(2 * nb, this->joint_names.size());
    G.block(0, 0, nb, this->joint_names.size()) = G1;
    G.block(nb, 0, nb, this->joint_names.size()) = G2;

    Eigen::VectorXd a = Eigen::VectorXd::Zero(2 * nb);
    a.block(0, 0, nb, 1) = a1;
    a.block(nb, 0, nb, 1) = a2;

    constraint.G = G;
    constraint.h = a;

    return constraint;
}

} // namespace qpik