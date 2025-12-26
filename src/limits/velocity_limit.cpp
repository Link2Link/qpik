#include "qpik/limits/velocity_limit.hpp"

namespace qpik {

VelocityLimit::VelocityLimit(
    std::string name,
    Configuration &config,
    const std::map<std::string, double> &joint_velocity_limits)
    : Limit(name), joint_velocity_limits(joint_velocity_limits) {
    this->joint_names = config.joint_names;
    this->joint_ids_map = config.joint_ids_map;
    this->set_joint_velocity_limits(joint_velocity_limits);
}

VelocityLimit::VelocityLimit(std::string name, Configuration &config)
    : Limit(name) {
    this->joint_names = config.joint_names;
    this->joint_ids_map = config.joint_ids_map;

    std::map<std::string, double> joint_velocity_limits;
    joint_velocity_limits.clear();
    int nv = config.model_.nv;
    for (int i = 0; i < nv; ++i) {
        std::string joint_name = config.joint_names[i];
        double max_velocity = config.model_.velocityLimit[i];
        joint_velocity_limits[joint_name] = max_velocity;
        // std::cout << "Default velocity limit for joint " << joint_name <<
        // " : " << max_velocity << std::endl;
    }
    this->set_joint_velocity_limits(joint_velocity_limits);
}

VelocityLimit::VelocityLimit(
    std::string name, Configuration &config, double max_velocity)
    : Limit(name) {
    this->joint_names = config.joint_names;
    this->joint_ids_map = config.joint_ids_map;

    std::map<std::string, double> joint_velocity_limits;
    joint_velocity_limits.clear();
    int nv = config.model_.nv;
    for (int i = 0; i < nv; ++i) {
        std::string joint_name = config.joint_names[i];
        joint_velocity_limits[joint_name] = max_velocity;
        // std::cout << "Default velocity limit for joint " << joint_name <<
        // " : " << max_velocity << std::endl;
    }
    this->set_joint_velocity_limits(joint_velocity_limits);
}

void VelocityLimit::set_joint_velocity_limits(
    const std::map<std::string, double> &joint_velocity_limits) {
    this->joint_velocity_limits = joint_velocity_limits;
    this->indices.clear();
    this->max_velocities.clear();

    for (const auto &pair : joint_velocity_limits) {
        const std::string &joint_name = pair.first;
        double max_velocity = pair.second;
        // auto it = std::find(
        //     this->joint_names.begin(), this->joint_names.end(),
        //     joint_name);
        // if (it == this->joint_names.end()) {
        //     throw std::invalid_argument(
        //         "Joint name not found in configuration");
        // }
        // int joint_index = std::distance(this->joint_names.begin(), it);
        int joint_index = this->joint_ids_map[joint_name];
        this->indices.push_back(joint_index);
        this->max_velocities.push_back(max_velocity);

        // std::cout << "Set velocity limit for joint " << joint_name
        //           << " (index " << joint_index << ") : "
        //           << max_velocity << std::endl;
    }

    int nb = this->indices.size();
    if (nb == 0) {
        throw std::invalid_argument("No joint velocity limits provided");
    }

    Eigen::MatrixXd G1 = Eigen::MatrixXd::Zero(nb, this->joint_names.size());
    Eigen::VectorXd v1 = Eigen::VectorXd::Zero(nb);
    // 正向速度限制
    for (int i = 0; i < nb; i++) {
        G1(i, indices[i]) = 1.0;
        v1(i) = max_velocities[i];
    }

    Eigen::MatrixXd G2 = Eigen::MatrixXd::Zero(nb, this->joint_names.size());
    Eigen::VectorXd v2 = Eigen::VectorXd::Zero(nb);
    // 反向速度限制
    for (int i = 0; i < nb; i++) {
        G2(i, indices[i]) = -1.0;
        v2(i) = max_velocities[i];
    }
    this->G = Eigen::MatrixXd::Zero(2 * nb, this->joint_names.size());
    this->G.block(0, 0, nb, this->joint_names.size()) = G1;
    this->G.block(nb, 0, nb, this->joint_names.size()) = G2;

    this->speed_limits_vec = Eigen::VectorXd::Zero(2 * nb);
    this->speed_limits_vec.block(0, 0, nb, 1) = v1;
    this->speed_limits_vec.block(nb, 0, nb, 1) = v2;
}

Constraint
VelocityLimit::compute_qp_inequalities(Configuration &config, float dt) {
    Constraint constraint;
    constraint.G = this->G;
    constraint.h = this->speed_limits_vec;
    return constraint;
}

} // namespace qpik