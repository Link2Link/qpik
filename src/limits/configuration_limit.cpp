#include "qpik/limits/configuration_limit.hpp"

namespace qpik {

ConfigurationLimit::ConfigurationLimit(
    std::string name,
    Configuration &config,
    const std::map<std::string, Eigen::Vector2d> &joint_limits_config)
    : Limit(name) {
    this->joint_names = config.joint_names;
    this->joint_ids_map = config.joint_ids_map;
    this->set_joint_limits(joint_limits_config);
}

ConfigurationLimit::ConfigurationLimit(std::string name, Configuration &config)
    : Limit(name) {
    this->joint_names = config.joint_names;
    this->joint_ids_map = config.joint_ids_map;

    std::map<std::string, Eigen::Vector2d> joint_limits_config;
    joint_limits_config.clear();
    for (int i = 0; i < config.model_.nq; i++) {
        std::string joint_name = config.joint_names[i];
        Eigen::Vector2d joint_limit = Eigen::Vector2d::Zero(2);
        joint_limit(0) = config.lower_limit(i);
        joint_limit(1) = config.upper_limit(i);
        joint_limits_config[joint_name] = joint_limit;
    }
    this->set_joint_limits(joint_limits_config);
}

void ConfigurationLimit::set_joint_limits(
    const std::map<std::string, Eigen::Vector2d> &joint_limits_config) {
    this->joint_limits_config = joint_limits_config;
    this->indices.clear();
    this->joint_limits.clear();

    for (const auto &pair : joint_limits_config) {
        const std::string &joint_name = pair.first;
        const Eigen::Vector2d &joint_limit = pair.second;
        this->indices.push_back(this->joint_ids_map[joint_name]);
        this->joint_limits.push_back(joint_limit);
    }
}

Constraint
ConfigurationLimit::compute_qp_inequalities(Configuration &config, float dt) {
    Constraint constraint;
    int num = this->indices.size(); // 关节限位数量 会产生 num * 2 个不等式约束

    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(num * 2, config.model_.nv);
    Eigen::VectorXd h = Eigen::VectorXd::Zero(num * 2);

    for (int i = 0; i < num; i++) {
        int idx = this->indices[i];
        double lower_limit = this->joint_limits[i](0);
        double upper_limit = this->joint_limits[i](1);
        // G * dq  <= h 的约束
        // 正向限制 v <= K (q_max - q)/dt
        G(i, idx) = 1.0;
        upper_limit -= this->min_distance_from_limits;
        h(i) = this->gain * (upper_limit - config.q(idx)) / dt;
        // 负向限制 -v <= K (q_min - q)/dt
        G(i + num, idx) = -1.0;
        lower_limit += this->min_distance_from_limits;
        h(i + num) = this->gain * (config.q(idx) - lower_limit) / dt;
    }

    constraint.G = G;
    constraint.h = h;
    return constraint;
}

} // namespace qpik