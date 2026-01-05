#pragma once
#include "qpik/configuration.hpp"
#include "qpik/limits/limit.hpp"
#include <Eigen/Dense>

namespace qpik {

struct VelocityLimit : public Limit {
    // 接受一个map<string, double>，表示关节的最大速度
    VelocityLimit(
        std::string name,
        Configuration &config,
        const std::map<std::string, double> &joint_velocity_limits);

    // 缺省joint_velocity_limits，所有关节使用默认的velocityLimit属性
    VelocityLimit(std::string name, Configuration &config);

    // 默认所有关节速度限制为max_velocity
    VelocityLimit(std::string name, Configuration &config, double max_velocity);

    void set_joint_velocity_limits(
        const std::map<std::string, double> &joint_velocity_limits);

    void set_scale(double scale) { this->scale = scale; }

    Constraint
    compute_qp_inequalities(Configuration &config, float dt) override;

    std::vector<std::string> joint_names;
    std::map<std::string, int> joint_ids_map;

    std::vector<int> indices;
    std::vector<double> max_velocities;
    Eigen::MatrixXd G;
    Eigen::VectorXd speed_limits_vec;
    std::map<std::string, double> joint_velocity_limits;

    double scale{1.0};
};

} // namespace qpik