#pragma once
#include "qpik/limits/limit.hpp"

namespace qpik {

struct AccelerationLimit : public Limit {
    AccelerationLimit(
        std::string name,
        Configuration &config,
        const std::map<std::string, double> &joint_acceleration_limits);
    // 缺省joint_acceleration_limits，所有关节使用默认的accelerationLimit属性,
    // 为速度的50倍
    AccelerationLimit(std::string name, Configuration &config);
    // 默认所有关节加速度限制为max_acceleration
    AccelerationLimit(
        std::string name, Configuration &config, double max_acceleration);

    void set_joint_acceleration_limits(
        const std::map<std::string, double> &joint_acceleration_limits);

    Constraint
    compute_qp_inequalities(Configuration &config, float dt) override;

    std::vector<std::string> joint_names;
    std::map<std::string, int> joint_ids_map;

    std::vector<int> indices;
    std::vector<double> max_accelerations;
    std::map<std::string, double> joint_acceleration_limits;
};

} // namespace qpik