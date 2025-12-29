#pragma once
#include <Eigen/Dense>
#include "qpik/configuration.hpp"
#include "qpik/limits/limit.hpp"

namespace qpik {

struct ConfigurationLimit : public Limit {
    ConfigurationLimit(
        std::string name,
        Configuration &config,
        const std::map<std::string, Eigen::Vector2d> &joint_limits_config);

    ConfigurationLimit(
        std::string name,
        Configuration &config);


    // 根据传入数据更新joint_limits_config、indices和joint_limits
    void set_joint_limits(
        const std::map<std::string, Eigen::Vector2d> &joint_limits_config);

    // gain: 增益因子，取值范围 (0,
    // 1]，用于决定每个关节每步距离极限的比例，取值越小越保守、安全，但收敛慢。
    void set_gain(float gain) {
        if (gain <= 0 || gain > 1) {
            throw std::invalid_argument("gain must be between 0 and 1");
        }
        this->gain = gain;
    }

    // min_distance_from_limits:
    // 极限边界的偏移，正值会收紧关节活动范围（单位为米或弧度），负值则放宽。
    void set_min_distance_from_limits(float min_distance_from_limits) {
        this->min_distance_from_limits = min_distance_from_limits;
    }

    Constraint
    compute_qp_inequalities(Configuration &config, float dt) override;

    std::vector<std::string> joint_names;
    std::map<std::string, int> joint_ids_map;
    std::map<std::string, Eigen::Vector2d> joint_limits_config;

    std::vector<int> indices;
    std::vector<Eigen::Vector2d> joint_limits;

    float gain{0.95};
    float min_distance_from_limits{0.0};
};

} // namespace qpik