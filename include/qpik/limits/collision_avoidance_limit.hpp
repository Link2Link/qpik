#pragma once
#include "qpik/configuration.hpp"
#include "qpik/limits/limit.hpp"
#include <Eigen/Dense>
namespace qpik {
using Geom = std::string;
using GeomSequence = std::vector<Geom>;
using CollisionPair = std::pair<GeomSequence, GeomSequence>;
using CollisionPairs = std::vector<CollisionPair>;

struct Contact {
    // 用于存储两个几何体间接触信息的结构体。
    double dist;
    Eigen::Vector3d p1;     // 第一个几何体上的接触点
    Eigen::Vector3d p2;     // 第二个几何体上的接触点
    std::string geom1_name; // 第一个几何体的坐标系名称
    std::string geom2_name; // 第二个几何体的坐标系名称
    double distmax;         // 两个几何体之间的最大检测距离

    //接触法线，方向从geom1指向geom2
    Eigen::Vector3d normal() {
        Eigen::Vector3d n;
        n = p2 - p1;
        n.normalize();
        return n;
    }

    bool inactive() {
        // 是否超出最大检测距离，用以降低后续碰撞检测计算量
        return (dist >= distmax || std::abs(dist - distmax) < 1e-5);
    }
};

Eigen::MatrixXd compute_contact_normal_jacobian(
    Configuration &config,
    Contact &contact,
    bool double_side_collision_avoidance);

/**
 * @class CollisionAvoidanceLimit
 * @brief 用于实现碰撞规避约束的限制类。
 *
 *
 * @param name 限制器的名称
 * @param config qpik的配置对象
 * @param collision_pairs 碰撞对
 * @param gain 碰撞增益系数，用于控制碰撞约束的松弛程度
 * @param minimum_distance_from_collisions
 * 最小碰撞距离，小于该距离的碰撞会采用固定的松弛
 * @param collision_detection_distance
 * 碰撞检测距离，用于控制碰撞检测的范围，超过该距离的碰撞不会进入后续计算
 * @param bound_relaxation
 * 范围松弛量，大于最小碰撞距离的碰撞会采用该松弛量作为上界
 * @param double_side_collision_avoidance 是否双向优化
 */

struct CollisionAvoidanceLimit : public Limit {
    CollisionAvoidanceLimit(
        std::string name,
        Configuration &config,
        CollisionPairs &collision_pairs,
        double gain = 0.85,
        double minimum_distance_from_collisions = 0.005,
        double collision_detection_distance = 0.01,
        double bound_relaxation = 0.0,
        bool double_side_collision_avoidance = false);

    void _construct_geom_pairs(CollisionPairs &collision_pairs);

    bool double_side_collision_avoidance;
    double gain;
    double minimum_distance_from_collisions;
    double collision_detection_distance;
    double bound_relaxation;
    int max_num_contacts;
    CollisionPairs collision_pairs;

    std::vector<std::pair<std::string, std::string>> geom_pairs;

    Contact _compute_contact_with_minimum_distance(
        Configuration &config,
        const std::string &geom1_name,
        const std::string &geom2_name);

    Constraint
    compute_qp_inequalities(Configuration &config, float dt) override;
};

} // namespace qpik