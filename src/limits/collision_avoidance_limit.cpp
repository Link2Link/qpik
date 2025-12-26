#include "qpik/limits/collision_avoidance_limit.hpp"

namespace qpik {

Eigen::MatrixXd compute_contact_normal_jacobian(
    Configuration &config,
    Contact &contact,
    bool double_side_collision_avoidance) {
    //计算接触法线方向的雅可比向量。

    Eigen::Vector3d p1 = contact.p1;
    Eigen::Vector3d p2 = contact.p2;
    Eigen::Vector3d normal = contact.normal();

    Eigen::MatrixXd Ja1 = config.Ja(contact.geom1_name);
    Eigen::MatrixXd Ja2 = config.Ja(contact.geom2_name);

    // 取Ja1和Ja2的后三行
    Eigen::MatrixXd jac1 = Ja1.bottomRows(3);
    Eigen::MatrixXd jac2 = Ja2.bottomRows(3);

    if (double_side_collision_avoidance) {
        return normal.transpose() * (jac2 - jac1);
    } else {
        return normal.transpose() * (-jac1);
    }
}

CollisionAvoidanceLimit::CollisionAvoidanceLimit(
    std::string name,
    Configuration &config,
    CollisionPairs &collision_pairs,
    double gain,
    double minimum_distance_from_collisions,
    double collision_detection_distance,
    double bound_relaxation,
    bool double_side_collision_avoidance)
    : Limit(name)
    , gain(gain)
    , minimum_distance_from_collisions(minimum_distance_from_collisions)
    , collision_detection_distance(collision_detection_distance)
    , bound_relaxation(bound_relaxation)
    , collision_pairs(collision_pairs)
    , double_side_collision_avoidance(double_side_collision_avoidance) {
    this->_construct_geom_pairs(collision_pairs);
    this->max_num_contacts = this->geom_pairs.size();
}

void CollisionAvoidanceLimit::_construct_geom_pairs(
    CollisionPairs &collision_pairs) {
    // 遍历collision_pairs，生成geom_pairs
    this->geom_pairs.clear();
    for (const auto &collision_pair : collision_pairs) {
        auto &first_seq = collision_pair.first;
        auto &second_seq = collision_pair.second;
        for (const auto &first : first_seq) {
            for (const auto &second : second_seq) {
                this->geom_pairs.emplace_back(first, second);
            }
        }
    }
}

Contact CollisionAvoidanceLimit::_compute_contact_with_minimum_distance(
    Configuration &config,
    const std::string &geom1_name,
    const std::string &geom2_name) {
    Contact contact;
    Eigen::Matrix4d T1 = config.FK(geom1_name);
    Eigen::Matrix4d T2 = config.FK(geom2_name);
    Eigen::Vector3d p1 = T1.block<3, 1>(0, 3);
    Eigen::Vector3d p2 = T2.block<3, 1>(0, 3);
    double r1 = config.collision_objects[geom1_name];
    double r2 = config.collision_objects[geom2_name];

    Eigen::Vector3d v12 = p2 - p1;
    double dist = v12.norm();
    v12.normalize();

    contact.p1 = p1 + r1 * v12;
    contact.p2 = p2 - r2 * v12;
    contact.dist = dist - r1 - r2;
    contact.geom1_name = geom1_name;
    contact.geom2_name = geom2_name;
    contact.distmax = this->collision_detection_distance;

    // std::cout << " geom1_name: " << geom1_name << " geom2_name: " <<
    // geom2_name << " dist: " << contact.dist << std::endl;

    if (contact.dist > contact.distmax) contact.dist = contact.distmax; // 截断

    return contact;
}

Constraint CollisionAvoidanceLimit::compute_qp_inequalities(
    Configuration &config, float dt) {
    Constraint constraint;

    std::vector<double> upper_bound_vec;
    // Eigen::VectorXd upper_bound;
    // upper_bound = Eigen::VectorXd::Constant(this->max_num_contacts,
    // std::numeric_limits<double>::infinity());

    std::vector<Eigen::MatrixXd> coefficient_matrix_vec;
    // Eigen::MatrixXd coefficient_matrix;
    // coefficient_matrix = Eigen::MatrixXd::Zero(this->max_num_contacts,
    // config.model_.nv);

    for (size_t idx = 0; idx < this->geom_pairs.size(); ++idx) {
        const auto &geom_pair = this->geom_pairs[idx];
        const std::string &geom1_name = geom_pair.first;
        const std::string &geom2_name = geom_pair.second;

        // 计算 geom1_name 和 geom2_name 之间的距离
        Contact contact = this->_compute_contact_with_minimum_distance(
            config, geom1_name, geom2_name);

        if (contact.inactive()) {
            continue; // 跳过距离较远的碰撞体
        }
        double hi_bound_dist = contact.dist;
        double upper_bound = 0.0;
        if (hi_bound_dist > this->minimum_distance_from_collisions) {
            double dist =
                hi_bound_dist - this->minimum_distance_from_collisions;
            // 正常情况，允许距离收缩的最大法向速度
            upper_bound = (this->gain * dist / dt) + this->bound_relaxation;

        } else {
            // 如果距离已经小于等于最小距离，只允许极小松弛
            upper_bound = this->bound_relaxation;
        }
        upper_bound_vec.push_back(upper_bound);
        Eigen::MatrixXd jac = compute_contact_normal_jacobian(
            config, contact, this->double_side_collision_avoidance);
        double sign =
            (hi_bound_dist >= 0) ? -1.0 : 1.0; // 保证正常面向外或穿透时符号一致
        coefficient_matrix_vec.push_back(sign * jac);

        // std::cout << "inside coefficient_matrix_vec: " <<
        // coefficient_matrix_vec.back() << std::endl;

        // std::cout << "inside upper_bound_vec: " << upper_bound_vec.back() <<
        // std::endl;
    }

    int num_contacts = upper_bound_vec.size();
    constraint.G = Eigen::MatrixXd::Zero(num_contacts, config.model_.nv);
    constraint.h = Eigen::VectorXd::Zero(num_contacts);
    for (int i = 0; i < num_contacts; i++) {
        constraint.G.row(i) = coefficient_matrix_vec[i];
        constraint.h(i) = upper_bound_vec[i];
    }

    // std::cout << "inside constraint.G: " << constraint.G << std::endl;
    // std::cout << "inside constraint.h: " << constraint.h << std::endl;

    return constraint;
}

} // namespace qpik