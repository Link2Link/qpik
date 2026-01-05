#ifndef CONFIGURATION_HPP
#define CONFIGURATION_HPP
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/sample-models.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace qpik {
namespace pin = pinocchio;

typedef std::map<std::string, float> CollisionObjectDict;

struct Configuration {
    pin::Model model_;
    pin::Data data_;

    int nq;
    int nv;

    Eigen::VectorXd q; // 内部状态
    Eigen::VectorXd v; // 速度

    Eigen::VectorXd lower_limit;
    Eigen::VectorXd upper_limit;
    Eigen::VectorXd velocityLimit;

    std::vector<std::string> frame_names;

    std::vector<std::string> joint_names;
    std::map<std::string, int> joint_ids_map;

    std::vector<std::string> link_names;

    std::string urdf_path;

    void init(const std::string &urdf_path, Eigen::VectorXd q);
    void update(Eigen::VectorXd q);
    void update(Eigen::VectorXd q, Eigen::VectorXd v);
    void update_velocity(Eigen::VectorXd v);
    // void update();
    void update_kinematics();
    Eigen::Matrix4d FK(std::string frame_name); // 正向运动学
    Eigen::Matrix4d
    FK(std::string a_frame_name,
       std::string b_frame_name); // 正向运动学 T_ab = Ta^-1 * Tb
    Eigen::MatrixXd Js(std::string frame_name); // 空间雅可比矩阵
    Eigen::MatrixXd Jb(std::string frame_name); // 体坐标系雅可比矩阵
    Eigen::MatrixXd
    Ja(std::string frame_name); // WORLD Local aligned 雅可比矩阵

    Eigen::MatrixXd Jb_dq(std::string frame_name, int idx); // 求Jb对q_i的偏导

    bool check_limits(double tolerance = 1e-6); // 检查关节是否在限位范围内

    Eigen::VectorXd integrate(Eigen::VectorXd v, float dt);
    // void integrate_inplace(Eigen::VectorXd v, float dt);

    void print_info();

    // 碰撞相关
    void load_collision_objects_from_urdf(
        const std::string &prefix = "qpik_collision");
    CollisionObjectDict collision_objects; // 存储碰撞对象和半径
};
} // namespace qpik

#endif // CONFIGURATION_HPP
