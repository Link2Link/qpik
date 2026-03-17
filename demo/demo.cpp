#include <fcl/fcl.h>
#include "qpik/qpik.hpp"
#include "urdf_parser/urdf_parser.h"
#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <vector>

using namespace qpik;

// 匿名命名空间，用于本文件内部工具函数和结构体
namespace {

// 将urdf::Pose转换为Eigen::Isometry3d（位姿），用于Eigen/FCL等库
Eigen::Isometry3d urdf_pose_to_isometry(const urdf::Pose &pose) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() =
        Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z); // 平移部分
    Eigen::Quaterniond q(pose.rotation.w, pose.rotation.x, pose.rotation.y,
                         pose.rotation.z); // 四元数（注意XYZW顺序）
    T.linear() = q.toRotationMatrix(); // 旋转部分
    return T;
}

// 根据URDF中的Geometry类型构建FCL几何体（指针），便于后续碰撞检测
std::shared_ptr<fcl::CollisionGeometryd>
build_fcl_geometry(const urdf::GeometrySharedPtr &geometry) {
    if (!geometry) {
        return nullptr;
    }
    switch (geometry->type) {
    case urdf::Geometry::SPHERE: {
        // 球体
        const auto *sphere = dynamic_cast<const urdf::Sphere *>(geometry.get());
        return std::make_shared<fcl::Sphered>(sphere->radius);
    }
    case urdf::Geometry::BOX: {
        // 盒体
        const auto *box = dynamic_cast<const urdf::Box *>(geometry.get());
        return std::make_shared<fcl::Boxd>(box->dim.x, box->dim.y, box->dim.z);
    }
    case urdf::Geometry::CYLINDER: {
        // 圆柱体
        const auto *cylinder =
            dynamic_cast<const urdf::Cylinder *>(geometry.get());
        return std::make_shared<fcl::Cylinderd>(cylinder->radius,
                                                cylinder->length);
    }
    case urdf::Geometry::MESH: // 目前暂不支持网格，返回nullptr
    default:
        return nullptr;
    }
}

// 碰撞体信息结构体，方便管理
struct CollisionEntry {
    std::string name; // 关联的link名称
    std::shared_ptr<fcl::CollisionGeometryd> geometry; // FCL几何体指针
    fcl::CollisionObjectd object; // FCL碰撞对象

    CollisionEntry(std::string name_in,
                   std::shared_ptr<fcl::CollisionGeometryd> geometry_in,
                   const fcl::Transform3d &tf)
        : name(std::move(name_in)), geometry(std::move(geometry_in)),
          object(geometry, tf) {} // 构造时初始化成员
};
} // namespace

int main(int argc, char **argv) {
    if (argc < 2) {
        // 命令行参数不足，输出用法
        std::cerr << "用法: " << argv[0] << " <urdf_path>" << std::endl;
        return -1;
    }
    std::string urdf_path = argv[1]; // URDF文件路径
    Configuration config;             // qpik 运动学配置
    int nq = 21;                      // 关节自由度，此处写死为21
    Eigen::VectorXd q = Eigen::VectorXd::Zero(nq); // 初始关节角为零
    config.init(urdf_path, q);        // 根据URDF和初始姿态初始化

    std::cout << "Configuration initialized" << std::endl;

    // 使用urdfdom解析器读取URDF文件
    urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(urdf_path);
    if (!model) {
        std::cerr << "Failed to parse URDF file." << std::endl;
        return -1;
    }

    // 创建碰撞对象集合
    std::vector<CollisionEntry> collision_objects;
    collision_objects.reserve(model->links_.size());

    // 遍历urdf的所有link，提取其碰撞体（仅支持简单几何体）
    for (const auto &link_pair : model->links_) {
        const auto &link = link_pair.second;
        if (!link || !link->collision || !link->collision->geometry) {
            continue; // 跳过无效link或未定义碰撞信息的link
        }

        auto geom = build_fcl_geometry(link->collision->geometry);
        if (!geom) {
            std::cout << "Skip unsupported geometry for link: " << link->name
                      << std::endl;
            continue; // 跳过不支持的碰撞体
        }

        // 计算该Link在世界系下的位姿
        Eigen::Matrix4d T_link_mat = config.FK(link->name);     // Forward Kinematics, 得到世界变换
        Eigen::Isometry3d T_link = Eigen::Isometry3d::Identity();
        T_link.matrix() = T_link_mat;
        Eigen::Isometry3d T_origin =
            urdf_pose_to_isometry(link->collision->origin); // link->collision的相对变换
        Eigen::Isometry3d T_world = T_link * T_origin; // 总变换: link世界变换 * 碰撞原点变换

        collision_objects.emplace_back(link->name, geom, T_world); // 添加到集合
        collision_objects.back().object.computeAABB(); // 更新AABB包围盒，用于加速后续碰撞计算
    }

    // FCL碰撞请求参数设定
    fcl::CollisionRequestd request;
    request.enable_contact = true;      // 记录联系人
    request.num_max_contacts = 10;      // 最多返回10个联系人

    // FCL距离请求参数设定
    fcl::DistanceRequestd dist_request;
    dist_request.enable_nearest_points = true; // 计算最近点对
    dist_request.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD; // 使用LIBCCD GJK求解器

    bool has_collision = false; // 标记是否检测到碰撞
    // 遍历所有碰撞体对（两两组合）
    for (size_t i = 0; i < collision_objects.size(); ++i) {
        for (size_t j = i + 1; j < collision_objects.size(); ++j) {
            fcl::CollisionResultd result;
            // 碰撞检测
            fcl::collide(&collision_objects[i].object,
                         &collision_objects[j].object, request, result);
            if (result.isCollision()) {
                has_collision = true;
                std::cout << "Collision: " << collision_objects[i].name
                          << " <-> " << collision_objects[j].name
                          << ", contacts: " << result.numContacts()
                          << std::endl;
            }

            // 距离计算
            fcl::DistanceResultd dist_result;
            double dist = fcl::distance(&collision_objects[i].object,
                                        &collision_objects[j].object,
                                        dist_request, dist_result);

            // 有碰撞时signed_dist取为负的最大穿透深度，否则等于距离
            double signed_dist = dist;
            if (result.isCollision()) {
                std::vector<fcl::Contactd> contacts;
                result.getContacts(contacts);
                double max_penetration = 0.0;
                for (const auto &c : contacts) {
                    if (c.penetration_depth > max_penetration) {
                        max_penetration = c.penetration_depth;
                    }
                }
                signed_dist = -max_penetration;
            }

            // 输出两物体间距离、带符号距离和最近点
            std::cout << "Distance: " << collision_objects[i].name << " <-> "
                      << collision_objects[j].name << " = " << dist
                      << ", signed: " << signed_dist << std::endl;
            std::cout << "Nearest points: p1 = "
                      << dist_result.nearest_points[0].transpose()
                      << ", p2 = "
                      << dist_result.nearest_points[1].transpose()
                      << std::endl;

            // 输出所有联系人信息
            if (result.isCollision()) {
                std::vector<fcl::Contactd> contacts;
                result.getContacts(contacts);
                for (const auto &c : contacts) {
                    std::cout << "Contact point: " << c.pos.transpose()
                              << ", penetration: " << c.penetration_depth
                              << std::endl;
                }
            }
        }
    }

    // 总结输出是否发现碰撞
    if (!has_collision) {
        std::cout << "No collisions detected." << std::endl;
    }

    // 以下为QP运动学解算main loop的示例模板，已注释
    // while (t < 1.0)
    // {
    //     config.update(q);                      // 更新当前关节位置至config
    //     config.check_limits(1e-4);             // 检查关节极限

    //     std::vector<BaseTask*> tasks;          // 任务目标集合
    //     std::vector<Limit*> limits;            // 约束极限集合
    //     std::vector<Task*> constraints;        // 等式/不等式约束

    //     // 关节空间任务
    //     CSpaceTargetTask cspace_task("CSpaceTargetTask", Eigen::VectorXd::Ones(nq));
    //     cspace_task.set_target(Eigen::VectorXd::Constant(nq, 1.0));
    //     Eigen::VectorXd weight = Eigen::VectorXd::Ones(nq);
    //     for (int i = 0; i < 7; i++) {
    //         weight(i) = 100.0;         // 前7个关节加大权重
    //     }
    //     cspace_task.set_weight(weight);
    //     tasks.push_back(&cspace_task);

    //     // 关节冻结约束
    //     DOFFreezingTask dof_freezing_task("DOFFreezingTask", config, {"first_leg_pitch_joint", "second_leg_pitch_joint"});
    //     constraints.push_back(&dof_freezing_task);

    //     // 速度限制
    //     VelocityLimit velocity_limit("VelocityLimit", config, 3.0);
    //     limits.push_back(&velocity_limit);

    //     QP_Problem problem = construct_qp_problem(config, tasks, limits, constraints, 0.01);

    //     Eigen::VectorXd v;

    //     // QP求解计时
    //     auto qp_start = std::chrono::high_resolution_clock::now();

    //     if (solve_qp_problem(problem, v) < 0) {
    //         std::cerr << "Solver failed" << std::endl;
    //         v.setZero();
    //     }

    //     auto qp_end = std::chrono::high_resolution_clock::now();
    //     std::chrono::duration<double, std::milli> qp_duration = qp_end - qp_start;
    //     std::cout << "QP solve time: " << qp_duration.count() << " ms" << std::endl;

    //     std::cout << "t : " << t << " norm(v): " << v.norm() << std::endl;

    //     q = q + v * dt;           // 按速度更新q
    //     t += dt;                  // 时间步长

    //     std::cout <<" t : " << t << " q:" << q.transpose() << std::endl;
    // }
}