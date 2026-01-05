#include "qpik/configuration.hpp"
#include "qpik/math_utils.hpp"
#include "urdf_parser/urdf_parser.h"
#include <Eigen/Dense>
#include <qpik/qpik.hpp>

namespace qpik {
void Configuration::init(const std::string &urdf_path, Eigen::VectorXd q) {
    this->urdf_path = urdf_path;
    int dof = q.size();
    this->q = q;
    this->v = Eigen::VectorXd::Zero(dof);

#ifdef DEVELOPMENT_DEBUG
    std::cout << "Initializing configuration with URDF path: " << urdf_path
              << std::endl;
#endif

    pin::urdf::buildModel(urdf_path, this->model_);
    this->data_ = pin::Data(this->model_);

    if (this->model_.nq != dof) {
        std::cerr << "关节数量不匹配,模型关节数量: " << this->model_.nq
                  << ",配置关节数量: " << dof << std::endl;
        return;
    }

    this->nq = this->model_.nq;
    this->nv = this->model_.nv;

    // this->v = Eigen::VectorXd::Zero(this->model_.nv);
    // this->a = Eigen::VectorXd::Zero(this->model_.nv);
    this->update_kinematics();

    for (int i = 0; i < this->nq; ++i) {
        pinocchio::JointIndex joint_idx = i + 1;
        std::string joint_name = model_.names[joint_idx];
        this->joint_ids_map.insert(std::make_pair(joint_name, i));
        this->joint_names.push_back(joint_name);
    }

    for (const auto &frame : this->model_.frames) {
        this->frame_names.push_back(frame.name);
        // 判断frame是不是link类型
        if (frame.type == pin::FrameType::BODY) {
            this->link_names.push_back(frame.name);
        }
    }

    // 读取并保存关节限位信息
    this->lower_limit = Eigen::VectorXd::Zero(this->nq);
    this->upper_limit = Eigen::VectorXd::Zero(this->nq);
    this->velocityLimit = Eigen::VectorXd::Zero(this->nv);
    for (int i = 0; i < this->nq; ++i) {
        this->lower_limit(i) = this->model_.lowerPositionLimit[i];
        this->upper_limit(i) = this->model_.upperPositionLimit[i];
    }
    for (int i = 0; i < this->nv; ++i) {
        this->velocityLimit(i) = this->model_.velocityLimit[i];
    }
    this->load_collision_objects_from_urdf("qpik_collision");
}

bool Configuration::check_limits(double tolerance) {
    for (int i = 0; i < this->nq; ++i) {
        if (this->q(i) < this->lower_limit(i) - tolerance
            || this->q(i) > this->upper_limit(i) + tolerance) {
            return false;
        }
    }
    return true;
}

Eigen::VectorXd Configuration::integrate(Eigen::VectorXd v, float dt) {
    return this->q + v * dt;
}

// void Configuration::integrate_inplace(Eigen::VectorXd v, float dt) {
//     this->q = this->q + v * dt;
// }

void Configuration::update(Eigen::VectorXd q) {
    // 检查只存下长度和当前自由度数量一致的q尺寸，否则报错
    if (q.size() != this->nq) {
        std::cerr << "更新失败，传入q尺寸(" << q.size() << ")与自由度("
                  << this->nq << ")不一致" << std::endl;
        return;
    }
    this->q = q;
    this->update_kinematics();
}

void Configuration::update(Eigen::VectorXd q, Eigen::VectorXd v) {
    this->update(q);
    this->v = v;
}

void Configuration::update_velocity(Eigen::VectorXd v) {
    this->v = v;
}

void Configuration::update_kinematics() {
    pin::forwardKinematics(this->model_, this->data_, this->q);
    pin::updateFramePlacements(this->model_, this->data_);
    pin::updateGlobalPlacements(this->model_, this->data_);
    pin::computeJointJacobians(this->model_, this->data_, this->q);
    // pin::computeJointJacobiansTimeVariation(this->model_, this->data_,
    // this->q, this->v);
}

Eigen::Matrix4d Configuration::FK(std::string frame_name) {
    return this->data_.oMf[this->model_.getFrameId(frame_name)]
        .toHomogeneousMatrix();
}

Eigen::Matrix4d
Configuration::FK(std::string a_frame_name, std::string b_frame_name) {
    return this->data_.oMf[this->model_.getFrameId(a_frame_name)]
               .inverse()
               .toHomogeneousMatrix()
           * this->data_.oMf[this->model_.getFrameId(b_frame_name)]
                 .toHomogeneousMatrix();
}

Eigen::MatrixXd Configuration::Js(std::string frame_name) {
    Eigen::MatrixXd Js_pin = pin::getFrameJacobian(
        this->model_, this->data_, this->model_.getFrameId(frame_name),
        pin::ReferenceFrame::WORLD);

    return qpik::utils::switch_w_v(Js_pin);
}

Eigen::MatrixXd Configuration::Jb(std::string frame_name) {
    Eigen::MatrixXd Jb_pin = pin::getFrameJacobian(
        this->model_, this->data_, this->model_.getFrameId(frame_name),
        pin::ReferenceFrame::LOCAL);

    return qpik::utils::switch_w_v(Jb_pin);
}

Eigen::MatrixXd Configuration::Ja(std::string frame_name) {
    Eigen::MatrixXd Ja_pin = pin::getFrameJacobian(
        this->model_, this->data_, this->model_.getFrameId(frame_name),
        pin::ReferenceFrame::LOCAL_WORLD_ALIGNED);
    return qpik::utils::switch_w_v(Ja_pin);
}

Eigen::MatrixXd Configuration::Jb_dq(std::string frame_name, int idx) {
    Eigen::VectorXd temp_v = Eigen::VectorXd::Zero(this->model_.nv);
    temp_v(idx) = 1.0;
    pin::computeJointJacobiansTimeVariation(
        this->model_, this->data_, this->q, temp_v);
    Eigen::MatrixXd dJ_pin = Eigen::MatrixXd::Zero(6, this->model_.nv);
    pin::getFrameJacobianTimeVariation(
        this->model_, this->data_, this->model_.getFrameId(frame_name),
        pin::ReferenceFrame::LOCAL, dJ_pin);
    return qpik::utils::switch_w_v(dJ_pin);
}

void Configuration::print_info() {
    std::cout
        << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
        << std::endl;
    std::cout << "关节数量: " << this->joint_names.size() << std::endl;
    std::cout << "坐标系数量: " << this->frame_names.size() << std::endl;
    std::cout << "连杆数量: " << this->link_names.size() << std::endl;
    std::cout
        << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
        << std::endl;
    std::cout << "关节限位信息: " << std::endl;
    std::cout << "关节限位下限: " << this->lower_limit.transpose() << std::endl;
    std::cout << "关节限位上限: " << this->upper_limit.transpose() << std::endl;
    std::cout
        << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
        << std::endl;
    std::cout << "关节最大速度: " << this->velocityLimit.transpose()
              << std::endl;
    std::cout
        << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
        << std::endl;
}

void Configuration::load_collision_objects_from_urdf(
    const std::string &prefix) {
    this->collision_objects.clear();
    urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(urdf_path);
    if (!model) { throw std::runtime_error("Failed to parse URDF file."); }

    std::vector<std::string> collision_link_names;
    for (const auto &link_pair : model->links_) {
        const std::string &link_name = link_pair.first;
        // 判断link_name是否以QPIK_COLLISION_PREFIX开头
        if (link_name.rfind(prefix, 0) == 0) {
            collision_link_names.push_back(link_name);
        }
    }

    for (const auto &link_name : collision_link_names) {
        urdf::LinkConstSharedPtr link = model->getLink(link_name);
        // Check collision geometry
        if (!link->collision || !link->collision->geometry) {
            throw std::runtime_error(
                "Link '" + link_name + "' has no collision geometry.");
        }
        // Cast to Sphere
        const urdf::Geometry *geom = link->collision->geometry.get();
        if (geom->type == urdf::Geometry::SPHERE) {
            const urdf::Sphere *sphere =
                static_cast<const urdf::Sphere *>(geom);
            this->collision_objects[link_name] = sphere->radius;
        } else {
            throw std::runtime_error("Collision geometry is not a sphere.");
        }
    }
}
} // namespace qpik