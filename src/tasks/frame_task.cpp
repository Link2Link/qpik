#include "qpik/tasks/frame_task.hpp"
#include <iostream>
#include <stdexcept>

namespace qpik {
FrameTask::FrameTask(
    std::string name,
    std::string frame_name,
    Eigen::VectorXd weight,
    double gain,
    double lm_damping)
    : Task(name, weight, gain, lm_damping) {
    this->frame_name = frame_name;
    this->T_world_target = Eigen::Matrix<double, 4, 4>::Identity();
    this->T_offset = Eigen::Matrix<double, 4, 4>::Identity();
}

bool FrameTask::is_SE3_matrix(const Eigen::Matrix<double, 4, 4> &T) {
    // 检查旋转部分是否近似为SO(3)，且最后一行为[0,0,0,1]
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    double det = R.determinant();
    // 检查旋转部分是否正交且行列式约等于1
    if ((std::abs(det - 1.0) > 1e-6)
        || ((R * R.transpose() - Eigen::Matrix3d::Identity()).norm() > 1e-6)) {
        return false;
    }
    // 检查最后一行为[0, 0, 0, 1]
    if ((T.row(3).head(3).array().abs() > 1e-8).any()
        || std::abs(T(3, 3) - 1.0) > 1e-8) {
        return false;
    }
    return true;
}

Eigen::VectorXd FrameTask::compute_error(Configuration &config, float dt) {
    Eigen::Matrix<double, 4, 4> T_target = this->T_world_target;
    // 获取当前帧在世界坐标系下的位姿
    Eigen::Matrix4d T_current = config.FK(this->frame_name);
    // 误差 = T_world_target right-minus T_current

    T_current = T_current * this->T_offset; // 偏移参考

    Eigen::VectorXd error = qpik::utils::right_minus(T_target, T_current);

    // 使用local坐标系下的误差
    return -error / dt;
}

Eigen::MatrixXd FrameTask::compute_jacobian(Configuration &config, float dt) {
    // T_current_offset  伴随换系
    Eigen::MatrixXd Ad = utils::Adjoint(utils::TransInv(this->T_offset));
    return Ad * config.Jb(this->frame_name);
}

void FrameTask::set_orientation_weight(const Eigen::Vector3d &weight) {
    // 姿态权重不能为负
    for (auto &w : weight) {
        if (w < 0.0) {
            throw std::invalid_argument("weight must be non-negative");
        }
    }
    this->weight.head(3) = weight;
}

void FrameTask::set_orientation_weight(double weight) {
    // 姿态权重不能为负
    if (weight < 0.0) {
        throw std::invalid_argument("weight must be non-negative");
    }
    this->weight.head(3) = Eigen::Vector3d::Constant(weight);
}

void FrameTask::set_position_weight(const Eigen::Vector3d &weight) {
    // 位置权重不能为负
    for (auto &w : weight) {
        if (w < 0.0) {
            throw std::invalid_argument("weight must be non-negative");
        }
    }
    this->weight.tail(3) = weight;
}

void FrameTask::set_position_weight(double weight) {
    if (weight < 0.0) {
        throw std::invalid_argument("weight must be non-negative");
    }
    this->weight.tail(3) = Eigen::Vector3d::Constant(weight);
}

void FrameTask::set_target(Eigen::Matrix<double, 4, 4> T_world_target) {
    if (!is_SE3_matrix(T_world_target)) {
        throw std::invalid_argument(
            "Target transformation must be a valid SE(3) matrix");
    }
    this->T_world_target = T_world_target;
}

void FrameTask::set_offset(const Eigen::Matrix<double, 4, 4> &T_offset) {
    if (!is_SE3_matrix(T_offset)) {
        std::cout << "T_offset is not a valid SE(3) matrix" << std::endl;
        return;
    }
    this->T_offset = T_offset;
}

} // namespace qpik