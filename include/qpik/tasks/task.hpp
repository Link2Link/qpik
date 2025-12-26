#pragma once
#include "qpik/math_utils.hpp"
#include <Eigen/Dense>
#include "qpik/configuration.hpp"
namespace qpik {

struct Objective {
    Eigen::MatrixXd H; // 目标函数的Hessian矩阵 形状为（nv, nv）
    Eigen::VectorXd c; // 目标函数的常数项 形状为（nv, ）

    double value(Eigen::VectorXd dq);
    Eigen::VectorXd gradient(Eigen::VectorXd dq);
};

struct BaseTask {
    BaseTask(std::string name = "BaseTask") : name(name) {}
    virtual Objective compute_qp_objective(Configuration &config, float dt) = 0;
    std::string name;
};

struct Task : public BaseTask {
    Task(
        std::string name,
        Eigen::VectorXd weight = Eigen::VectorXd::Zero(0),
        double gain = 1.0,
        double lm_damping = 1e-4) {
        this->name = name;
        if (gain < 0.0 || gain > 1.0) {
            throw std::invalid_argument("gain must be between 0 and 1");
        }
        if (lm_damping < 0) {
            throw std::invalid_argument("lm_damping must be greater than 0");
        }
        this->weight = weight;
        this->gain = gain;
        this->lm_damping = lm_damping;
    }

    virtual Eigen::VectorXd compute_error(Configuration &config, float dt) = 0;
    virtual Eigen::MatrixXd
    compute_jacobian(Configuration &config, float dt) = 0;

    Objective compute_qp_objective(Configuration &config, float dt) override;

    Eigen::VectorXd weight;
    double gain;
    double lm_damping;
};

} // namespace qpik