#pragma once
#include "qpik/configuration.hpp"
#include "qpik/limits/limit.hpp"
#include "qpik/tasks/task.hpp"

#include <OsqpEigen/OsqpEigen.h>

namespace qpik {

// OSQPEigen只有不等式约束
struct OSQP_Problem {
    Eigen::MatrixXd H; // 目标函数的Hessian矩阵 形状为（nv, nv）
    Eigen::VectorXd c; // 目标函数的线性向量 形状为（nv, ）

    Eigen::MatrixXd A; // 约束矩阵 形状为（m, nv）
    Eigen::VectorXd lb;
    Eigen::VectorXd ub;
};

struct QP_Problem {
    Eigen::MatrixXd H; // 目标函数的Hessian矩阵 形状为（nv, nv）
    Eigen::VectorXd c; // 目标函数的线性向量 形状为（nv, ）

    Eigen::MatrixXd G; // 约束矩阵 形状为（m, nv）
    Eigen::VectorXd h; // 约束向量 形状为（m, ）

    Eigen::MatrixXd A; // 约束矩阵 形状为（n, nv）
    Eigen::VectorXd b; // 约束向量 形状为（n, ）

    OSQP_Problem to_osqp_problem(double eps = 1e-6);
};

QP_Problem construct_qp_problem(
    Configuration &config,
    const std::vector<BaseTask *> &tasks,
    const std::vector<Limit *> &limits,
    const std::vector<Task *> &constraints,
    float dt,
    float damping = 1e-12);

QP_Problem construct_qp_problem(
    Configuration &config,
    const std::vector<BaseTask *> &tasks,
    const std::vector<Limit *> &limits,
    float dt,
    float damping = 1e-12);

QP_Problem construct_qp_problem(
    Configuration &config,
    const std::vector<BaseTask *> &tasks,
    float dt,
    float damping = 1e-12);

int solve_qp_problem(QP_Problem &problem, Eigen::VectorXd &x);

} // namespace qpik
