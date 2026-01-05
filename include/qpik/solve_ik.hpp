#pragma once
#include "qpik/configuration.hpp"
#include "qpik/limits/limit.hpp"
#include "qpik/tasks/task.hpp"

#include <OsqpEigen/OsqpEigen.h>

namespace qpik {

/*
OSQP中QP问题的标准数学公式如下：

minimize    (1/2) * x^T H x + c^T x

subject to  lb <= A * x <= ub

其中，x为待优化变量（向量），H为二次项系数的对称半正定矩阵（目标函数Hessian矩阵），
c为目标函数的线性系数向量，lb和ub分别为不等式约束的下界和上界，A为等式约束的系数矩阵。
*/

struct OSQP_Problem {
    Eigen::MatrixXd H; // 目标函数的Hessian矩阵 形状为（nv, nv）
    Eigen::VectorXd c; // 目标函数的线性向量 形状为（nv, ）

    Eigen::MatrixXd A; // 约束矩阵 形状为（m, nv）
    Eigen::VectorXd lb;
    Eigen::VectorXd ub;
};

/*
QP（Quadratic Programming，二次规划）问题的标准数学公式如下：

minimize    (1/2) * x^T H x + c^T x

subject to  G x <= h
            A x  = b

其中，
- x 为待优化变量（向量）
- H 为二次项系数的对称半正定矩阵（目标函数Hessian矩阵）
- c 为目标函数的线性系数向量
- G 和 h 定义了不等式约束（Gx <= h）
- A 和 b 定义了等式约束（Ax = b）
*/

struct QP_Problem {
    Eigen::MatrixXd H; // 目标函数的Hessian矩阵 形状为（nv, nv）
    Eigen::VectorXd c; // 目标函数的线性向量 形状为（nv, ）

    Eigen::MatrixXd G; // 约束矩阵 形状为（m, nv）
    Eigen::VectorXd h; // 约束向量 形状为（m, ）

    Eigen::MatrixXd A; // 约束矩阵 形状为（n, nv）
    Eigen::VectorXd b; // 约束向量 形状为（n, ）

    // 将标准QP问题转换为OSQP形式
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
