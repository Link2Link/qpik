#pragma once
#include "qpik/configuration.hpp"

namespace qpik {

struct Constraint {
    Eigen::MatrixXd G; // 约束矩阵 形状为 (m, n)
    Eigen::VectorXd h; // 约束向量 形状为 (m, )

    virtual bool inactive() { return (G.size() == 0 || h.size() == 0); }

    bool active() { return !inactive(); }
};

struct Limit {
    Limit(std::string name) : name(name) {}
    /*
    Compute limit as linearized QP inequalities of the form:
        G * dq  <= h
    */
    virtual Constraint
    compute_qp_inequalities(Configuration &config, float dt) = 0;

    std::string name;
};

} // namespace qpik