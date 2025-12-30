#include "qpik/solve_ik.hpp"

namespace qpik {

Objective _compute_qp_objective(
    Configuration &config,
    const std::vector<BaseTask *> &tasks,
    float dt,
    float damping) {
    int nv = config.model_.nv;
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(nv, nv);
    Eigen::VectorXd c = Eigen::VectorXd::Zero(nv);
    for (const auto &task : tasks) {
        Objective obj = task->compute_qp_objective(config, dt);
        H += obj.H;
        c += obj.c;
    }
    H += damping * Eigen::MatrixXd::Identity(nv, nv); // LM型正则化

    Objective obj;
    obj.H = H;
    obj.c = c;
    return obj;
}

void _compute_qp_inequalities(
    Configuration &config,
    const std::vector<Limit *> &limits,
    float dt,
    Eigen::MatrixXd &G,
    Eigen::VectorXd &h) {
    int nv = config.model_.nv;

    G.resize(0, config.model_.nv); // 确保G和h初始为零行
    h.resize(0);
    for (const auto &limit : limits) {
        Constraint constraint = limit->compute_qp_inequalities(config, dt);
        // 竖向拼接G和h
        int old_rows = G.rows();
        int add_rows = constraint.G.rows();
        if (add_rows > 0) {
            Eigen::MatrixXd newG(G.rows() + constraint.G.rows(), G.cols());
            Eigen::VectorXd newh(h.size() + constraint.h.size());
            if (old_rows > 0) {
                newG.topRows(old_rows) = G;
                newh.head(h.size()) = h;
            }
            newG.bottomRows(add_rows) = constraint.G;
            newh.tail(constraint.h.size()) = constraint.h;
            G = newG;
            h = newh;
        }
    }
}

void _compute_qp_equalities(
    Configuration &config,
    const std::vector<Task *> &constraints,
    float dt,
    Eigen::MatrixXd &A,
    Eigen::VectorXd &b) {
    int nv = config.model_.nv;
    A.resize(0, config.model_.nv);
    b.resize(0);
    for (const auto &task : constraints) {
        Eigen::MatrixXd jacobian = task->compute_jacobian(config, dt);
        Eigen::VectorXd feedback =
            -task->gain * task->compute_error(config, dt);

        int jacobian_rows = jacobian.rows();
        int old_rows = A.rows();

        // 竖向拼接A和b
        if (jacobian_rows > 0) {
            Eigen::MatrixXd newA(old_rows + jacobian_rows, A.cols());
            Eigen::VectorXd newb(b.size() + feedback.size());
            if (old_rows > 0) {
                newA.topRows(old_rows) = A;
                newb.head(b.size()) = b;
            }
            newA.bottomRows(jacobian_rows) = jacobian;
            newb.tail(feedback.size()) = feedback;
            A = newA;
            b = newb;
        }
    }
}

QP_Problem construct_qp_problem(
    Configuration &config,
    const std::vector<BaseTask *> &tasks,
    const std::vector<Limit *> &limits,
    const std::vector<Task *> &constraints,
    float dt,
    float damping) {
    QP_Problem problem;

    // 计算目标函数
    Objective obj = _compute_qp_objective(config, tasks, dt, damping);
    problem.H = obj.H;
    problem.c = obj.c;

    // 计算不等式约束
    if (!limits.empty()) {
        _compute_qp_inequalities(config, limits, dt, problem.G, problem.h);
    }

    // 计算等式约束
    if (!constraints.empty()) {
        _compute_qp_equalities(config, constraints, dt, problem.A, problem.b);
    }

    return problem;
}

QP_Problem construct_qp_problem(
    Configuration &config,
    const std::vector<BaseTask *> &tasks,
    const std::vector<Limit *> &limits,
    float dt,
    float damping) {
    std::vector<Task *> constraints;
    return construct_qp_problem(
        config, tasks, limits, constraints, dt, damping);
}

QP_Problem construct_qp_problem(
    Configuration &config,
    const std::vector<BaseTask *> &tasks,
    float dt,
    float damping) {
    std::vector<Task *> constraints;
    std::vector<Limit *> limits;
    return construct_qp_problem(
        config, tasks, limits, constraints, dt, damping);
}

OSQP_Problem QP_Problem::to_osqp_problem(double eps) {
    OSQP_Problem os;
    os.H = this->H;
    os.c = this->c;

    int nv = this->H.rows();
    int nin = this->G.rows();
    int neq = this->A.rows();

    int n = nin + neq;

    if (n == 0) {
        // 无约束
        return os;
    }

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, nv);
    Eigen::VectorXd lb =
        Eigen::VectorXd::Constant(n, -std::numeric_limits<double>::infinity());
    Eigen::VectorXd ub =
        Eigen::VectorXd::Constant(n, std::numeric_limits<double>::infinity());

    if (nin > 0) {
        // 不等式约束
        A.topRows(nin) = this->G;
        ub.head(nin) = this->h;
    }

    if (neq > 0) {
        // 等式约束
        A.bottomRows(neq) = this->A;
        ub.tail(neq) = this->b + Eigen::VectorXd::Constant(neq, eps);
        lb.tail(neq) = this->b - Eigen::VectorXd::Constant(neq, eps);
    }

    os.A = A;
    os.lb = lb;
    os.ub = ub;

    return os;
}

int solve_qp_problem(QP_Problem &problem, Eigen::VectorXd &x) {
    OSQP_Problem os = problem.to_osqp_problem();

    // std::cout << "OSQP H: " << os.H << std::endl;
    // std::cout << "OSQP c: " << os.c.transpose() << std::endl;
    // std::cout << "OSQP A: " << os.A << std::endl;
    // std::cout << "OSQP lb: " << os.lb.transpose() << std::endl;
    // std::cout << "OSQP ub: " << os.ub.transpose() << std::endl;

    OsqpEigen::Solver solver;
    int dim = os.H.rows(); // 变量维度
    x = Eigen::VectorXd::Zero(dim);
    int nin = os.A.rows(); // 不等式约束维度

    solver.settings()->setWarmStart(false);
    solver.settings()->setVerbosity(false);
    solver.data()->setNumberOfVariables(dim);
    solver.data()->setNumberOfConstraints(nin);
    Eigen::SparseMatrix<double> H_sparse = os.H.sparseView();
    Eigen::SparseMatrix<double> A_sparse = os.A.sparseView();

    if (!solver.data()->setHessianMatrix(H_sparse)) {
        std::cerr << "Error setting Hessian matrix" << std::endl;
        return -1;
    }

    if (!solver.data()->setGradient(os.c)) {
        std::cerr << "Error setting gradient vector" << std::endl;
        return -1;
    }

    if (nin == 0) { goto solve; }
    if (!solver.data()->setLinearConstraintsMatrix(A_sparse)) {
        std::cerr << "Error setting constraint matrix" << std::endl;
        return -1;
    }

    if (!solver.data()->setLowerBound(os.lb)) {
        std::cerr << "Error setting lower bounds" << std::endl;
        return -1;
    }

    if (!solver.data()->setUpperBound(os.ub)) {
        std::cerr << "Error setting upper bounds" << std::endl;
        return -1;
    }

solve:
    if (!solver.initSolver()) {
        std::cerr << "Solver initialization failed" << std::endl;
        return -1;
    }

    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        std::cerr << "Solver failed" << std::endl;
        return -1;
    }

    auto temp = solver.getSolution();
    x = temp;

    solver.clearSolver();

    return 0;
}

} // namespace qpik