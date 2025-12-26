#pragma once
#include <Eigen/Dense>
#include <vector>

namespace qpik::utils {

bool NearZero(const double);

Eigen::MatrixXd Normalize(Eigen::MatrixXd);

Eigen::MatrixXd ad(Eigen::VectorXd);

Eigen::MatrixXd Adjoint(const Eigen::MatrixXd &);

Eigen::Matrix3d VecToso3(const Eigen::Vector3d &);

Eigen::Vector3d so3ToVec(const Eigen::MatrixXd &);

Eigen::MatrixXd VecTose3(const Eigen::VectorXd &);

Eigen::VectorXd se3ToVec(const Eigen::MatrixXd &);

Eigen::Vector4d AxisAng3(const Eigen::Vector3d &);

Eigen::Matrix3d MatrixExp3(const Eigen::Matrix3d &);

Eigen::Matrix3d MatrixLog3(const Eigen::Matrix3d &);

Eigen::MatrixXd MatrixExp6(const Eigen::MatrixXd &);

Eigen::MatrixXd MatrixLog6(const Eigen::MatrixXd &);

Eigen::MatrixXd RpToTrans(const Eigen::Matrix3d &, const Eigen::Vector3d &);

std::vector<Eigen::MatrixXd> TransToRp(const Eigen::MatrixXd &);

Eigen::Matrix3d TransToR(const Eigen::MatrixXd &);

Eigen::Vector3d TransTop(const Eigen::MatrixXd &);

Eigen::MatrixXd FKinSpace(
    const Eigen::MatrixXd &, const Eigen::MatrixXd &, const Eigen::VectorXd &);

Eigen::MatrixXd JacobianSpace(const Eigen::MatrixXd &, const Eigen::MatrixXd &);

Eigen::MatrixXd JacobianBody(const Eigen::MatrixXd &, const Eigen::MatrixXd &);

Eigen::MatrixXd TransInv(const Eigen::MatrixXd &);

Eigen::MatrixXd RotInv(const Eigen::MatrixXd &);

Eigen::MatrixXd ProjectToSO3(const Eigen::MatrixXd &);

Eigen::MatrixXd ProjectToSE3(const Eigen::MatrixXd &);

double DistanceToSO3(const Eigen::Matrix3d &);

double DistanceToSE3(const Eigen::Matrix4d &);

Eigen::VectorXd ScrewToAxis(Eigen::Vector3d q, Eigen::Vector3d s, double h);

Eigen::MatrixXd pinv(Eigen::MatrixXd A);

double SO3_err(Eigen::Matrix3d R1, Eigen::Matrix3d R2);

double SE3_err(
    const Eigen::Matrix<double, 4, 4> &T1,
    const Eigen::Matrix<double, 4, 4> &T2);

Eigen::MatrixXd switch_w_v(Eigen::MatrixXd in);

Eigen::VectorXd right_minus(
    const Eigen::Matrix<double, 4, 4> &T1,
    const Eigen::Matrix<double, 4, 4> &T2);
Eigen::VectorXd left_minus(
    const Eigen::Matrix<double, 4, 4> &T1,
    const Eigen::Matrix<double, 4, 4> &T2);

} // namespace qpik::utils
