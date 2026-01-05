#include "qpik/math_utils.hpp"
#include <Eigen/Dense>

namespace qpik::utils {
bool NearZero(const double val) {
    return (std::abs(val) < .000001);
}

Eigen::MatrixXd Normalize(Eigen::MatrixXd V) {
    V.normalize();
    return V;
}

Eigen::MatrixXd ad(Eigen::VectorXd V) {
    Eigen::Matrix3d omgmat = VecToso3(Eigen::Vector3d(V(0), V(1), V(2)));

    Eigen::MatrixXd result(6, 6);
    result.topLeftCorner<3, 3>() = omgmat;
    result.topRightCorner<3, 3>() = Eigen::Matrix3d::Zero(3, 3);
    result.bottomLeftCorner<3, 3>() =
        VecToso3(Eigen::Vector3d(V(3), V(4), V(5)));
    result.bottomRightCorner<3, 3>() = omgmat;
    return result;
}

Eigen::MatrixXd Adjoint(const Eigen::MatrixXd &T) {
    std::vector<Eigen::MatrixXd> R = TransToRp(T);
    Eigen::MatrixXd ad_ret(6, 6);
    ad_ret = Eigen::MatrixXd::Zero(6, 6);
    Eigen::MatrixXd zeroes = Eigen::MatrixXd::Zero(3, 3);
    ad_ret << R[0], zeroes, VecToso3(R[1]) * R[0], R[0];
    return ad_ret;
}

Eigen::Matrix3d VecToso3(const Eigen::Vector3d &omg) {
    Eigen::Matrix3d m_ret;
    m_ret << 0, -omg(2), omg(1), omg(2), 0, -omg(0), -omg(1), omg(0), 0;
    return m_ret;
}

Eigen::Vector3d so3ToVec(const Eigen::MatrixXd &so3mat) {
    Eigen::Vector3d v_ret;
    v_ret << so3mat(2, 1), so3mat(0, 2), so3mat(1, 0);
    return v_ret;
}

Eigen::MatrixXd VecTose3(const Eigen::VectorXd &V) {
    // Separate angular (exponential representation) and linear velocities
    Eigen::Vector3d exp(V(0), V(1), V(2));
    Eigen::Vector3d linear(V(3), V(4), V(5));

    // Fill in values to the appropriate parts of the transformation matrix
    Eigen::MatrixXd m_ret(4, 4);
    m_ret << VecToso3(exp), linear, 0, 0, 0, 0;

    return m_ret;
}

Eigen::VectorXd se3ToVec(const Eigen::MatrixXd &T) {
    Eigen::VectorXd m_ret(6);
    m_ret << T(2, 1), T(0, 2), T(1, 0), T(0, 3), T(1, 3), T(2, 3);

    return m_ret;
}

Eigen::Vector4d AxisAng3(const Eigen::Vector3d &expc3) {
    Eigen::Vector4d v_ret;
    v_ret << Normalize(expc3), expc3.norm();
    return v_ret;
}

Eigen::Matrix3d MatrixExp3(const Eigen::Matrix3d &so3mat) {
    Eigen::Vector3d omgtheta = so3ToVec(so3mat);

    Eigen::Matrix3d m_ret = Eigen::Matrix3d::Identity();
    if (NearZero(so3mat.norm())) {
        return m_ret;
    } else {
        double theta = (AxisAng3(omgtheta))(3);
        Eigen::Matrix3d omgmat = so3mat * (1 / theta);
        return m_ret + std::sin(theta) * omgmat
               + ((1 - std::cos(theta)) * (omgmat * omgmat));
    }
}

Eigen::Matrix3d MatrixLog3(const Eigen::Matrix3d &R) {
    double acosinput = (R.trace() - 1) / 2.0;
    Eigen::MatrixXd m_ret = Eigen::MatrixXd::Zero(3, 3);
    if (acosinput >= 1) return m_ret;
    else if (acosinput <= -1) {
        Eigen::Vector3d omg;
        if (!NearZero(1 + R(2, 2)))
            omg = (1.0 / std::sqrt(2 * (1 + R(2, 2))))
                  * Eigen::Vector3d(R(0, 2), R(1, 2), 1 + R(2, 2));
        else if (!NearZero(1 + R(1, 1)))
            omg = (1.0 / std::sqrt(2 * (1 + R(1, 1))))
                  * Eigen::Vector3d(R(0, 1), 1 + R(1, 1), R(2, 1));
        else
            omg = (1.0 / std::sqrt(2 * (1 + R(0, 0))))
                  * Eigen::Vector3d(1 + R(0, 0), R(1, 0), R(2, 0));
        m_ret = VecToso3(M_PI * omg);
        return m_ret;
    } else {
        double theta = std::acos(acosinput);
        m_ret = theta / 2.0 / sin(theta) * (R - R.transpose());
        return m_ret;
    }
}

Eigen::MatrixXd MatrixExp6(const Eigen::MatrixXd &se3mat) {
    // Extract the angular velocity vector from the transformation matrix
    Eigen::Matrix3d se3mat_cut = se3mat.block<3, 3>(0, 0);
    Eigen::Vector3d omgtheta = so3ToVec(se3mat_cut);

    Eigen::MatrixXd m_ret(4, 4);

    // If negligible rotation, m_Ret = [[Identity, angular velocty ]]
    //									[	0	 ,
    // 1
    //]]
    if (NearZero(omgtheta.norm())) {
        // Reuse previous variables that have our required size
        se3mat_cut = Eigen::MatrixXd::Identity(3, 3);
        omgtheta << se3mat(0, 3), se3mat(1, 3), se3mat(2, 3);
        m_ret << se3mat_cut, omgtheta, 0, 0, 0, 1;
        return m_ret;
    }
    // If not negligible, MR page 105
    else {
        double theta = (AxisAng3(omgtheta))(3);
        Eigen::Matrix3d omgmat = se3mat.block<3, 3>(0, 0) / theta;
        Eigen::Matrix3d expExpand =
            Eigen::MatrixXd::Identity(3, 3) * theta
            + (1 - std::cos(theta)) * omgmat
            + ((theta - std::sin(theta)) * (omgmat * omgmat));
        Eigen::Vector3d linear(se3mat(0, 3), se3mat(1, 3), se3mat(2, 3));
        Eigen::Vector3d GThetaV = (expExpand * linear) / theta;
        m_ret << MatrixExp3(se3mat_cut), GThetaV, 0, 0, 0, 1;
        return m_ret;
    }
}

Eigen::MatrixXd MatrixLog6(const Eigen::MatrixXd &T) {
    Eigen::MatrixXd m_ret(4, 4);
    auto rp = qpik::utils::TransToRp(T);
    Eigen::Matrix3d omgmat = MatrixLog3(rp.at(0));
    Eigen::Matrix3d zeros3d = Eigen::Matrix3d::Zero(3, 3);
    if (NearZero(omgmat.norm())) {
        m_ret << zeros3d, rp.at(1), 0, 0, 0, 0;
    } else {
        double theta = std::acos((rp.at(0).trace() - 1) / 2.0);
        Eigen::Matrix3d logExpand1 =
            Eigen::MatrixXd::Identity(3, 3) - omgmat / 2.0;
        Eigen::Matrix3d logExpand2 =
            (1.0 / theta - 1.0 / std::tan(theta / 2.0) / 2) * omgmat * omgmat
            / theta;
        Eigen::Matrix3d logExpand = logExpand1 + logExpand2;
        m_ret << omgmat, logExpand * rp.at(1), 0, 0, 0, 0;
    }
    return m_ret;
}

Eigen::MatrixXd RpToTrans(const Eigen::Matrix3d &R, const Eigen::Vector3d &p) {
    Eigen::MatrixXd m_ret(4, 4);
    m_ret << R, p, 0, 0, 0, 1;
    return m_ret;
}

std::vector<Eigen::MatrixXd> TransToRp(const Eigen::MatrixXd &T) {
    std::vector<Eigen::MatrixXd> Rp_ret;
    Eigen::Matrix3d R_ret;
    // Get top left 3x3 corner
    R_ret = T.block<3, 3>(0, 0);

    Eigen::Vector3d p_ret(T(0, 3), T(1, 3), T(2, 3));

    Rp_ret.push_back(R_ret);
    Rp_ret.push_back(p_ret);

    return Rp_ret;
}

Eigen::Matrix3d TransToR(const Eigen::MatrixXd &T) {
    return T.block<3, 3>(0, 0);
}

Eigen::Vector3d TransTop(const Eigen::MatrixXd &T) {
    return T.block<3, 1>(0, 3);
}

Eigen::MatrixXd FKinSpace(
    const Eigen::MatrixXd &M,
    const Eigen::MatrixXd &Slist,
    const Eigen::VectorXd &thetaList) {
    Eigen::MatrixXd T = M;
    for (int i = (thetaList.size() - 1); i > -1; i--) {
        T = MatrixExp6(VecTose3(Slist.col(i) * thetaList(i))) * T;
    }
    return T;
}
Eigen::MatrixXd
JacobianSpace(const Eigen::MatrixXd &Slist, const Eigen::MatrixXd &thetaList) {
    Eigen::MatrixXd Js = Slist;
    Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);
    Eigen::VectorXd sListTemp(Slist.col(0).size());
    for (int i = 1; i < thetaList.size(); i++) {
        sListTemp << Slist.col(i - 1) * thetaList(i - 1);
        T = T * MatrixExp6(VecTose3(sListTemp));
        // std::cout << "array: " << sListTemp << std::endl;
        Js.col(i) = Adjoint(T) * Slist.col(i);
    }

    return Js;
}

Eigen::MatrixXd
JacobianBody(const Eigen::MatrixXd &Blist, const Eigen::MatrixXd &thetaList) {
    Eigen::MatrixXd Jb = Blist;
    Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);
    Eigen::VectorXd bListTemp(Blist.col(0).size());
    for (int i = thetaList.size() - 2; i >= 0; i--) {
        bListTemp << Blist.col(i + 1) * thetaList(i + 1);
        T = T * MatrixExp6(VecTose3(-1 * bListTemp));
        // std::cout << "array: " << sListTemp << std::endl;
        Jb.col(i) = Adjoint(T) * Blist.col(i);
    }
    return Jb;
}

Eigen::MatrixXd TransInv(const Eigen::MatrixXd &transform) {
    auto rp = qpik::utils::TransToRp(transform);
    auto Rt = rp.at(0).transpose();
    auto t = -(Rt * rp.at(1));
    Eigen::MatrixXd inv(4, 4);
    inv = Eigen::MatrixXd::Zero(4, 4);
    inv.block(0, 0, 3, 3) = Rt;
    inv.block(0, 3, 3, 1) = t;
    inv(3, 3) = 1;
    return inv;
}

Eigen::MatrixXd RotInv(const Eigen::MatrixXd &rotMatrix) {
    return rotMatrix.transpose();
}

Eigen::MatrixXd ProjectToSO3(const Eigen::MatrixXd &M) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        M, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::MatrixXd R = svd.matrixU() * svd.matrixV().transpose();
    if (R.determinant() < 0)
        // In this case the result may be far from M; reverse sign of 3rd column
        R.col(2) *= -1;
    return R;
}

Eigen::MatrixXd ProjectToSE3(const Eigen::MatrixXd &M) {
    Eigen::Matrix3d R = M.block<3, 3>(0, 0);
    Eigen::Vector3d t = M.block<3, 1>(0, 3);
    Eigen::MatrixXd T = RpToTrans(ProjectToSO3(R), t);
    return T;
}

double DistanceToSO3(const Eigen::Matrix3d &M) {
    if (M.determinant() > 0)
        return (M.transpose() * M - Eigen::Matrix3d::Identity()).norm();
    else
        return 1.0e9;
}

double DistanceToSE3(const Eigen::Matrix4d &T) {
    Eigen::Matrix3d matR = T.block<3, 3>(0, 0);
    if (matR.determinant() > 0) {
        Eigen::Matrix4d m_ret;
        m_ret << matR.transpose() * matR, Eigen::Vector3d::Zero(3), T.row(3);
        m_ret = m_ret - Eigen::Matrix4d::Identity();
        return m_ret.norm();
    } else
        return 1.0e9;
}

Eigen::MatrixXd pinv(Eigen::MatrixXd A) //计算矩阵伪逆
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    double pinvtoler = 1.e-8; // tolerance
    int row = A.rows();
    int col = A.cols();
    int k = std::min(row, col);
    Eigen::MatrixXd X = Eigen::MatrixXd::Zero(col, row);
    Eigen::MatrixXd singularValues_inv = svd.singularValues(); //奇异值
    Eigen::MatrixXd singularValues_inv_mat = Eigen::MatrixXd::Zero(col, row);
    for (long i = 0; i < k; ++i) {
        if (singularValues_inv(i) > pinvtoler)
            singularValues_inv(i) = 1.0 / singularValues_inv(i);
        else
            singularValues_inv(i) = 0;
    }
    for (long i = 0; i < k; ++i) {
        singularValues_inv_mat(i, i) = singularValues_inv(i);
    }
    X = (svd.matrixV()) * (singularValues_inv_mat)
        * (svd.matrixU().transpose());

    return X;
}

Eigen::VectorXd ScrewToAxis(Eigen::Vector3d q, Eigen::Vector3d s, double h) {
    Eigen::VectorXd axis(6);
    axis.segment(0, 3) = s;
    axis.segment(3, 3) = q.cross(s) + (h * s);
    return axis;
}

double SO3_err(Eigen::Matrix3d R1, Eigen::Matrix3d R2) {
    Eigen::Matrix3d Rerr = R2 * R1.transpose();
    return so3ToVec(MatrixLog3(Rerr)).norm();
}

double SE3_err(
    const Eigen::Matrix<double, 4, 4> &T1,
    const Eigen::Matrix<double, 4, 4> &T2) {
    Eigen::Matrix<double, 4, 4> Terr = T2 * TransInv(T1);
    return se3ToVec(MatrixLog6(Terr)).norm();
}

Eigen::MatrixXd switch_w_v(Eigen::MatrixXd in) {
    Eigen::MatrixXd out;
    out = in;
    out.row(0) = in.row(3);
    out.row(1) = in.row(4);
    out.row(2) = in.row(5);
    out.row(3) = in.row(0);
    out.row(4) = in.row(1);
    out.row(5) = in.row(2);
    return out;
}

Eigen::VectorXd right_minus(
    const Eigen::Matrix<double, 4, 4> &T1,
    const Eigen::Matrix<double, 4, 4> &T2) {
    Eigen::Matrix<double, 4, 4> result = TransInv(T2) * T1;
    return se3ToVec(MatrixLog6(result));
}

Eigen::VectorXd left_minus(
    const Eigen::Matrix<double, 4, 4> &T1,
    const Eigen::Matrix<double, 4, 4> &T2) {
    Eigen::Matrix<double, 4, 4> result = T1 * TransInv(T2);
    return se3ToVec(MatrixLog6(result));
}

Eigen::Matrix3d RotX(const double &theta) {
    Eigen::Matrix3d R;
    R << 1, 0, 0, 0, cos(theta), -sin(theta), 0, sin(theta), cos(theta);
    return R;
}

Eigen::Matrix3d RotY(const double &theta) {
    Eigen::Matrix3d R;
    R << cos(theta), 0, sin(theta), 0, 1, 0, -sin(theta), 0, cos(theta);
    return R;
}

Eigen::Matrix3d RotZ(const double &theta) {
    Eigen::Matrix3d R;
    R << cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0, 0, 1;
    return R;
}

} // namespace qpik::utils