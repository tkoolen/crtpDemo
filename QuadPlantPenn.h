//
// Created by Twan Koolen on 9/3/15.
//

#ifndef CRTPDEMO_QUADPLANTPENN_H
#define CRTPDEMO_QUADPLANTPENN_H


#include "DrakeSystem.h"
#include "drakeGeometryUtil.h"
#include "drakeGradientUtil.h"
#include <unsupported/Eigen/AutoDiff>

class QuadPlantPenn : public DrakeSystem<QuadPlantPenn>
{
public:
  typedef DrakeSystem<QuadPlantPenn> Base;
  using Base::getNumStates;

  QuadPlantPenn() : DrakeSystem<QuadPlantPenn>(*this, 12, 0, 4, 12) { };

private:
  friend class DrakeSystem<QuadPlantPenn>;

  template<typename DerivedX, typename DerivedU>
  XdotType<DerivedX> dynamics_impl(double t, const Eigen::MatrixBase<DerivedX> &x, const Eigen::MatrixBase<DerivedU> &u) {
    using namespace Eigen;

    typedef typename DerivedX::Scalar Scalar;
    typedef Matrix<Scalar, 3, 1> Vector3;
    typedef Matrix<Scalar, 3, 3> Matrix3;

    Scalar phi = x(3);
    Scalar theta = x(4);
    Scalar psi = x(5);
    Scalar phidot = x(9);
    Scalar thetadot = x(10);
    Scalar psidot = x(11);

    Scalar w1 = u(0);
    Scalar w2 = u(1);
    Scalar w3 = u(2);
    Scalar w4 = u(3);

    Vector3 rpy;
    rpy << phi, theta, psi;

    Vector3 rpydot;
    rpydot << phidot, thetadot, psidot;

    typedef AutoDiffScalar<Matrix<Scalar, 1, 1>> TimeAutoDiffType;
    auto rpy_autodiff = rpy.template cast<TimeAutoDiffType>().eval();
    gradientMatrixToAutoDiff(rpydot, rpy_autodiff);

    auto R_autodiff = rpy2rotmat(rpy_autodiff);
    Matrix3 R = autoDiffToValueMatrix(R_autodiff);

    Scalar F1 = kf * w1;
    Scalar F2 = kf * w2;
    Scalar F3 = kf * w3;
    Scalar F4 = kf * w4;

    Scalar M1 = km * w1;
    Scalar M2 = km * w2;
    Scalar M3 = km * w3;
    Scalar M4 = km * w4;

    Vector3 gvec;
    gvec << 0, 0, -m * g;
    Vector3 forcevec;
    forcevec << 0, 0, F1 + F2 + F3 + F4;
    Vector3 xyz_ddot = (1.0 / m) * (gvec + R * forcevec);

    Vector3 pqr = R.adjoint() * rpydot2angularvelMatrix(rpy) * rpydot; // angular velocity in body frame

    Vector3 tau;
    tau << L * (F2 - F4), L * (F3 - F1), (M1 - M2 + M3 - M4);
    Vector3 pqr_dot = I.ldlt().solve(tau - pqr.cross(I * pqr)); // angular velocity rate

    auto pqr_autodiff = pqr.template cast<TimeAutoDiffType>().eval();
    gradientMatrixToAutoDiff(pqr_dot, pqr_autodiff);

    auto rpydot_autodiff = angularvel2rpydotMatrix(rpy_autodiff) * R_autodiff * pqr_autodiff;
    Vector3 rpy_ddot = autoDiffToGradientMatrix(rpydot_autodiff);

    Matrix<Scalar, 6, 1> qdd;
    qdd << xyz_ddot, rpy_ddot;
    Matrix<Scalar, 6, 1> qd;
    qd = x.template segment<6>(6);

    XdotType<DerivedX> xdot(getNumStates(), 1);

    xdot << qd, qdd;

    return xdot;
  }

  template<typename DerivedX>
  YType<DerivedX> output_impl(const Eigen::MatrixBase <DerivedX>& x) const
  {
    return x;
  }

private:
  double m = 1.0;
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  double g = 9.81;
  double L = 0.1750;
  double kf = 1; // 6.11*10^-8;
  double km = 0.0245;
};




#endif //CRTPDEMO_QUADPLANTPENN_H
