//
// Created by Twan Koolen on 9/3/15.
//

#ifndef CRTPDEMO_DRAKEGEOMETRYUTIL_H_H
#define CRTPDEMO_DRAKEGEOMETRYUTIL_H_H

template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> rpy2rotmat(const Eigen::MatrixBase<Derived>& rpy) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3);
  auto rpy_array = rpy.array();
  auto s = rpy_array.sin();
  auto c = rpy_array.cos();

  Eigen::Matrix<typename Derived::Scalar, 3, 3> R;
  R.row(0) << c(2) * c(1), c(2) * s(1) * s(0) - s(2) * c(0), c(2) * s(1) * c(0) + s(2) * s(0);
  R.row(1) << s(2) * c(1), s(2) * s(1) * s(0) + c(2) * c(0), s(2) * s(1) * c(0) - c(2) * s(0);
  R.row(2) << -s(1), c(1) * s(0), c(1) * c(0);

  return R;
};

template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> angularvel2rpydotMatrix(const Eigen::MatrixBase<Derived>& rpy) {

  typedef typename Derived::Scalar Scalar;
  Scalar p = rpy(1);
  Scalar y = rpy(2);

  using namespace std;
  Scalar sy = sin(y);
  Scalar cy = cos(y);
  Scalar sp = sin(p);
  Scalar cp = cos(p);
  Scalar tp = sp / cp;

  Eigen::Matrix<Scalar, 3, 3> ret;
  ret << cy / cp, sy / cp, 0.0, -sy, cy, 0.0, cy * tp, tp * sy, 1.0;
  return ret;
};

template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> rpydot2angularvelMatrix(const Eigen::MatrixBase<Derived>& rpy) {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3);
  typedef typename Derived::Scalar Scalar;
  Scalar p = rpy(1);
  Scalar y = rpy(2);
  Scalar sp = sin(p);
  Scalar cp = cos(p);
  Scalar sy = sin(y);
  Scalar cy = cos(y);

  Eigen::Matrix<Scalar, 3, 3> ret;
  ret << cp*cy, -sy, 0.0, cp*sy, cy, 0.0, -sp, 0.0, 1.0;
  return ret;
};

#endif //CRTPDEMO_DRAKEGEOMETRYUTIL_H_H
