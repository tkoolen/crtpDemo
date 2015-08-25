//
// Created by Twan Koolen on 8/24/15.
//
#include <iostream>
#include "Pendulum.h"
#include <unsupported/Eigen/AutoDiff>
#include "drakeGradientUtil.h"

using namespace Eigen;
using namespace std;

int main () {
  Pendulum p1;
  DrakeSystem<Pendulum>& p = p1;

  // double
  double t = 1.0;
  auto x = Vector2d::Random().eval();
  auto u = Matrix<double, 1, 1>::Random().eval();
  auto xdot = p.dynamics(t, x, u);
  cout << "xdot:\n" << xdot << endl << endl;
  auto y = p.output(x);
  cout << "y:\n" << y << endl << endl;

  // AutoDiffScalar
  typedef AutoDiffScalar<Vector3d> TaylorVar;
  Matrix<TaylorVar, 2, 1> x_taylor = x.cast<TaylorVar>();
  gradientMatrixToAutoDiff(Eigen::Matrix<double, 2, 3>::Identity().eval(), x_taylor);
  Matrix<TaylorVar, 1, 1> u_taylor = u.cast<TaylorVar>();
  u_taylor(0).derivatives() << 0.0, 0.0, 1.0;
  auto xdot_taylor = p.dynamics(t, x_taylor, u_taylor);
  std::cout << "gradient:\n" << autoDiffToGradientMatrix(xdot_taylor) << std::endl;

  return 0;
}