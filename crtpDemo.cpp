//
// Created by Twan Koolen on 8/24/15.
//
#include <iostream>
#include "Pendulum.h"
#include "CoutSystem.h"
#include "CascadeSystem.h"
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
  Matrix<TaylorVar, 3, 1> taylorvars;
  auto x_taylor = taylorvars.head<2>();
  auto u_taylor = taylorvars.tail<1>();
  x_taylor = x.cast<TaylorVar>();
  u_taylor = u.cast<TaylorVar>();
  gradientMatrixToAutoDiff(Matrix3d::Identity().eval(), taylorvars);
  auto xdot_taylor = p.dynamics(t, x_taylor, u_taylor);
  std::cout << "gradient:\n" << autoDiffToGradientMatrix(xdot_taylor) << std::endl;

  cout << endl << "Cascade system output:" << endl;
  CoutSystem out(p.getNumOutputs());
  CascadeSystem<Pendulum, CoutSystem> cascade(p, out);
  cascade.dynamics(t, x, u);

  return 0;
}