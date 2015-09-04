//
// Created by Twan Koolen on 8/24/15.
//
#include <iostream>
#include <vector>
#include "Pendulum.h"
#include "QuadPlantPenn.h"
#include "CoutSystem.h"
#include "systems/CascadeSystem.h"
#include "systems/DrakeSystemDoubleView.h"

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

  // Cascade system
  cout << endl << "Cascade system output:" << endl;
  CoutSystem out(p.getNumOutputs());
  CascadeSystem<Pendulum, CoutSystem> pendulum_cascade(p, out);
  pendulum_cascade.dynamics(t, x, u);

  // quad
  cout << endl << "Quad system output:" << endl;
  QuadPlantPenn quad;
  auto x_quad = VectorXd::Random(quad.getNumStates()).eval();
  auto u_quad = VectorXd::Random(quad.getNumInputs()).eval();
  CoutSystem quad_out(quad.getNumOutputs());
  CascadeSystem<QuadPlantPenn, CoutSystem> quad_cascade(quad, quad_out);
  quad_cascade.dynamics(t, x_quad, u_quad);

  // call both systems by iterating over DrakeSystemDoubleViews
  cout << endl << "Interate over DrakeSystemDoubleViews:" << endl;
  vector<DrakeSystemDoubleView> systems {&pendulum_cascade, &quad_cascade};
  vector<VectorXd> states {x, x_quad};
  vector<VectorXd> inputs {u, u_quad};

  for (int i = 0; i < systems.size(); ++i) {
    cout << systems[i].dynamics(t, states[i], inputs[i]) << endl << endl;
  }

  return 0;
}