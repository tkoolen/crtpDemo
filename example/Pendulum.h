//
// Created by Twan Koolen on 8/24/15.
//

#ifndef CRTPDEMO_PENDULUM_H
#define CRTPDEMO_PENDULUM_H

#include "IntermediateTypeExample.h"

class Pendulum : public IntermediateTypeExample<Pendulum>
{
public:
  Pendulum() :
      IntermediateTypeExample<Pendulum>(*this, 2, 0, 1),
      m(1.0), // kg
      l(.5),  // m
      b(0.1), // kg m^2 /s
      lc(.5), // m
      I(.25), // m*l^2; % kg*m^2
      g(9.81) // m/s^2
  { };

private:
  friend class DrakeSystem<Pendulum>;

  template<typename DerivedX, typename DerivedU>
  XdotType<DerivedX> dynamics_impl(double t, const Eigen::MatrixBase <DerivedX>& x, const Eigen::MatrixBase <DerivedU>& u) const
  {
    // scalar type constraint demo:
//    static_assert(std::is_floating_point<typename DerivedX::Scalar>::value, "Method only accepts floating point value types");
//    static_assert(std::is_same<typename DerivedX::Scalar, double>::value, "Method only accepts double");

    typename DerivedX::PlainObject xdot(2, 1);
    xdot(0) = x(1);
    xdot(1) = (u(0) - m * g * lc * sin(x(0)) - b * x(1)) / I;
    return xdot;
  }

private:
  double m, l, b, lc, I, g;
};


#endif //CRTPDEMO_PENDULUM_H
