//
// Created by Twan Koolen on 9/3/15.
//

#ifndef CRTPDEMO_COUTSYSTEM_H
#define CRTPDEMO_COUTSYSTEM_H

#include "systems/DrakeSystem.h"
#include <iostream>

class CoutSystem : public DrakeSystem<CoutSystem>
{
public:
  CoutSystem(int num_inputs) : DrakeSystem<CoutSystem>(0, 0, num_inputs, 0) {};

private:
  friend class DrakeSystem<CoutSystem>;

  template<typename DerivedX, typename DerivedU>
  typename DerivedX::PlainObject dynamics_impl(double t, const Eigen::MatrixBase<DerivedX>& x, const Eigen::MatrixBase<DerivedU>& u) const {
    std::cout << u << std::endl;
    typename DerivedX::PlainObject xdot(0, 1);
    return xdot;
  }

  template<typename DerivedX>
  Eigen::Matrix<typename DerivedX::Scalar, Eigen::Dynamic, 1> output_impl(const Eigen::MatrixBase<DerivedX>& x) const {
    return Eigen::Matrix<typename DerivedX::Scalar, Eigen::Dynamic, 1>::Zero(0);
  };
};

#endif //CRTPDEMO_COUTSYSTEM_H
