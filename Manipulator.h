//
// Created by Twan Koolen on 8/24/15.
//

#ifndef CRTPDEMO_SUPERBUILD_MANIPULATOR_H
#define CRTPDEMO_SUPERBUILD_MANIPULATOR_H

#include "DrakeSystem.h"

template<typename Derived>
class Manipulator : public DrakeSystem<Derived>
{
public:
  Manipulator(Derived& derived, int num_continuous_states, int num_discrete_states, int num_inputs) :
      DrakeSystem<Derived>(derived, num_continuous_states, num_discrete_states, num_inputs, num_continuous_states + num_discrete_states)
  { };


  template<typename DerivedX>
  Eigen::Matrix<typename DerivedX::Scalar, Eigen::Dynamic, 1> output(const Eigen::MatrixBase <DerivedX>& x) const
  {
    return x;
  }
};


#endif //CRTPDEMO_SUPERBUILD_MANIPULATOR_H
