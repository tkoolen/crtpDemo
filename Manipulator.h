//
// Created by Twan Koolen on 8/24/15.
//

#ifndef CRTPDEMO_MANIPULATOR_H
#define CRTPDEMO_MANIPULATOR_H

#include "DrakeSystem.h"

template<typename Derived>
class Manipulator : public DrakeSystem<Derived>
{
public:
  Manipulator(Derived& derived, int num_continuous_states, int num_discrete_states, int num_inputs) :
      DrakeSystem<Derived>(derived, num_continuous_states, num_discrete_states, num_inputs, num_continuous_states + num_discrete_states)
  { };


private:
  friend class DrakeSystem<Derived>;

  template<typename DerivedX>
  YType<DerivedX> output_impl(const Eigen::MatrixBase <DerivedX>& x) const
  {
    return x;
  }
};

#endif //CRTPDEMO_MANIPULATOR_H
