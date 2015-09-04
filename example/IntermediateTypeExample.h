//
// Created by Twan Koolen on 8/24/15.
//

#ifndef CRTPDEMO_IntermediateTypeExample_H
#define CRTPDEMO_IntermediateTypeExample_H

#include "systems/DrakeSystem.h"

template<typename Derived>
class IntermediateTypeExample : public DrakeSystem<Derived>
{
public:
  IntermediateTypeExample(Derived& derived, int num_continuous_states, int num_discrete_states, int num_inputs) :
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

#endif //CRTPDEMO_IntermediateTypeExample_H
