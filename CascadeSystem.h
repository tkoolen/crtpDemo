//
// Created by Twan Koolen on 9/3/15.
//

#ifndef CRTPDEMO_CascadeSystem_H
#define CRTPDEMO_CascadeSystem_H

#include "DrakeSystem.h"

template <typename Sys1, typename Sys2>
class CascadeSystem : public DrakeSystem<CascadeSystem<Sys1, Sys2>>{
private:
  DrakeSystem<Sys1> sys1;
  DrakeSystem<Sys2> sys2;
  typedef DrakeSystem<CascadeSystem<Sys1, Sys2>> Base;

public:
  using Base::getNumContinuousStates;

  CascadeSystem(const DrakeSystem<Sys1>& sys1, const DrakeSystem<Sys2>& sys2) :
      DrakeSystem<CascadeSystem>(*this, sys1.getNumContinuousStates() + sys2.getNumContinuousStates(), sys1.getNumDiscreteStates() + sys2.getNumDiscreteStates(), sys1.getNumInputs(), sys2.getNumOutputs()),
  sys1(sys1), sys2(sys2){ };

private:
  friend class DrakeSystem<CascadeSystem>;

  template<typename DerivedX, typename DerivedU>
  XdotType<DerivedX> dynamics_impl(double t, const Eigen::MatrixBase<DerivedX>& x, const Eigen::MatrixBase<DerivedU>& u) const {
    typename DerivedX::PlainObject xdot(getNumContinuousStates());

    auto x1 = x.head(sys1.getNumContinuousStates());
    auto x2 = x.tail(sys2.getNumContinuousStates());
    auto y1 = sys1.output(x1);

    auto xdot1 = xdot.head(sys1.getNumContinuousStates());
    auto xdot2 = xdot.tail(sys2.getNumContinuousStates());

    xdot1 = sys1.dynamics(t, x1, u);
    xdot2 = sys2.dynamics(t, x2, y1);

    return xdot;
  }

  template <typename DerivedX>
  YType<DerivedX> output_impl(const Eigen::MatrixBase<DerivedX>& x) const {
    return sys2.output(x.tail(sys2.getNumStates()));
  }
};


#endif //CRTPDEMO_CascadeSystem_H
