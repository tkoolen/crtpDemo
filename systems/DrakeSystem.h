//
// Created by Twan Koolen on 8/24/15.
//

#ifndef CRTPDEMO_DRAKESYSTEM
#define CRTPDEMO_DRAKESYSTEM

#include <Eigen/Core>
#include "DrakeSystemTypedefs.h"

template<typename Derived>
class DrakeSystem
{
private:
  Derived& derived;
  const size_t num_continuous_states;
  const size_t num_discrete_states;
  const size_t num_inputs;
  const size_t num_outputs;

public:
  DrakeSystem(size_t num_continuous_states, size_t num_discrete_states, size_t num_inputs, size_t num_outputs) :
      derived(static_cast<Derived&>(*this)),
      num_continuous_states(num_continuous_states),
      num_discrete_states(num_discrete_states),
      num_inputs(num_inputs),
      num_outputs(num_outputs) { };

  template<typename DerivedX, typename DerivedU>
  XdotType<DerivedX> dynamics(double t, const Eigen::MatrixBase<DerivedX>& x, const Eigen::MatrixBase<DerivedU>& u) const
  {
    static_assert(DerivedX::ColsAtCompileTime == 1, "x must be a column vector");
    int rows = x.rows();
    int this_rows = getNumStates();
    assert(x.rows() == getNumStates());
    static_assert(DerivedU::ColsAtCompileTime == 1, "u must be a column vector");
    assert(u.rows() == num_inputs);

    auto xdot = derived.dynamics_impl(t, x, u);

    assert(xdot.rows() == getNumStates());
    static_assert(decltype(xdot)::ColsAtCompileTime == 1, "xdot must be a column vector");
    return xdot;
  }

  template<typename DerivedX>
  YType<DerivedX> output(const Eigen::MatrixBase<DerivedX>& x) const
  {
    static_assert(DerivedX::ColsAtCompileTime == 1, "x must be a column vector");
    assert(x.rows() == getNumStates());

    auto y = derived.output_impl(x);

    static_assert(decltype(y)::ColsAtCompileTime == 1, "y must be a column vector");
    assert(y.rows() == num_outputs);
    return y;
  }

  size_t getNumStates() const
  {
    return num_continuous_states + num_discrete_states;
  }

  size_t getNumContinuousStates() const {
    return num_continuous_states;
  }

  size_t getNumDiscreteStates() const {
    return num_discrete_states;
  }

  size_t getNumInputs() const {
    return num_inputs;
  }

  size_t getNumOutputs() const {
    return num_outputs;
  }
};

#endif //CRTPDEMO_DRAKESYSTEM
