//
// Created by Twan Koolen on 8/24/15.
//

#ifndef CRTPDEMO_DRAKESYSTEM
#define CRTPDEMO_DRAKESYSTEM

#include <Eigen/Core>

template<typename Derived>
class DrakeSystem
{
private:
  Derived& derived;
  const unsigned int num_continuous_states;
  const unsigned int num_discrete_states;
  const unsigned int num_inputs;
  const unsigned int num_outputs;

public:
  DrakeSystem(Derived& derived, int num_continuous_states, int num_discrete_states, int num_inputs, int num_outputs) :
      derived(derived),
      num_continuous_states(num_continuous_states),
      num_discrete_states(num_discrete_states),
      num_inputs(num_inputs),
      num_outputs(num_outputs) { };

  template<typename DerivedX, typename DerivedU>
  typename DerivedX::PlainObject dynamics(double t, const Eigen::MatrixBase<DerivedX>& x, const Eigen::MatrixBase<DerivedU>& u) const
  {
    static_assert(DerivedX::ColsAtCompileTime == 1, "x must be a column vector");
    assert(x.rows() == numStates());

    static_assert(DerivedU::ColsAtCompileTime == 1, "u must be a column vector");
    assert(u.rows() == num_inputs);

    auto xdot = derived.dynamics_impl(t, x, u);
    assert(xdot.rows() == numStates());
    static_assert(decltype(xdot)::ColsAtCompileTime == 1, "xdot must be a column vector");
    return xdot;
  }

  template<typename DerivedX>
  Eigen::Matrix<typename DerivedX::Scalar, Eigen::Dynamic, 1> output(const Eigen::MatrixBase<DerivedX>& x) const
  {
    static_assert(DerivedX::ColsAtCompileTime == 1, "x must be a column vector");
    assert(x.rows() == num_continuous_states + num_discrete_states);

    auto y = derived.output_impl(x);
    static_assert(decltype(y)::ColsAtCompileTime == 1, "y must be a column vector");
    assert(y.rows() == num_outputs);
    return y;
  }

  unsigned int numStates() const
  {
    return num_continuous_states + num_discrete_states;
  }
};

#endif //CRTPDEMO_DRAKESYSTEM
