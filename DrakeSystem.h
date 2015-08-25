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
      num_outputs(num_outputs)
  { };

  template<typename DerivedX, typename DerivedU>
  typename DerivedX::PlainObject dynamics(double t, const Eigen::MatrixBase<DerivedX>& x, const Eigen::MatrixBase<DerivedU>& u) const
  {
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedX)
    assert(x.rows() == num_continuous_states + num_discrete_states);

    EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedU)
    assert(u.rows() == num_inputs);

    return derived.dynamics(t, x, u);
  }

  template<typename DerivedX>
  Eigen::Matrix<typename DerivedX::Scalar, Eigen::Dynamic, 1> output(const Eigen::MatrixBase<DerivedX>& x) const
  {
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedX)
    assert(x.rows() == num_continuous_states + num_discrete_states);

    return derived.output(x);
  }
};

#endif //CRTPDEMO_DRAKESYSTEM
