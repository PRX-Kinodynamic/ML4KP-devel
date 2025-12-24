#pragma once

#include "prx/utilities/math/continuous_algebraic_riccati_equation.hpp"

namespace prx
{
// I would prefer to hace a "control" namespace / directory
namespace simulation
{

template <Eigen::Index Dim>
class pid_t
{
public:
  using Array = Eigen::Array<double, Dim, 1>;
  using Vector = Eigen::Vector<double, Dim>;

  using Diff = std::function<Vector(const Vector&, const Vector&)>;

  inline static Diff DefaultDiff = [](const Vector& a, const Vector& b) { return a - b; };

  template <typename Gains>
  pid_t(const Gains kp, const Gains ki, const Gains kd, const Vector ref, const Diff diff = DefaultDiff)
    : _kp(kp)
    , _ki(ki)
    , _kd(kd)
    , _diff(diff)
    , _ref(ref)
    , _integral(Vector::Zero())
    , _deriv(Vector::Zero())
    , _error(Vector::Zero())
  {
  }

  template <Eigen::Index InputDim = Dim, std::enable_if_t<(InputDim == Eigen::Dynamic), bool> = true>
  pid_t(const std::size_t& dim, const Diff diff = DefaultDiff)
    : pid_t(Array::Ones(dim), Array::Ones(dim), Array::Ones(dim), Vector::Zero(dim), diff)
  {
  }

  template <Eigen::Index InputDim = Dim, std::enable_if_t<(InputDim != Eigen::Dynamic), bool> = true>
  pid_t()
    : pid_t(Array::Ones(InputDim), Array::Ones(InputDim), Array::Ones(InputDim), Vector::Zero(InputDim), DefaultDiff)
  {
  }

  virtual ~pid_t() {};

  Array& kp()
  {
    return _kp;
  }

  void kp(const Array kp)
  {
    _kp = kp;
  }

  Array& ki()
  {
    return _ki;
  }

  void ki(const Array ki)
  {
    _ki = ki;
  }

  Array& kd()
  {
    return _kd;
  }

  void kd(const Array kd)
  {
    _kd = kd;
  }

  void goal(const Vector ref)
  {
    _ref = ref;
  }

  // Computes the control u = -K * X;
  inline Vector operator()(const Vector& x) const
  {
    return this->operator()(x, _ref);
  }

  inline Vector operator()(const Vector& x, const Vector& ref)
  {
    const Vector curr_error{ -_diff(x, ref) };

    _deriv = curr_error - _error;
    _integral += _error;
    _error = curr_error;

    return _kp * _error.array() + _ki * _integral.array() + _kd * _deriv.array();
    // return -_K * (x - x_ref);
  }

  Vector _error;
  Vector _integral;
  Vector _deriv;

  Vector _ref;

protected:
  Array _kp, _ki, _kd;
  const Diff _diff;
};
}  // namespace simulation
}  // namespace prx