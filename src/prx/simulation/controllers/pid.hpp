#pragma once

#include "prx/simulation/controller.hpp"

namespace prx
{
template <typename Control, typename Error, typename State, typename Gain>
class pid_t : public controller_t
{
public:
  using ErrorFunction = std::function<Error()>;

  pid_t(const system_ptr_t& _sys_ptr, const std::string _name,  // no-lint
        ErrorFunction error_function, const Gain kp, const Gain ki, const Gain kd)
    : controller_t(_sys_ptr, _name), _error_function(error_function), _kp(kp), _ki(ki), _kd(kd)
  {
  }

  virtual ~pid_t()
  {
  }

  using controller_t::compute_controls;
  virtual void compute_controls() override
  {
    _error_current = _error_function();
    _error_accum += _error_current;

    _u = _kp * _error_current + _ki * _error_accum + _kd * (_error_current - _error_prev);
    // PRX_DEBUG_VAR_1(_u.transpose());
    plant->get_control_space()->copy_from(_u);
    plant->get_control_space()->enforce_bounds();

    _error_prev = _error_current;
  }

protected:
  ErrorFunction _error_function;
  Gain _kp, _ki, _kd;

  Error _error_current;
  Error _error_accum;
  Error _error_prev;

  Control _u;
};
}  // namespace prx