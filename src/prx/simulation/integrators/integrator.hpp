#pragma once
#include "prx/utilities/spaces/space.hpp"

namespace prx
{

typedef std::function<void()> compute_derivative_t;
/**
 * @file integrator.hpp
 * @brief <b>A base class for different integrators.</b>
 *
 * Given the ODE for a plant of the form \dot{x} = f(x,u), the integrator integrates the forward dynamics
 * given a simulation step size.
 *
 * @authors Edgar Granados
 */
class integrator_t
{
public:
  integrator_t(space_t* state_space, space_t* derivative_space, std::function<void()> deriv_f, const double h = 0.01)
    : _h(h)
    , _state_space(state_space)
    , _derivative_space(derivative_space)
    , _compute_derivative(deriv_f)
    , _dim(state_space->size())
  {
  }

  ~integrator_t(){};

  virtual void integrate(const double simulation_step = 0.0) = 0;

  enum integrators
  {
    kEULER = 0,
    kRK4 = 1,
    kDOPRI5 = 2
  };

protected:
  double _h;
  compute_derivative_t _compute_derivative;
  space_t* _state_space;
  space_t* _derivative_space;
  const std::size_t _dim;

private:
};
}  // namespace prx
