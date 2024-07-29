#include "prx/simulation/integrators/euler.hpp"

namespace prx
{

euler_t::euler_t(space_t* state_space, space_t* derivative_space, std::function<void()> deriv_f, const double h)
  : integrator_t(state_space, derivative_space, deriv_f, h)
{
}

euler_t::~euler_t()
{
}

void euler_t::integrate(const double simulation_step)
{
  _compute_derivative();
  _state_space->integrate(_derivative_space, simulation_step);
}
}  // namespace prx