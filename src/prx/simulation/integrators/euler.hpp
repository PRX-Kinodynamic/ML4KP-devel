#pragma once

#include "prx/utilities/spaces/space.hpp"
#include "prx/simulation/integrators/integrator.hpp"

namespace prx
{

class euler_t : public integrator_t
{
public:
  euler_t(space_t* state_space, space_t* derivative_space, std::function<void()> deriv_f, const double h);

  virtual ~euler_t();

  void integrate(const double simulation_step = 0.0) override;

protected:
private:
};
}  // namespace prx
