#pragma once
#include "prx/simulation/observer.hpp"

namespace prx
{
namespace simulation
{

// Given {xdot, ydot, (...)}, compute the velocity v_observed = sqrt(xdot^2 + ydot^2 + (...))
class velocity_observer_t : public observer_t
{
public:
  velocity_observer_t(std::vector<double*>& input_addresses)
    : observer_t(), _velocity(0.0), _input_addresses(input_addresses)
  {
    _observer_memory = { &_velocity };
    _observed_space = new prx::space_t("E", _observer_memory, "velocity_observer_space");
  }

  virtual void observe(const std::shared_ptr<plant_t> plant) override final
  {
    double observed_velocity{ 0 };
    for (auto address : _input_addresses)
    {
      observed_velocity += std::pow((*address), 2);
    }
    _velocity = std::sqrt(observed_velocity);
  }

private:
  double _velocity;
  const std::vector<double*> _input_addresses;
  const double* _output_address;
};

}  // namespace simulation
}  // namespace prx