#pragma once

#include "prx/simulation/controller.hpp"

namespace prx
{

class random_walk_t : public controller_t
{
public:
  random_walk_t(system_ptr_t _plant, std::string _name = "random_walk") : controller_t(_plant, _name)
  {
    counter = 0;
    current_control = get_control_space()->make_point();
  };
  virtual ~random_walk_t();

  virtual void compute_controls() override;

  virtual void propagate(const double simulation_step) override;

  double counter;
  space_point_t current_control;
};
}  // namespace prx