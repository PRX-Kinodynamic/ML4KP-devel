
#include "prx/simulation/controllers/random_walk.hpp"

namespace prx
{
random_walk_t::~random_walk_t()
{
}

void random_walk_t::compute_controls()
{
  if (fmod(counter, 50.0) < .5)
  {
    get_control_space()->sample(current_control);
  }
  get_control_space()->copy_from_point(current_control);
}

void random_walk_t::propagate(const double simulation_step)
{
  counter += 1;
  controller_t::propagate(simulation_step);
}
}  // namespace prx