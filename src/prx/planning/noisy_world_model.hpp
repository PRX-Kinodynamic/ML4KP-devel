#pragma once

#include "prx/simulation/simulator.hpp"
#include "prx/simulation/system_group_manager.hpp"
#include "prx/simulation/collision_checking/collision_checker.hpp"
#include "prx/simulation/world_model.hpp"

#include "prx/utilities/general/noise.hpp"

#include <unordered_map>

namespace prx
{
template <typename T>
class noisy_world_model_t : public world_model_t
{
public:
  template <class... Types>
  noisy_world_model_t(const std::vector<system_ptr_t>& all_systems,
                      const std::vector<std::shared_ptr<movable_object_t>>& all_obstacles, Types... args)
    : world_model_t(all_systems, all_obstacles)
  {
    noise = std::make_shared<T>(args...);
  }

  virtual void step_simulation(propagate_step step) override
  {
    world_model_t::step_simulation(step);

    for (auto name : all_context_names)
    {
      noise->add_noise(system_groups->get_system_group(name)->get_state_space());
    }
  }

protected:
  std::shared_ptr<T> noise;
};
}  // namespace prx
