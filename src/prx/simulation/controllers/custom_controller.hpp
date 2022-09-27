#pragma once

#include "prx/simulation/controller.hpp"
#include "prx/simulation/plants/types/linear_time_invariant.hpp"
#include "prx/utilities/math/continuous_algebraic_riccati_equation.hpp"

namespace prx
{
class custom_controller_t : public controller_t
{
public:
  // template<class S>
  custom_controller_t(std::shared_ptr<plant_t> plant, std::string name) : controller_t(plant, name), _plant(plant)
  {
    x_goal = _plant->get_state_space()->make_point();
    u_goal = _plant->get_control_space()->make_point();
  }

  virtual ~custom_controller_t();

  virtual void compute_controls() override
  {
    custom_control_function(x_goal, u_goal);
  }

  std::function<void(const space_point_t&, space_point_t&)> custom_control_function =
      [](const space_point_t& goal, space_point_t& control) { PRX_NOT_IMPLEMENTED };

protected:
  space_point_t x_goal;
  space_point_t u_goal;

  std::shared_ptr<plant_t> _plant;
};
}  // namespace prx