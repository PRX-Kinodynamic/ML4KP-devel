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
  custom_controller_t(std::shared_ptr<plant_t> plant_in, std::string name) : controller_t(plant_in, name)
  {
    control_pt = plant_in->get_control_space()->make_point();
  }

  virtual ~custom_controller_t()
  {
  }

  virtual void compute_controls() override
  {
    custom_control_function(goal, control_pt);
    plant->get_control_space()->copy_from(control_pt);
    plan->append_onto_back(simulation_step, true);
  }

  std::function<void(const space_point_t&, const space_point_t&)> custom_control_function =
      [](const space_point_t& goal, const space_point_t& control) { PRX_NOT_IMPLEMENTED };

protected:
  space_point_t control_pt;

  // std::shared_ptr<plant_t> _plant;
};
}  // namespace prx