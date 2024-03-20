#pragma once

#include "prx/simulation/system.hpp"
#include "prx/simulation/playback/plan.hpp"
#include "prx/simulation/playback/trajectory.hpp"

#include <unordered_map>

namespace prx
{
class controller_t;
typedef std::shared_ptr<controller_t> controller_ptr_t;

class controller_t : public std::enable_shared_from_this<controller_t>
{
public:
  controller_t(const controller_t& other) = default;

  controller_t(system_ptr_t _plant, std::string _name = "base_controller")
  {
    plant = _plant;
    name = _name;
  }
  virtual ~controller_t();

  virtual void compute_controls() = 0;

  void compute_controls(space_point_t& u)
  {
    compute_controls();
    get_control_space()->copy_to_point(u);
  }

  // wrapper functions for better readability

  inline const space_t* get_state_space() const
  {
    return plant->get_state_space();
  }

  inline const space_t* get_control_space() const
  {
    return plant->get_control_space();
  }

  virtual void propagate(const double simulation_step)
  {
    plant->propagate(simulation_step);
  }
  virtual void set_plan(const plan_t& _plan)
  {
    plan = std::make_shared<plan_t>(_plan);
  }

  virtual std::shared_ptr<plan_t> get_plan()
  {
    return plan;
  }

  virtual void init_plan()
  {
    plan = std::make_shared<plan_t>(get_control_space());
  }

  virtual void set_goal(space_point_t _goal)
  {
    plant->get_state_space()->copy_point(goal, _goal);
  }

  virtual bool goal_reached(const space_point_t& current_state)
  {
    return space_t::euclidean_2d(current_state, goal) < 0.1;
  }

  std::shared_ptr<controller_t> get_ptr()
  {
    return shared_from_this();
  }

protected:
  controller_t(const controller_ptr_t& other)
  {
    plant = other->plant;
    name = other->name;
    goal = other->goal;
    plan = other->plan;
  };
  system_ptr_t plant;
  std::string name;

  space_point_t goal;
  std::shared_ptr<plan_t> plan;  // Control sequence
};
}  // namespace prx
