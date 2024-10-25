#pragma once

#include "prx/simulation/system.hpp"
#include "prx/simulation/plant.hpp"
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

  controller_t(system_ptr_t system_ptr, std::string name = "base_controller")
    : _plant(std::dynamic_pointer_cast<prx::plant_t>(system_ptr)), _name(name)
  {
    // set_points = std::make_shared<set_points_t>(plant -> get_state_space());
  }
  virtual ~controller_t();

  virtual void compute_controls() = 0;

  void compute_controls(space_point_t& u)
  {
    compute_controls();
    get_control_space()->copy_to(u);
  }

  // wrapper functions for better readability

  inline space_t* get_state_space() const
  {
    return _plant->get_state_space();
  }

  inline space_t* get_control_space() const
  {
    return _plant->get_control_space();
  }

  virtual void propagate(const double simulation_step)
  {
    _plant->propagate(simulation_step);
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
    _plant->get_state_space()->copy_point(goal, _goal);
  }

  virtual bool goal_reached(const space_point_t& current_state, distance_function_t df, const double tolerance = 0.1)
  {
    return df(current_state, goal) < tolerance;
  }

  virtual bool goal_reached(const space_point_t& current_state, distance_function_t df, const double tolerance = 0.1)
  {
    return df(current_state, goal) < tolerance;
  }

  std::shared_ptr<controller_t> get_ptr()
  {
    return shared_from_this();
  }

protected:
  controller_t(const controller_ptr_t& other)
  {
    _plant = other->_plant;
    _name = other->_name;
    goal = other->goal;
    plan = other->plan;
  };
  std::shared_ptr<plant_t> _plant;
  std::string _name;

  space_point_t goal;
  std::shared_ptr<plan_t> plan;  // Control sequence
};
}  // namespace prx
