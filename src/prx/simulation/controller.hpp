#pragma once

#include "prx/simulation/system.hpp"
#include "prx/simulation/plant.hpp"
#include "prx/simulation/playback/plan.hpp"
#include "prx/simulation/playback/trajectory.hpp"

#include <unordered_map>

namespace prx
{
class controller_t;

// TODO: Change name?
// Desired states/points
// Objective
// Goal
// local_goal
class set_points_t
{
public:
  set_points_t(const space_t* _space)
  {
    space = _space;
  }

  inline space_point_t operator[](unsigned index) const
  {
    prx_assert(index < set_points.size(),
               "Set point out of bounds. Size: " << set_points.size() << " requested: " << index);
    return set_points[index];
  }

  inline space_point_t& operator[](unsigned index)
  {
    prx_warn_cond(index <= set_points.size(), "Adding " << (index - set_points.size()) << " set_points");
    for (int i = set_points.size(); i <= index; ++i)
    {
      set_points.push_back(space->make_point());
    }
    return set_points[index];
  }

private:
  set_points_t() {};
  std::vector<space_point_t> set_points;
  const space_t* space;
  friend controller_t;
};

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

  inline const space_t* get_state_space() const
  {
    return _plant->get_state_space();
  }

  inline const space_t* get_control_space() const
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
    set_points = other->set_points;
  };
  std::shared_ptr<plant_t> _plant;
  std::string _name;

  std::shared_ptr<set_points_t> set_points;
  space_point_t goal;
  std::shared_ptr<plan_t> plan;  // Control sequence
};
}  // namespace prx
