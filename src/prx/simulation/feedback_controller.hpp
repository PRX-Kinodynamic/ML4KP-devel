#pragma once
#include "prx/simulation/controller.hpp"

namespace prx
{
template <typename Points>
class feedback_controller_t : public controller_t
{
public:
  feedback_controller_t(system_ptr_t _plant, std::string _name = "feedback_controller") : controller_t(_plant, _name)
  {
    goal = get_state_space()->make_point();

    df = [](const space_point_t& a, const space_point_t& b) {
      return std::sqrt(std::pow(a->at(0) - b->at(0), 2) + std::pow(a->at(1) - b->at(1), 2));
    };
  };

  virtual bool goal_reached(const space_point_t& current_state) = 0;

  template <typename T>
  void set_goal(const T& _goal)
  {
    get_state_space()->copy(goal, _goal);
  }
  
  virtual void reset() = 0;

  virtual void get_control(const space_point_t& current_state, Eigen::VectorXd& control) = 0;

  using controller_t::compute_controls;
  virtual void compute_controls() override
  {
    space_point_t current = get_state_space()->make_point();
    get_state_space()->copy_to(current);
    Eigen::VectorXd control;
    get_control(current,control);
    get_control_space()->copy_from(control);
    get_control_space()->enforce_bounds();
  }

  virtual void set_points(std::shared_ptr<Points> points)
  {
    _points = points;
  }

  inline double control_duration() const
  {
    return duration;
  }

protected:
  std::shared_ptr<Points> _points;
  double duration;
  distance_function_t df;
};

}  // namespace prx