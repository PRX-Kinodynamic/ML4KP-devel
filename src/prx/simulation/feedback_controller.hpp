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