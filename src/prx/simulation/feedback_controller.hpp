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
    };
    
    virtual void set_points(std::shared_ptr<Points> points)
    {
      _points = points;
    }

    std::shared_ptr<Points> _points;
  };

}  // namespace prx