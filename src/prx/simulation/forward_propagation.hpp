#pragma once

#include <__utility/piecewise_construct.h>
#include <memory>
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/playback/trajectory_v2.hpp"
#include "prx/utilities/defs.hpp"
// #include "prx/utilities/spaces/space_v2.hpp"
#include "prx/simulation/dynamical_system.hpp"

namespace prx
{
// template <typename System, typename Controller>
// template <typename Trajectory, typename System, typename Controller>
// class forward_propagation_t
// {
// };

// template <typename DynamicalSystem, typename Trajectory, typename Controller>  // primary template
template <typename DerivedSystem, typename ControlType>
class forward_propagation_t  //<prx::experimental::trajectory_t<typename DerivedSystem::StateSpace, double>,
//                            dynamical_system_t<DerivedSystem>, prx::experimental::piecewise_step_t<ControlType,
//                            double>>
{
public:
  using Trajectory = prx::experimental::trajectory_t<typename DerivedSystem::StateSpace, double>;
  using DynamicalSystemPtr = std::shared_ptr<dynamical_system_t<DerivedSystem>>;
  // using PiecewisePlan = prx::experimental::piecewise_step_t<ControlType, double>;
  using Controller = prx::experimental::piecewise_plan_t<ControlType, double>;
  using State = typename dynamical_system_t<DerivedSystem>::State;
  forward_propagation_t(DynamicalSystemPtr dyn_sys) : _f(dyn_sys)
  {
  }

  void operator()(Trajectory& traj, const State& x0_in, const Controller& plan)
  {
    State x0{ x0_in };
    double ti{ 0.0 };
    traj.push_back(x0, ti);
    for (auto&& step : plan)
    {
      x0 = _f->propagate(x0, step.control, step.duration);
      ti += step.duration;
      traj.push_back(x0, ti);
    }
  }

protected:
  DynamicalSystemPtr _f;
};

}  // namespace prx
