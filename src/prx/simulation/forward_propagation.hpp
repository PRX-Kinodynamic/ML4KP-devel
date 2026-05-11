#pragma once

#include <memory>
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/playback/trajectory_v2.hpp"
#include "prx/utilities/defs.hpp"
// #include "prx/utilities/spaces/space_v2.hpp"
#include "prx/simulation/dynamical_system.hpp"

namespace prx
{
// template <typename System, typename Controller>
template <typename DerivedSystem, typename Trajectory, typename ControlType>
class forward_propagation_t
{
public:
  using DynamicalSystemPtr = std::shared_ptr<dynamical_system_t<DerivedSystem>>;
  forward_propagation_t(DynamicalSystemPtr dyn_sys) = delete;
};

// template <typename DynamicalSystem, typename Trajectory, typename Controller>  // primary template
// template <typename DerivedSystem, typename ControlType>
template <typename DynamicalSystem>
class forward_propagation_t<
    DynamicalSystem,  // no-lint
    prx::experimental::trajectory_t<typename dynamical_system_t<DynamicalSystem>::StateSpace, double>,
    prx::experimental::piecewise_plan_t<typename dynamical_system_t<DynamicalSystem>::Control, double>>
{
public:
  // using DynamicalSystem = dynamical_system_t<DerivedSystemType>;
  using DynamicalSystemPtr = std::shared_ptr<DynamicalSystem>;
  using Trajectory = prx::experimental::trajectory_t<typename DynamicalSystem::StateSpace, double>;
  using Controller = prx::experimental::piecewise_plan_t<typename DynamicalSystem::Control, double>;
  using State = typename DynamicalSystem::State;

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
