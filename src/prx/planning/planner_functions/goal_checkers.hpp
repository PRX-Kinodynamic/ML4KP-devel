#pragma once

#include "general/param_loader.hpp"
#include "prx/utilities/defs.hpp"
#include "prx/simulation/playback/plan.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/simulation/playback/plan.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include <gtsam/config.h>
#include <gtsam/base/Lie.h>

namespace prx
{
template <typename State>
class goal_radius_checker_t
{
  static constexpr Eigen::Index Dim{ gtsam::traits<State>::dimension };
  using Vector = Eigen::Vector<double, Dim>;

public:
  goal_radius_checker_t() : _weights(Eigen::Vector<double, Dim>::Ones()), _radius(0.5), _goal(State())
  {
  }

  goal_radius_checker_t(prx::param_loader param)
    : _weights(param.get_or_default("weights", Eigen::Vector<double, Dim>::Ones().eval()))
    , _radius(param.get_or_default("radius", 0.5))
    , _goal(param.get_or_default("goal", std::move(State())))
  {
  }

  bool operator()(const State& state)
  {
    const State between{ _goal.between(state) };
    const Vector error{ State::Logmap(between) };
    const Vector w_error{ (_weights.array() * error.array()).matrix().template cast<double>() };
    const double norm{ w_error.norm() };
    return norm < _radius;
  }

protected:
  double _radius;
  State _goal;
  Vector _weights;
};
}  // namespace prx
