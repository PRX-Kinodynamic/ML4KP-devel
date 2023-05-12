#pragma once
#include "prx/utilities/defs.hpp"
#include "prx/simulation/playback/plan.hpp"
#include "prx/simulation/playback/trajectory.hpp"

namespace prx
{
namespace simulation
{
// Useful to iterate over plans and trajectories:
// An iter is (x_i, u_i, x_{i+1})
struct plan_trajectory_stepper_t
{
  // ToDo: Think of a better name for "step"...
  using StateStepState = std::tuple<prx::space_point_t, prx::plan_step_t, prx::space_point_t, std::size_t>;

  plan_trajectory_stepper_t(plan_t* plan, trajectory_t* traj) : _plan(plan), _trajectory(traj)
  {
    const std::size_t plan_size{ _plan->size() };
    const std::size_t traj_size{ _trajectory->size() };
    prx_assert(plan_size + 1 == traj_size,
               "Mismatch in sizes. Trajectory size: " << traj_size << " plan size: " << plan_size);
  }

  class iterator
  {
    using iterator_category = std::output_iterator_tag;
    using value_type = StateStepState;  // crap
    using difference_type = StateStepState;
    using pointer = const StateStepState*;
    using reference = StateStepState;

    prx::trajectory_t::iterator _xi;    // x_i
    prx::plan_t::iterator _ui;          // u_i
    prx::trajectory_t::iterator _xip1;  // x_{i+1}
    std::size_t _idx;

  public:
    explicit iterator(plan_t* plan, trajectory_t* traj, const std::size_t idx = 0)
      : _xi((*traj).begin()), _ui((*plan).begin()), _xip1((*traj).begin() + 1), _idx(idx)
    {
    }

    iterator& operator++()
    {
      _xi = _xip1;
      _ui++;
      _xip1++;
      _idx++;
      return *this;
    }
    iterator operator++(int)
    {
      iterator retval = *this;
      ++(*this);
      return retval;
    }
    bool operator==(iterator other) const
    {
      return _idx == other._idx;
    }
    bool operator!=(iterator other) const
    {
      return !(*this == other);
    }
    reference operator*() const
    {
      prx::space_point_t xi_pt = *_xi;
      prx::plan_step_t ui_pt = *_ui;
      prx::space_point_t xip1_pt = *_xip1;
      return std::make_tuple(xi_pt, ui_pt, xip1_pt, _idx);
    }
  };

  iterator begin()
  {
    return iterator(_plan, _trajectory);
  }
  iterator end()
  {
    return iterator(_plan, _trajectory, _plan->size());
  }

  plan_t* _plan;
  trajectory_t* _trajectory;
};
}  // namespace simulation
}  // namespace prx