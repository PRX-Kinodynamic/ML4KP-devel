#pragma once

#include <deque>
#include <fstream>
#include <iterator>

// #include "prx/utilities/general/debug_utils.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/defs.hpp"
#include "prx/utilities/spaces/streamer.hpp"

namespace prx
{

template <typename ControlType, typename DurationType>
struct piecewise_step_t
{
  using PiecewiseStep = piecewise_step_t<ControlType, DurationType>;
  piecewise_step_t() : control(ControlType()), duration(DurationType()) {};
  piecewise_step_t(const ControlType control_, const DurationType duration_) : control(control_), duration(duration_)
  {
  }

  friend std::ostream& operator<<(std::ostream& os, const PiecewiseStep& obj)
  {
    prx::streamer_t<ControlType>::to_stream(os, obj.control);
    prx::streamer_t<ControlType>::to_stream(os, obj.duration);
    os << "\n";
    return os;
  }

  ControlType control;
  DurationType duration;
};
/**
 *
 * @authors Edgar Granados
 *
 */
template <typename ControlType, typename DurationType>
class piecewise_plan_t
{
public:
  using PiecewiseStep = piecewise_step_t<ControlType, DurationType>;
  using PiecewisePlan = piecewise_plan_t<ControlType, DurationType>;
  using PiecewisePlanPtr = std::shared_ptr<PiecewisePlan>;
  using iterator = typename std::deque<PiecewiseStep>::iterator;
  using const_iterator = typename std::deque<PiecewiseStep>::const_iterator;

  // piecewise_plan_t(const space_t* new_space){};
  piecewise_plan_t() : _total_duration(0.0) {};
  piecewise_plan_t(const piecewise_plan_t& other) = default;

  ~piecewise_plan_t()
  {
  }

  inline std::size_t size() const
  {
    return _steps.size();
  }
  /**
   * @brief Returns the duration of the plan.
   *
   * For a plan {(u_1,t_1),(u_2,t_2),...,(u_M,t_M)}, it's duration is given by
   * t_1 + t_2 + ... + t_M.
   *
   * @return Duration of the plan.
   */
  inline double duration() const
  {
    return _total_duration;
  }

  inline PiecewiseStep operator[](const std::size_t& index)
  {
    return _steps[index];
  }

  inline PiecewiseStep front()
  {
    return _steps.front();
  }

  inline PiecewiseStep back()
  {
    return _steps.back();
  }

  inline const_iterator begin() const
  {
    return _steps.begin();
  }

  inline const_iterator end() const
  {
    return _steps.end();
  }

  inline iterator begin()
  {
    return _steps.begin();
  }

  inline iterator end()
  {
    return _steps.end();
  }

  // plan_t& operator=(const plan_t& t);

  // plan_t& operator+=(const plan_t& t);

  /**
   * @brief Clear the contents of the plan.
   */
  void clear()
  {
    _total_duration = 0;
    _steps.clear();
  }

  void copy_to(const double start_time, const double duration, PiecewisePlan& plan) const
  {
    double s = start_time;
    double d = duration;
    auto step = _steps.begin();
    while (s > step->duration)
    {
      s -= step->duration;
      step++;
      if (step == _steps.end())
        prx_throw("Indexed into plan with time outside the plan's full duration.");
    }
    if (d < step->duration)
    {
      plan.push_back(step->control, d);
      // std::cout<<duration<<" Copy to: "<<t.duration()<<std::endl;
      return;
    }
    plan.push_back(step->control, step->duration - s);
    d -= (step->duration - s);
    step++;
    while (step != _steps.end())
    {
      if (d < step->duration)
      {
        plan.push_back(step->control, d);
        break;
      }
      plan.push_back(step->control, step->duration);
      d -= step->duration;
      step++;
    }
  }

  void push_front(const ControlType& control, DurationType time)
  {
    _total_duration += time;
    _steps.emplace_front(control, time);
  }

  void push_back(const ControlType& control, DurationType time)
  {
    _total_duration += time;
    _steps.emplace_back(control, time);
  }

  void push_back(const PiecewiseStep& step)
  {
    _total_duration += step.duration;
    _steps.push_back(step);
  }

  void pop_front()
  {
    _total_duration -= _steps.front().duration;
    _steps.pop_front();
  }

  /**
   * @brief Removes the last control from the plan.
   */
  void pop_back()
  {
    _total_duration -= _steps.back().duration;
    _steps.pop_back();
  }

  void to_file(const std::string filename, const std::ios_base::openmode mode = std::ofstream::trunc) const
  {
    std::ofstream ofs(filename, mode);
    ofs << *this;
    ofs.close();
  }
  // void from_file(const std::string file_name);

  friend std::ostream& operator<<(std::ostream& os, const PiecewisePlanPtr obj)
  {
    os << *obj;
    return os;
  }

  friend std::ostream& operator<<(std::ostream& os, const PiecewisePlan& obj)
  {
    for (auto&& step : obj)
    {
      os << step;
    }

    return os;
  }

private:
  double _total_duration;
  std::deque<PiecewiseStep> _steps;
};
namespace experimental
{
// To avoid some refactoring for now
template <typename ControlType, typename DurationType>
using piecewise_step_t = prx::piecewise_step_t<ControlType, DurationType>;

template <typename ControlType, typename DurationType>
using piecewise_plan_t = prx::piecewise_plan_t<ControlType, DurationType>;
}  // namespace experimental

template <typename ControlType, typename TimeType>
void merge(piecewise_plan_t<ControlType, TimeType>& plan, const piecewise_plan_t<ControlType, TimeType> other)
{
  for (auto&& step : other)
  {
    plan.push_back(step);
  }
}

}  // namespace prx
