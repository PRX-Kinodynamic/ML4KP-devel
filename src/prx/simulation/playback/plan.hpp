#pragma once

#include <deque>
#include <fstream>
#include <iterator>

// #include "prx/utilities/general/debug_utils.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/defs.hpp"

namespace prx
{
struct plan_step_t
{
  plan_step_t(space_point_t control, const double duration) : control(control), duration(duration)
  {
  }

  void copy_step(const space_t* space, const plan_step_t& step)
  {
    space->copy_point(control, step.control);
    duration = step.duration;
  }

  friend std::ostream& operator<<(std::ostream& os, const plan_step_t& obj)
  {
    os << obj.control << " ";
    os << obj.duration << " ";
    return os;
  }

  space_point_t control;
  double duration;
};
/**
 * @brief <b>A class that defines a plan.</b>
 *
 * A plan consists of a sequence of piecewise constant controls, each of which are
 * applied for a specified duration. Each (control, duration) pair is referred to as a <i> step </i>.
 *
 * @authors Zakary Littlefield
 *
 */
class plan_t
{
public:
  typedef std::deque<plan_step_t>::iterator iterator;
  typedef std::deque<plan_step_t>::const_iterator const_iterator;

  plan_t(const space_t* new_space);

  plan_t(const plan_t& other);

  ~plan_t();

  // Given a plan, make a new plan where every plan_step_t duration is prx::simulation_step
  // This is, the new plan is the same as the old one but its size is other.size() / prx::simulation_step
  static plan_t expand(const plan_t& other);

  // Expand in place
  void expand();

  /**
   * @brief Returns the number of steps in the plan.
   *
   * A plan {(u_1,t_1),(u_2,t_2),...,(u_M,t_M)} has <i> M </i> steps.
   * @return Number of steps in the plan.
   */
  inline std::size_t size() const
  {
    return num_steps;
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
    // TODO: In general, this is inefficient (O(n)), could change to constant by always storing the plan's duration or
    // amortized cte by storing a dirty flag.
    double t = 0;
    // for (auto&& step : *this)
    for (auto iter = begin(); iter != end_iterator; iter++)
    {
      t += iter->duration;
    }
    return t;
  }

  inline plan_step_t& operator[](const std::size_t& index)
  {
    prx_assert(index < num_steps, "Trying to access plan outside of bounds.");
    return steps[index];
  }

  inline const plan_step_t operator[](const std::size_t& index) const
  {
    prx_assert(index < num_steps, "Trying to access plan outside of bounds.");
    return steps[index];
  }

  inline space_point_t at(const double t) const
  {
    double t_accum{ t };
    for (auto&& step : steps)
    {
      if (t_accum < step.duration)
        return step.control;
      else
        t_accum -= step.duration;
    }
    prx_throw("Indexed into plan with time outside the plan's full duration.");
  }

  // Split at specified time: ThisOld = [ ThisNew | NewPlan ]
  plan_t split(const double time_of_split)
  {
    plan_t new_plan(control_space);

    bool split_done{ false };
    double accum_duration{ 0.0 };
    iterator split_iter;
    for (auto iter = steps.begin(); iter != end_iterator; iter++)
    {
      if (accum_duration + iter->duration > time_of_split)
      {
        const double step_time_split{ time_of_split - accum_duration };
        // new_plan.steps.insert(new_plan.steps.begin(), iter->split(step_time_split));
        // PRX_DBG_VARS(time_of_split, accum_duration, iter->duration);
        new_plan.copy_onto_back(iter->control, iter->duration - step_time_split);
        iter->duration = step_time_split;
        // PRX_DBG_VARS(new_plan);
        // // Split at specified time: ThisOld = [ ThisNew | NewPlanStep ]
        // plan_step_t split(const double time_to_split)
        // {
        //   prx_assert(time_to_split < duration, "Time to split is less than duration");
        //   plan_step_t plan(control, duration - time_to_split);
        //   duration = time_to_split;
        //   return plan;
        // }

        split_iter = iter + 1;
        break;
      }
      accum_duration += iter->duration;
    }
    if (split_iter != steps.end())
    {
      // new_plan.end_iterator = new_plan.steps.begin();
      // PRX_DBG_VARS(new_plan.steps.size(), std::distance(steps.begin(), split_iter));
      const long dist{ std::distance(split_iter, end_iterator) + 1 };
      new_plan.steps.insert(new_plan.end_iterator, split_iter, end_iterator);
      // PRX_DBG_VARS(dist, new_plan.steps.size());
      steps.erase(split_iter, end_iterator);
      new_plan.end_iterator = new_plan.steps.begin();
      std::advance(new_plan.end_iterator, dist);

      // new_plan.end_iterator = new_plan.steps.begin() + dist + 1;

      // PRX_DBG_VARS(num_steps, std::distance(steps.begin(), end_iterator));
      num_steps = std::distance(steps.begin(), end_iterator);
      // PRX_DBG_VARS(new_plan.steps.size(), std::distance(new_plan.steps.begin(), new_plan.end_iterator));
      new_plan.num_steps = std::distance(new_plan.steps.begin(), new_plan.end_iterator);
    }

    return new_plan;
  }

  inline plan_step_t& front()
  {
    prx_assert(num_steps != 0, "Trying to access the front of an empty plan.");
    return steps[0];
  }

  inline plan_step_t& back()
  {
    prx_assert(num_steps != 0, "Trying to access the back of an empty plan.");
    return steps[num_steps - 1];
  }

  inline iterator begin()
  {
    return steps.begin();
  }

  inline iterator end()
  {
    return end_iterator;
  }

  inline const_iterator begin() const
  {
    return steps.begin();
  }

  inline const_iterator end() const
  {
    return const_end_iterator;
  }

  void resize(unsigned num_size);

  plan_t& operator=(const plan_t& t);

  plan_t& operator+=(const plan_t& t);

  /**
   * @brief Clear the contents of the plan.
   */
  void clear();

  void copy_to(const double start_time, const double duration, plan_t& t) const;

  template <typename Control>
  void copy_onto_back(const Control& control, const double duration)
  {
    if ((num_steps + 1) >= max_num_steps)
    {
      increase_buffer();
      end_iterator = steps.begin();
      const_end_iterator = steps.begin();
      std::advance(end_iterator, num_steps);
      std::advance(const_end_iterator, num_steps);
    }
    control_space->copy((*end_iterator).control, control);
    (*end_iterator).duration = duration;
    ++end_iterator;
    ++const_end_iterator;
    ++num_steps;
  }

  template <typename Control>
  void copy_onto_front(const Control& control, double time)
  {
    if ((num_steps + 1) >= max_num_steps)
      increase_buffer();

    plan_step_t new_step = steps.back();
    steps.pop_back();
    steps.push_front(new_step);
    control_space->copy((*steps.begin()).control, control);
    (*steps.begin()).duration = time;
    ++num_steps;
    end_iterator = steps.begin();
    const_end_iterator = steps.begin();
    std::advance(end_iterator, num_steps);
    std::advance(const_end_iterator, num_steps);
  }

  /**
   * @brief Add one step to the front of the plan.
   * @param time Duration of the first control of the modified plan.
   */
  void append_onto_front(double time);

  /**
   * @brief Add one step to the back of the plan.
   * @param time Duration of the last control of the modified plan.
   */
  void append_onto_back(double time);

  /**
   * @brief Extend the duration of the last control in the plan by a specified time.
   * @param time Time to extend the duration of the last control by (in seconds).
   */
  void extend_last_control(double time);

  /**
   * @brief Reduce the duration of the last control in the plan by a specified time.
   * @param time Time to reduce the duration of the last control by (in seconds).
   */
  void reduce_last_control(double time);

  /**
   * @brief Removes the first control from the plan.
   */
  void pop_front();

  /**
   * @brief Removes the last control from the plan.
   */
  void pop_back();

  /**
   * @brief Helper function to print out the plan.
   * @return A string object that outputs the plan.
   */
  std::string print(unsigned precision = 3) const;

  void to_file(const std::string, const std::ios_base::openmode _mode = std::ofstream::trunc) const;
  void from_file(const std::string file_name);

  friend std::ostream& operator<<(std::ostream& os, const plan_t* obj)
  {
    os << *obj;
    return os;
  }

  friend std::ostream& operator<<(std::ostream& os, const plan_t& obj)
  {
    os << obj.print();

    return os;
  }

  // Create a (uniform) random plan
  void random(const std::size_t min_steps, const std::size_t max_steps, const double tmin, const double tmax)
  {
    const std::size_t tot_steps{ static_cast<std::size_t>(std::floor(uniform_random(min_steps, max_steps))) };

    resize(tot_steps);
    for (int i = 0; i < tot_steps; ++i)
    {
      control_space->sample((*this)[i].control);
      (*this)[i].duration = uniform_random(tmin, tmax);
    }
  }

private:
  void increase_buffer();

  const space_t* control_space;

  iterator end_iterator;

  const_iterator const_end_iterator;

  unsigned num_steps;

  unsigned max_num_steps;

  std::deque<plan_step_t> steps;
};
}  // namespace prx
