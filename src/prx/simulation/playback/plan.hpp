#pragma once

#include "prx/simulation/defs.hpp"

#include "prx/utilities/defs.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/general/constants.hpp"
#include "prx/utilities/general/transforms.hpp"

#include <deque>

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
    os << "duration: " << obj.duration << " ctrl: " << obj.control;
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

  /**
   * @brief A plan can contain plan_steps that have a duration > simulation_step. This expands all plan_steps to have
   * duration==simulation_step by adding as many plan_steps as necessary.
   */
  void expand();

  /**
   * @brief The oposite of expand: if multiple consecutive plan_steps have the same control, have a single plan_step
   * with duration = total plan_steps that are equal.
   */
  void compress();
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
    double t = 0;
    for (auto&& step : *this)
    {
      t += step.duration;
    }
    return t;
  }

  inline plan_step_t operator[](std::size_t index) const
  {
    prx_assert(index < num_steps, "Trying to access plan[ " << index << "] outside of bounds ( " << num_steps << ").");
    return steps[index];
  }

  inline plan_step_t& operator[](std::size_t index)
  {
    prx_assert(index < num_steps, "Trying to access plan[ " << index << "] outside of bounds ( " << num_steps << ").");
    return steps[index];
  }

  // inline const plan_step_t& at(std::size_t index) const
  // {
  //   prx_assert(index < num_steps, "Trying to access plan outside of bounds.");
  //   return steps[index];
  // }

  // inline space_point_t operator[](double t) const
  // {
  //   return at(t);
  // }

  inline space_point_t at(double t) const
  {
    for (auto&& step : steps)
    {
      if (t < step.duration)
        return step.control;
      else
        t -= step.duration;
    }
    prx_throw("Indexed into plan with time outside the plan's full duration.");
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

  void copy_to(const double start_time, const double duration, plan_t& t);

  template <typename Control, typename Duration>
  void copy_onto_back(const Control& control, const Duration& duration)
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

  // void copy_onto_back(space_point_t control, double time);
  // void copy_onto_back(Eigen::VectorXd v_control, double time);

  void copy_onto_front(space_point_t control, double time);

  /**
   * @brief Add one step to the front of the plan.
   * @param time Duration of the first control of the modified plan.
   */
  void append_onto_front(double time);

  // void append_onto_back(double time);

  /**
   * @brief Add one step to the back of the plan.
   * @param time Duration of the last control of the modified plan.
   */
  void append_onto_back(double time, bool copy_from_control_space = false);

  void append_onto_back(double time, space_t* ctrl_space);

  void append_onto_back(const plan_t& other_plan)
  {
    (*this) += other_plan;
  }

  /**
   * @brief Extend the duration of the last control in the plan by a specified time.
   * @param time Time to extend the duration of the last control by (in seconds).
   */
  void extend_last_control(double time);

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
    if (obj.num_steps < 10)
    {
      os << obj.print() << " ";
    }
    else
    {
      for (unsigned i = 0; i < 5; ++i)
      {
        os << obj[i] << "\n";
      }
      os << "(...)\n";
      for (unsigned i = obj.num_steps - 5; i < obj.num_steps; ++i)
      {
        os << obj[i] << "\n";
      }
    }
    return os;
  }

private:
  void increase_buffer();

  const space_t* control_space;

  iterator end_iterator;

  const_iterator const_end_iterator;

  std::size_t num_steps;

  std::size_t max_num_steps;

  std::deque<plan_step_t> steps;
};
}  // namespace prx
