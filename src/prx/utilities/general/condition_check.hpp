#pragma once
/**
 * @file condition_check.hpp
 * @brief A class which checks if a condition is met.
 * @details A class which checks if a condition is met.
 * @author Zakary Littlefield, Aravind Sivaramakrishnan, Edgar Granados
 */

#include <map>
#include <string>

#include "prx/utilities/general/timer.hpp"
#include "prx/utilities/general/param_loader.hpp"

namespace prx
{
extern double simulation_step;

typedef std::function<bool()> custom_check_t;

// TODO: Add condition of "Return at first solution found"
/**
 * @brief A class which checks if a condition is met.
 * @details A class which checks if a condition is met.
 *
 * Two types of conditions can be specified: iterations and time.
 *
 * @author Zakary Littlefield, Aravind Sivaramakrishnan
 */
class condition_check_t
{
public:
  condition_check_t(std::string type, double check);

  condition_check_t(custom_check_t _custom_check);

  condition_check_t(const prx::param_loader&);

  /**
   * @brief Resets the internal counts and timers.
   * @details Resets the internal counts and timers.
   */
  void reset();

  /**
   * @brief Check if condition is satisfied.
   * @details Check if condition is satisfied.
   * @return True if satisfied, false if not.
   */
  bool check();

  bool operator()()
  {
    return check();
  }
  /**
   * @brief Get the time on the timer.
   * @details Get the time on the timer.
   *
   * @return The current time.
   */
  double time()
  {
    return timer.measure();
  }

  /**
   * @brief Get the current iterations.
   * @details Get the current iterations.
   *
   * @return The current iterations.
   */
  long unsigned iterations()
  {
    return iteration_counter;
  }

  /**
   * @brief Get the max time or iterations.
   * @details Get the max time or iterations.
   *
   * @return The max time or iterations.
   */
  long unsigned get_check_value()
  {
    return condition_check;
  }

  void set_check_value(const double _condition_check)
  {
    condition_check = _condition_check;
  }

  void add_condition(condition_check_t* _cond);

  std::vector<std::string> get_available_types()
  {
    std::vector<std::string> v;
    for (auto str : available_types)
    {
      v.push_back(str.first);
    }
    return v;
  }

  void print_available_types()
  {
    auto v = get_available_types();
    std::cout << "[condition_check_t] Available types:" << std::endl;
    for (auto s : v)
    {
      std::cout << "\t" << s << std::endl;
    }
  }

  virtual prx::param_loader initialization_parameters()
  {
    prx::param_loader params{};

    for (auto type : available_types)
    {
      if (type.second == condition_type)
        params["type"].set(type.first);
    }
    params["value"].set(condition_check);
    return params;
  }

  static prx::param_loader init()
  {
    prx::param_loader params;
    params["type"].set("iterations | time | sim_time | custom");
    params["value"].set(0.0);
    return params;
  }

  virtual void init(const prx::param_loader& params)
  {
    if (params.exists("type"))
    {
      condition_type = available_types[params["type"].as<>()];
    }
    else
    {
      prx_throw("[Condition Check] No condition type");
    }
    if (params.exists("value"))
    {
      condition_check = params["value"].as<double>();
    }
    else
    {
      prx_throw("[Condition Check] No value type");
    }
  }

protected:
  condition_check_t() : iteration_counter(0), condition_check(0), condition_type(0), sim_time_accum(0)
  {
    timer.reset();
  }

  // condition_check_t(const condition_check_t&) = default ;

  /**
   * @brief The timer used for checking times.
   */
  timer_t timer;

  /**
   * @brief A counter for iterations.
   */
  long unsigned iteration_counter;

  /**
   * @brief The maximum time or iterations before being satisfied.
   */
  double condition_check;

  /**
   * @brief Which type of condition to check. 0 for iterations, 1 for time.
   */
  unsigned condition_type;

  /**
   * @brief The accumulator for simulation time.
   */
  double sim_time_accum;

  /**
   * @brief Function to use for custom check condition. When return true, check() returns true .
   */
  custom_check_t custom_check;

  std::vector<condition_check_t*> others;

  static std::map<std::string, unsigned> available_types;
};
}  // namespace prx
