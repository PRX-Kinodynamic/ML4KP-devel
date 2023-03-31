#include "prx/utilities/defs.hpp"

#include "prx/simulation/defs.hpp"
#include "prx/simulation/general/condition_check.hpp"

namespace prx
{
std::map<std::string, unsigned> condition_check_t::available_types{ { "iterations", 0 },
                                                                    { "time", 1 },
                                                                    { "sim_time", 2 },
                                                                    { "custom", 3 } };

condition_check_t::condition_check_t(std::string type, double check) : condition_check_t()
{
  condition_check = check;
  condition_type = available_types[type];

  if (condition_type > 2)
  {
    prx_throw("Condition type is invalid!");
  }
}

condition_check_t::condition_check_t(custom_check_t _custom_check) : condition_check_t()
{
  condition_type = available_types["custom"];
  custom_check = _custom_check;
}

void condition_check_t::add_condition(condition_check_t* _cond)
{
  others.push_back(_cond);
}

void condition_check_t::reset()
{
  timer.reset();
  iteration_counter = 0;
  sim_time_accum = 0;

  for (auto c : others)
  {
    c->reset();
  }
}

bool condition_check_t::check()
{
  ++iteration_counter;
  sim_time_accum += simulation_step;

  switch (condition_type)
  {
    case 0:
      if (iteration_counter >= condition_check)
        return true;
      break;
    case 1:
      if (timer.measure() >= condition_check)
        return true;
      break;
    case 2:
      if (sim_time_accum >= condition_check)
        return true;
      break;
    case 3:
      if (custom_check())
        return true;
      break;
    default:
      prx_throw("Problem with condition_check type");
  }

  for (auto c : others)
  {
    if (c->check())
      return true;
  }
  return false;
}
}  // namespace prx