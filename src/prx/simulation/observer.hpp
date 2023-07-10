#pragma once
#include "prx/utilities/defs.hpp"
#include "prx/utilities/spaces/space.hpp"

#include "prx/simulation/plant.hpp"
#include "prx/simulation/system_group.hpp"

namespace prx
{
namespace simulation
{
// Observer class to go from state space -> observed space
// Can be though of a sensor(s) that are only looking at specific variables / function of variables
// Using observer to be closer to the control terminology
class observer_t
{
public:
  observer_t()
  {
    _observed_space = new space_t("", {}, "empty");
  }

  observer_t& add(observer_t* new_observer)
  {
    if (_observed_space != nullptr)
    {
      _observed_space = new space_t({ _observed_space, new_observer->_observed_space });
    }
    else
    {
      _observed_space = new_observer->_observed_space;
    }
    observers.push_back(new_observer);
    return *this;
  }

  virtual void observe(const std::shared_ptr<plant_t> plant){};

  template <typename Observation>
  void operator()(const std::shared_ptr<plant_t> plant, Observation observation)
  {
    this->observe(plant);
    for (auto&& o : observers)
    {
      o->observe(plant);
    }
    _observed_space->copy_to(observation);
  }

  template <typename Observation>
  void operator()(std::shared_ptr<prx::system_group_t> group, Observation observation)
  {
    for (auto sys_ptr : *group)
    {
      auto plant = std::dynamic_pointer_cast<prx::plant_t>(sys_ptr);
      this->operator()(plant, observation);
    }
  }

  inline space_t* get_observed_space()
  {
    return _observed_space;
  }

protected:
  space_t* _observed_space;
  std::vector<double*> _observer_memory;

  std::vector<observer_t*> observers;
};
}  // namespace simulation
}  // namespace prx