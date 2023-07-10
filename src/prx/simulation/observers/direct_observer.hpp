#pragma once
#include "prx/simulation/observer.hpp"

namespace prx
{
namespace simulation
{

// Direct as in Observation(State[i:j]) = State[i:j]
// Trying to avoid passthrough, but there may be a better name
class direct_observer_t : public observer_t
{
public:
  direct_observer_t(const std::vector<double*>& input_addresses, const std::string& topology)
    : observer_t()
    , dimension(input_addresses.size())
    , _input_addresses(input_addresses)
    , _memory(input_addresses.size(), 0.0)
  {
    prx_assert(topology.size() == dimension, "Wrong topology dimension");
    for (int i = 0; i < dimension; ++i)
    {
      _observer_memory.push_back(&_memory[i]);
    }
    _observed_space = new prx::space_t(topology, _observer_memory, "direct_observer_space");
  }

  virtual void observe(const std::shared_ptr<plant_t> plant) override final
  {
    for (int i = 0; i < dimension; ++i)
    {
      _memory[i] = *(_input_addresses[i]);
    }
  }

private:
  const std::size_t dimension;
  std::vector<double> _memory;
  const std::vector<double*> _input_addresses;
};

}  // namespace simulation
}  // namespace prx