#pragma once

#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/general/noise.hpp"
#include <vector>
#include <string>
#include <memory>
#include <numeric>

namespace prx
{
template <class T>
class noisy_space_t : public space_t
{
public:
  noisy_space_t(const std::string& topology, const std::vector<double*>& addresses, const std::string& name) = delete;

  noisy_space_t(const std::string& topology, const std::vector<double*>& addresses) = delete;

  noisy_space_t(const std::vector<const space_t*>& spaces) = delete;

  template <class... Types>
  noisy_space_t(const space_t* _space, Types... args) : space_t(_space), noise(args...)
  {
  }

  template <typename Pt>
  void add_noise(Pt& pt) const
  {
    noise.add_noise(pt);
  }

  virtual void operator()() const override
  {
    noise.add_noise(this);
  }

  const T& get_noise()
  {
    return noise;
  }

protected:
  T noise;
};
}  // namespace prx