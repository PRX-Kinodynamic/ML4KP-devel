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

  // virtual ~noisy_space_t(){}

  virtual void copy_to_point(const space_point_t& point) const override
  {
    // prx_assert(point->parent->space_name==space_name,"Point and space have different names:
    // "<<point->parent->space_name<<" and "<<space_name);
    space_t::copy_to_point(point);
    noise.add_noise(point);
    // for(unsigned i=0;i<dimension;++i)
    // {

    // 	(*point)[i] = noise.add_noise(*addresses[i]);
    // }
  }

  virtual void copy_to_vector(std::vector<double>& destination) const override
  {
    destination.clear();
    for (unsigned i = 0; i < dimension; ++i)
    {
      destination.push_back(noise.add_noise(*addresses[i]));
    }
  }

  virtual void copy_to_vector(Eigen::VectorXd& _v) const override
  {
    prx_assert(_v.size() == dimension, "Vector and space must have the same dimensions.");
    for (unsigned i = 0; i < dimension; ++i)
    {
      _v[i] = noise.add_noise(*addresses[i]);
    }
  }

protected:
  T noise;
};
}  // namespace prx