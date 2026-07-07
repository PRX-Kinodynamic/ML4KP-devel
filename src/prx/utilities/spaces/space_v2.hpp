#pragma once

#include <vector>
#include <string>
#include <memory>
#include <numeric>

#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/debug_utils.hpp"
#include "prx/utilities/general/template_utils.hpp"
#include "prx/utilities/general/type_conversions.hpp"
#include <prx/utilities/spaces/sampler.hpp>

namespace prx
{
namespace experimental
{
template <typename Element>
class space_t
{
public:
  using State = Element;
  using Space = space_t<State>;
  using SpacePtr = std::shared_ptr<Space>;
  using Sampler = prx::sampler_t<Element>;

  // space_t(prx::param_loader params) : sampler(params["bounds"])
  space_t(prx::param_loader params) : sampler(params.exists("bounds") ? params["bounds"] : prx::param_loader())
  {
  }
  template <typename SamplerBounds>
  space_t(const SamplerBounds min, const SamplerBounds max) : sampler(min, max)
  {
  }

  template <typename... Args>
  static SpacePtr create(Args... args)
  {
    return std::make_shared<Space>(args...);
  }

  Sampler sampler;
};
}  // namespace experimental
}  // namespace prx