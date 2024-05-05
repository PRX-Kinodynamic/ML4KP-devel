#include "pyprx/simulation/plants/types/linear_time_invariant_py.hpp"
#include "pyprx/simulation/plants/types/linear_time_variant_py.hpp"
#include "pyprx/simulation/plants/types/noisy_plant_py.hpp"

namespace pyprx
{
namespace simulation
{
namespace plants
{
namespace types
{
void bindings()
{
  lti::bindings();
  ltv::bindings();
  noisy_plant::bindings();
}

}  // namespace types
}  // namespace plants
}  // namespace simulation
}  // namespace pyprx