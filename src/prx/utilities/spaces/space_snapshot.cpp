#include "prx/utilities/spaces/space_snapshot.hpp"
#include "prx/utilities/spaces/space.hpp"
namespace prx
{

space_snapshot_t::space_snapshot_t(const space_t* const in_parent, const std::size_t dim)
  : _parent(in_parent), _memory(dim), _map_vector(_memory.data(), dim, 1)
{
}

prx::param_loader space_snapshot_t::init()
{
  std::vector<double> values(_memory.size(), 0.0);
  _parent->copy(values, *this);
  prx::param_loader params{};
  params.set(values);
  return params;
}

void space_snapshot_t::init(const prx::param_loader& params)
{
  const std::vector<double> values{ params.as<std::vector<double>>() };
  _parent->copy(*this, values);
}

bool space_snapshot_t::step(const double step_inc)
{
  return step(step_inc, _parent->get_lower_bounds(), _parent->get_upper_bounds());
}

}  // namespace prx