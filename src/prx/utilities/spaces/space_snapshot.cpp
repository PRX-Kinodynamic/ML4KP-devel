#include "prx/utilities/spaces/space_snapshot.hpp"
#include "prx/utilities/spaces/space.hpp"
namespace prx
{

space_snapshot_t::space_snapshot_t(const space_t* const in_parent, const std::size_t dim)
  : _parent(in_parent), _memory(dim), _map_vector(_memory.data(), dim, 1)
{
}
void space_snapshot_t::init(const prx::param_loader& params)
{
  const std::vector<double> values{ params.as<std::vector<double>>() };
  _parent->copy(*this, values);
}
}  // namespace prx