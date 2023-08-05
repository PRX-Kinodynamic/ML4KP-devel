#include "prx/utilities/spaces/space_snapshot.hpp"

namespace prx
{

space_snapshot_t::space_snapshot_t(const space_t* const in_parent, const std::size_t dim)
  : _parent(in_parent), _memory(dim), _map_vector(_memory.data(), dim, 1)
{
}

}  // namespace prx