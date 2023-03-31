#pragma once

#include "prx/utilities/defs.hpp"
#include "prx/utilities/spaces/space.hpp"

#include <fstream>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

namespace prx
{
namespace fg
{
// Idea: Wrap somehow space_snapshot_t (or space_point_t) for gtsam to use it avoiding any deep copy
//			 and making trivial going between prx and gtsam.

class state_t : space_snapshot_t
{
  state_t(const space_t* const in_parent) : space_snapshot_t(in_parent)
  {
  }

  template <typename ValueType>
  struct traits<GenericValue<ValueType> > : public Testable<GenericValue<ValueType> >
  {
  };
}

}  // namespace fg
}  // namespace prx