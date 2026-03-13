namespace prx
{
namespace node_selection
{
template <typename PlannerMemoryPtr>
void voronoi_single_random(PlannerMemoryPtr memory)
{
  const auto point = memory->state_space()->sample();
  const auto selected_node = memory->nearest_neighbors()->single_query(point);
  memory->nodes_to_expand.push_back(selected_node);
}

}  // namespace node_selection

}  // namespace prx
#include <prx/planning/planners/node_selection-inl.hpp>
