#include <memory>
#include <queue>
namespace prx
{
namespace node_selection
{
// Interface (I/O) of a node selection method
// Don't need to use *this* definition, it gives the minimum implementation
template <typename Node>
struct interface_out_t
{
  using NodePtr = std::shared_ptr<Node>;

  NodePtr sample;
  // FIFO -  Nodes selected for expansion
  std::queue<NodePtr> nodes_to_expand;
};

// Interface Requirements:
//  * [In] Nothing
//  * [Out] std::queue - type structure
template <typename Output, typename PlannerMemory>
static void voronoi_single_random(std::shared_ptr<Output> output, std::shared_ptr<PlannerMemory> memory)
{
  memory->sample(output->sample);
  const typename Output::NodePtr selected_node{ memory->nearest_neighbors()->closest_neighbor(output->sample).first };
  output->nodes_to_expand.push(selected_node);
}
};  // namespace node_selection

}  // namespace prx
// #include <prx/planning/planners/node_selection-inl.hpp>
