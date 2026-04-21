#include <memory>
#include <queue>
namespace prx
{
namespace graph_update
{
template <typename Node, typename Edge>
struct interface_out_t
{
  using NodePtr = std::shared_ptr<Node>;
  using EdgePtr = std::shared_ptr<Edge>;

  std::size_t new_node_index;
  std::size_t new_edge_index;
};

// Create a single edge out of a single control-duration
template <typename Output, typename Input, typename PlannerMemory>
void add_edge_node_to_tree(std::shared_ptr<Output> output, std::shared_ptr<Input> input,
                           std::shared_ptr<PlannerMemory> memory)
{
  if (input->candidate_edges.size() == 0)
    return;

  auto& next_candidate_edge = input->candidate_edges.front();
  typename Input::NodePtr node{ Input::node(next_candidate_edge) };
  typename Input::ControllerPtr controller{ Input::controller(next_candidate_edge) };
  typename Input::TrajectoryPtr trajectory{ Input::trajectory(next_candidate_edge) };

  const auto new_node_index = memory->tree()->add_node();
  typename Input::NodePtr new_tree_node{ memory->tree()->node(new_node_index) };
  new_tree_node->state(trajectory->back());

  memory->nearest_neighbors()->insert(new_tree_node, new_node_index);

  const auto new_edge_index = memory->tree()->add_edge(node->index(), new_node_index);
  typename Output::EdgePtr new_edge{ memory->tree()->edge(new_edge_index) };

  new_edge->controller(controller);
  new_edge->trajectory(trajectory);

  const double edge_cost{ memory->cost(trajectory, controller) };
  new_tree_node->cost_to_come(node->cost_to_come() + edge_cost);

  output->new_node_index = new_node_index;
  output->new_edge_index = new_edge_index;
}

}  // namespace graph_update

}  // namespace prx