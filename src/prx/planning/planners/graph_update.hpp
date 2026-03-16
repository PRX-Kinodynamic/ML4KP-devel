#include <memory>
#include <queue>
namespace prx
{
namespace graph_update
{
template <typename Node, typename Plan, typename Trajectory>
class interface_out_t
{
  using NodePtr = std::shared_ptr<Node>;
  using PlanPtr = std::shared_ptr<Plan>;
  using TrajectoryPtr = std::shared_ptr<Trajectory>;
  using CandidateEdge = std::tuple<NodePtr, Plan, Trajectory>;

  static NodePtr node(CandidateEdge& candidate)
  {
    return std::get<0>(candidate);
  }

  static PlanPtr plan(CandidateEdge& candidate)
  {
    return std::get<1>(candidate);
  }

  static TrajectoryPtr trajectory(CandidateEdge& candidate)
  {
    return std::get<1>(candidate);
  }
  // FIFO -  Expanded nodes
  std::queue<CandidateEdge> candidate_edges;
};

// Create a single edge out of a single control-duration
template <typename Output, typename Input, typename PlannerMemory>
void add_edge_node_to_tree(std::shared_ptr<Output> output, std::shared_ptr<Input> input,
                           std::shared_ptr<PlannerMemory> memory)
{
  auto& next_candidate_edge = input->candidate_edges.front();
  typename Input::NodePtr node{ Input::node(next_candidate_edge) };
  typename Input::ControllerPtr controller{ Input::controller(next_candidate_edge) };
  typename Input::TrajectoryPtr trajectory{ Input::trajectory(next_candidate_edge) };

  const auto new_node_index = memory->tree().add_node();
  typename Input::NodePtr new_tree_node{ memory->tree().node(new_node_index) };
  new_tree_node->state(trajectory.back());

  memory->nearest_neighbors()->add_node(new_tree_node);

  const auto new_edge_index = memory->tree().add_edge(node->index(), new_node_index);
  typename Input::EdgePtr new_edge{ memory->tree().edge(new_edge_index) };

  new_edge->controller(controller);
  new_edge->trajectory(trajectory);

  const double edge_cost{ PlannerMemory::Edge::cost(trajectory, controller) };
  new_tree_node->cost_to_come(node->cost_to_come() + edge_cost);

  output.new_node_index = new_node_index;
  output.new_edge_index = new_edge_index;
}

}  // namespace graph_update

}  // namespace prx