#include <memory>
#include <queue>
#include "general/prx_assert.hpp"
namespace prx
{
namespace solution_update
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
void update_solution_if_goal_found(std::shared_ptr<Output> output, std::shared_ptr<Input> input,
                                   std::shared_ptr<PlannerMemory> memory)
{
  // auto new_tree_node = _tree.get_vertex_as<rrt_node_t>(node_index);
  typename Input::NodePtr node{ Input::node(input.new_node_index) };
  if (memory->goal_check(node->state()))
  {
    const bool solution_found{ memory->statistics()->solution_found() };
    const double previous_cost{ memory->statistics()->current_solution_cost() };
    const double new_cost{ node->cost_to_come() };

    if (solution_found and new_cost < previous_cost)
    {
      memory->update_goal(node);
      memory->statistics()->update_cost(new_cost);

      ///////////
      const double& solution_cost{ new_tree_node->cost_to_come };

      if (_bnb)
      {
        bnb(start_vertex, solution_cost);
      }
      _tree.remove_vertices();
    }
  }
}

template <typename Output, typename Input, typename PlannerMemory>
void tree_branch_and_bound(std::shared_ptr<Output> output, std::shared_ptr<Input> input,
                           std::shared_ptr<PlannerMemory> memory)
{
  typename Input::NodePtr node{ Input::node(input->node_index) };

  const bool delete_branch{ input->delete_branch };
  const double cost_bound{ input->cost_bound };
  const bool bound{ node->cost_to_come() > cost_bound };

  input->delete_branch = delete_branch or bound;
  for (auto& child : node->children())
  {
    input->node_index = child;
    tree_branch_and_bound(output, input, memory);
  }

  if (delete_branch)
  {
    prx_assert(node->children().size() == 0, "Cannot BnB if node has children.");
    memory->nearest_neighbors()->remove_node(node);
    memory->tree()->remove_node(node);
  }
  input->delete_branch = delete_branch;
}

}  // namespace solution_update
}  // namespace prx