#include <memory>
#include <queue>
#include "general/debug_utils.hpp"
#include "general/prx_assert.hpp"
namespace prx
{
namespace solution_update
{
struct interface_out_t
{
  bool goal_updated;
  std::size_t goal_index;
};

// Create a single edge out of a single control-duration
template <typename Output, typename Input, typename PlannerMemory>
void update_solution_if_goal_found(std::shared_ptr<Output> output, std::shared_ptr<Input> input,
                                   std::shared_ptr<PlannerMemory> memory)
{
  output->goal_updated = false;
  typename Input::NodePtr node{ memory->tree()->node(input->new_node_index) };
  if (memory->goal_check(node->state()))
  {
    const bool solution_found{ memory->statistics()->solution_found };
    const double previous_cost{ memory->statistics()->current_solution_cost };
    const double new_cost{ node->cost_to_come() };

    if (solution_found and new_cost < previous_cost)
    {
      memory->goal_node(node);
      memory->statistics()->update_solution(new_cost);

      output->goal_updated = true;
      output->goal_index = node->index();
    }
  }
}

template <typename Node>
struct tree_bnb_input
{
  using NodePtr = std::shared_ptr<Node>;
  std::size_t node_index;
  bool delete_branch;
  double cost_bound;
};

template <typename Input, typename PlannerMemory>
void tree_branch_and_bound(std::shared_ptr<Input> input, std::shared_ptr<PlannerMemory> memory)
{
  const auto node_index = input->node_index;
  typename Input::NodePtr node{ memory->tree()->node(node_index) };

  const bool delete_branch{ input->delete_branch };
  const double cost_bound{ input->cost_bound };
  const bool bound{ node->cost_to_come() > cost_bound };

  input->delete_branch = delete_branch or bound;
  for (auto& child : node->children())
  {
    input->node_index = child;
    tree_branch_and_bound(input, memory);
  }

  if (input->delete_branch)
  {
    prx_assert(node->children().size() == 0, "Cannot BnB if node has children.");
    memory->nearest_neighbors()->remove_node(node_index);
    memory->tree()->remove_node(node_index);
  }
  input->node_index = node_index;
  input->delete_branch = delete_branch;
}

}  // namespace solution_update
}  // namespace prx