#include <memory>
#include <queue>
namespace prx
{
namespace node_validation
{
template <typename Node, typename Plan, typename Trajectory>
class interface_out_t
{
  using NodePtr = std::shared_ptr<Node>;
};

// Create a single edge out of a single control-duration
template <typename Input, typename PlannerMemory>
void trajectory_collision_check(std::shared_ptr<Input> input, std::shared_ptr<PlannerMemory> memory)
{
  auto& next_candidate_edge = input->candidate_edges.front();

  auto trajectory = Input::trajectory(next_candidate_edge);
  auto collision_checker = memory->collision_checker();
  for (auto state : trajectory)
  {
    if (collision_checker->collision(state))
    {
      input->candidate_edges.pop();
    }
  }
}

template <typename Input, typename PlannerMemory>
void cost_to_come_check(std::shared_ptr<Input> input, std::shared_ptr<PlannerMemory> memory)
{
  auto& next_candidate_edge = input->candidate_edges.front();
  typename PlannerMemory::NodePtr node{ Input::node(next_candidate_edge) };
  typename PlannerMemory::ControllerPtr controller{ PlannerMemory::controller(next_candidate_edge) };
  typename PlannerMemory::TrajectoryPtr trajectory{ PlannerMemory::trajectory(next_candidate_edge) };

  const double edge_cost{ PlannerMemory::Edge::cost(trajectory, controller) };
  // const double edge_cost{ memory->cost_function(trajectory, controller) };
  if (node->cost_to_come() + edge_cost > memory->statistics()->current_solution_cost())
  {
    input->candidate_edges.pop();
  }
}
}  // namespace node_validation

}  // namespace prx