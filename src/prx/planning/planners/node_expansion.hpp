#include <memory>
#include <queue>
namespace prx
{
namespace node_expansion
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
void single_piecewise_random(std::shared_ptr<Output> output, std::shared_ptr<Input> input,
                             std::shared_ptr<PlannerMemory> memory)
{
  const auto ctrl = memory->control_space()->sample();
  const auto duration = memory->sample_step();
  const auto node = input->nodes_to_expand.front();
  const auto x0 = node->state();
  input->nodes_to_expand.pop();

  typename Output::ControllerPtr controller{ PlannerMemory::Controller::create(ctrl, duration) };
  typename Output::TrajectoryPtr trajectory{ PlannerMemory::Trajectory::create() };

  memory->simulator()->propagate(trajectory, x0, plan);

  output->candidate_edges.push(std::make_tuple(node, plan, trajectory));
}
}  // namespace node_expansion

}  // namespace prx