#include <memory>
#include <queue>
namespace prx
{
namespace node_expansion
{
template <typename Node, typename ControllerIn, typename Trajectory>
struct interface_out_t
{
  using NodePtr = std::shared_ptr<Node>;
  using Controller = ControllerIn;
  using ControllerPtr = std::shared_ptr<Controller>;
  using TrajectoryPtr = std::shared_ptr<Trajectory>;
  using CandidateEdge = std::tuple<NodePtr, ControllerPtr, TrajectoryPtr>;

  static NodePtr node(CandidateEdge& candidate)
  {
    return std::get<0>(candidate);
  }

  static ControllerPtr controller(CandidateEdge& candidate)
  {
    return std::get<1>(candidate);
  }

  static TrajectoryPtr trajectory(CandidateEdge& candidate)
  {
    return std::get<2>(candidate);
  }
  // FIFO -  Expanded nodes
  std::queue<CandidateEdge> candidate_edges;
};

// Create a single edge out of a single control-duration
template <typename Output, typename Input, typename PlannerMemory>
void single_random_expansion(std::shared_ptr<Output> output, std::shared_ptr<Input> input,
                             std::shared_ptr<PlannerMemory> memory)
{
  // const auto ctrl = memory->control_space()->sample();
  // const auto duration = memory->sample_step();

  const typename Output::NodePtr node{ input->nodes_to_expand.front() };
  const auto x0 = node->state();
  input->nodes_to_expand.pop();

  const typename Output::ControllerPtr controller{ memory->template sample<typename Output::Controller>() };
  // typename Output::TrajectoryPtr trajectory{ PlannerMemory::Trajectory::create() };

  const typename Output::TrajectoryPtr trajectory{ memory->propagate(x0, controller) };

  output->candidate_edges.push(std::make_tuple(node, controller, trajectory));
}
}  // namespace node_expansion

}  // namespace prx