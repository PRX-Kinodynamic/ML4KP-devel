namespace prx
{
namespace node_expansion
{
// Create a single edge out of a single control-duration
template <typename PlannerMemoryPtr>
void single_piecewise_random(PlannerMemoryPtr memory)
{
  const auto ctrl = memory->control_space()->sample();
  const auto duration = memory->sample_step();
  const auto node = memory->next_node_to_expand();
  const auto x0 = node->state();
  memory->simulator()->propagate(x0, plan, trajectory);
}
}  // namespace node_expansion

}  // namespace prx