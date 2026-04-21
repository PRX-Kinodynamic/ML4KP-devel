#pragma once
#include <memory>
#include <queue>
// #include "prx/utilities/defs.hpp"

namespace prx
{
namespace planner_postprocessing
{
// Interface (I/O) of a node selection method
// Don't need to use *this* definition, it gives the minimum implementation
template <typename Controller, typename Trajectory, typename EdgeIn>
struct interface_out_t
{
  using Edge = EdgeIn;
  using ControllerPtr = std::shared_ptr<Controller>;
  using TrajectoryPtr = std::shared_ptr<Trajectory>;

  Trajectory trajectory;
  Controller controller;
};

// Interface Requirements:
//  * [In] Nothing
//  * [Out] std::queue - type structure
template <typename Output, typename PlannerMemory>
static void recover_solution(std::shared_ptr<Output> output, std::shared_ptr<PlannerMemory> memory)
{
  auto node = memory->goal_node();
  if (node == nullptr)
    return;

  output->trajectory.clear();
  output->controller.clear();

  while (node != memory->root_node())
  {
    auto edge_idx = node->parent_edge();
    auto edge = memory->tree()->edge(edge_idx);
    auto edge_trajectory = edge->trajectory();
    auto edge_controller = edge->controller();
    prx::merge(output->trajectory, *edge_trajectory);
    prx::merge(output->controller, *edge_controller);

    node = memory->tree()->node(node->parent);
  }
}
};  // namespace planner_postprocessing

}  // namespace prx
// #include <prx/planning/planners/node_selection-inl.hpp>
