#include <memory>
#include <queue>
namespace prx
{
namespace query_answer
{
template <typename Node, typename Plan, typename Trajectory>
class interface_out_t
{
  using NodePtr = std::shared_ptr<Node>;
};

// Create a single edge out of a single control-duration
template <typename PlannerQuery, typename PlannerMemory>
void trajectory_in_goal(std::shared_ptr<PlannerQuery> query, std::shared_ptr<PlannerMemory> memory)
{
  if (memory->statistics()->solution_found())
  {
    typename PlannerMemory::NodePtr start_node{ memory->start() };
    typename PlannerMemory::NodePtr goal_node{ memory->goal() };
    std::deque<typename PlannerMemory::NodeIndex> solution_indices;

    typename PlannerMemory::NodeIndex start_index{ start_node->index() };
    typename PlannerMemory::NodeIndex current_index{ goal_node->index() };

    while (current_index != start_index)
    {
      solution_indices.push_front(current_index);
      current_index = memory->tree()->node(current_index)->parent();
    }
    query->solution_controller.clear();
    query->solution_trajectory.clear();
    for (int i = 1; i < solution_indices.size(); i++)
    {
      const typename PlannerMemory::NodePtr node{ memory->tree()->node(solution_indices[i]) };
      const typename PlannerMemory::EdgePtr edge{ memory->tree()->edge(node->parent_edge()) };

      query->solution_controller.push_back(edge->controller());
      query->solution_trajectory.push_back(edge->trajectory());
    }
  }

  ////////////////////////

  // rrt_query->solution_plan = *_tree.get_edge_as<rrt_edge_t>(_tree[node_indices[0]]->get_parent_edge())->plan;
  // rrt_query->solution_traj = *_tree.get_edge_as<rrt_edge_t>(_tree[node_indices[0]]->get_parent_edge())->traj;

  // for (int i = 1; i < node_indices.size(); i++)
  // {
  //   rrt_query->solution_traj.resize(rrt_query->solution_traj.size() - 1);
  //   rrt_query->solution_plan += *_tree.get_edge_as<rrt_edge_t>(_tree[node_indices[i]]->get_parent_edge())->plan;
  //   rrt_query->solution_traj += *_tree.get_edge_as<rrt_edge_t>(_tree[node_indices[i]]->get_parent_edge())->traj;
  // }
  // }
  // else
  // {
  //   rrt_query->solution_cost = 0;
  // }
}
}  // namespace query_answer

}  // namespace prx