#pragma once
#include "prx/simulation/playback/plan.hpp"
#include "prx/simulation/playback/trajectory.hpp"

namespace prx
{
namespace simulation
{
// Traverse a tree collecting the stored trajectories and plans.
// This method is slow because it requires copying many trajs/plans (as many as leafs are in the tree)
// Should only be used "offline" (collecting data).

// Recursive call from somewhere in the middle of the tree.
template <typename Node, typename Edge, typename TreePtr>
std::vector<std::pair<prx::trajectory_t, prx::plan_t>> tree_to_trajs_and_plans(const TreePtr tree,
                                                                               const node_index_t idx,
                                                                               prx::trajectory_t traj, prx::plan_t plan)
{
  using VectorTrajsPlans = std::vector<std::pair<prx::trajectory_t, prx::plan_t>>;
  VectorTrajsPlans res;
  // std::pair<prx::trajectory_t, prx::plan_t>
  const edge_index_t parent_edge{ tree->operator[](idx)->get_parent_edge() };

  plan += *(tree->template get_edge_as<Edge>(parent_edge)->plan);
  traj += *(tree->template get_edge_as<Edge>(parent_edge)->traj);

  // const Node node{ tree->template get_vertex_as<Node>(idx) };
  // for (int i = 0; i < node.get_children(); ++i)
  const auto children{ tree->template get_vertex_as<Node>(idx)->get_children() };
  if (children.size() > 0)
  {
    for (auto child : children)
    {
      const VectorTrajsPlans next{ tree_to_trajs_and_plans<Node, Edge>(tree, child, traj, plan) };
      res.insert(res.end(), next.begin(), next.end());
    }
  }
  else
  {
    res.emplace_back(traj, plan);
  }
  return res;
}

// Call this one from the starting point from which to collect trajectories / plans
template <typename Node, typename Edge, typename TreePtr>
std::vector<std::pair<prx::trajectory_t, prx::plan_t>> tree_to_trajs_and_plans(const TreePtr tree,
                                                                               const node_index_t idx)
{
  using VectorTrajsPlans = std::vector<std::pair<prx::trajectory_t, prx::plan_t>>;
  VectorTrajsPlans res;
  const auto children{ tree->template get_vertex_as<Node>(idx)->get_children() };
  for (auto child : children)
  {
    const edge_index_t parent_edge{ tree->operator[](child)->get_parent_edge() };
    // Need to init plan and traj with *something*. Start with empty plan.
    prx::plan_t plan{ *(tree->template get_edge_as<Edge>(parent_edge)->plan) };
    prx::trajectory_t traj{ *(tree->template get_edge_as<Edge>(parent_edge)->traj) };
    plan.clear();
    traj.clear();

    const VectorTrajsPlans next{ tree_to_trajs_and_plans<Node, Edge>(tree, child, traj, plan) };
    res.insert(res.end(), next.begin(), next.end());
  }
  return res;
}
}  // namespace simulation
}  // namespace prx