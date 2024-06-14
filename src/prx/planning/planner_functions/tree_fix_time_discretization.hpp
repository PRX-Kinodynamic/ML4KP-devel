#pragma once

#include "prx/utilities/defs.hpp"
#include "prx/utilities/data_structures/tree.hpp"

#include "prx/simulation/playback/plan.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"

namespace prx
{
namespace planning
{

template <typename EdgePtr, typename Planner>
void discretize_edge(EdgePtr& edge, tree_t& tree, const double desired_edge_duration, Planner& planner)
{
  using Node = typename Planner::Node;
  using Edge = typename Planner::Edge;
  using NodePtr = std::shared_ptr<Node>;

  // PRX_DEBUG_PRINT;
  std::shared_ptr<plan_t> current_plan{ edge->plan };
  NodePtr previous_node{ tree.get_vertex_as<Node>(edge->get_source()) };
  EdgePtr current_edge{ edge };
  // prx_assert(previous_node != nullptr, "Node [" << previous_edge->get_index() << "] not found");
  NodePtr final_node{ tree.get_vertex_as<Node>(edge->get_target()) };
  while (current_plan->duration() > desired_edge_duration)
  {
    // PRX_DBG_VARS(current_plan->duration(), desired_edge_duration);

    // prx_assert(new_edge != nullptr, "Edge [" << previous_edge->get_index() << "] not found");

    EdgePtr e1{ planner.split_edge(current_edge, desired_edge_duration) };
    // NodePtr new_node{ tree.get_vertex_as<Node>(new_edge->get_source()) };

    // PRX_DBG_VARS(previous_node->get_index(), new_edge->get_index(), next_node->get_index());
    current_edge = e1;
    // next_node->point = state_space->clone_point(traj.back());
    // prx_assert(previous_node->point != nullptr, "Node point [" << previous_node->get_index() << "] not found");
    // prx_assert(next_node->point != nullptr, "Node point [" << next_node->get_index() << "] not found");

    // current_plan = new_edge->plan;
    // previous_edge = new_edge;
    // previous_node = next_node;
  }
  // PRX_DBG_VARS(current_edge->traj->size());
  // NodePtr next_node{ tree.get_vertex_as<Node>(previous_edge->get_target()) };
}

template <typename Planner>
void discretize_tree(tree_t& tree, Planner& planner, const double desired_edge_duration)
{
  using Node = typename Planner::Node;
  using Edge = typename Planner::Edge;
  using EdgePtr = std::shared_ptr<Edge>;

  auto edges_iter_pairs = tree.edges();
  const tree_t::const_edge_iterator last_iterator{ edges_iter_pairs.second };
  for (auto iter = edges_iter_pairs.first; iter != last_iterator; iter++)
  {
    EdgePtr edge{ std::dynamic_pointer_cast<Edge>(*iter) };
    discretize_edge(edge, tree, desired_edge_duration, planner);
  }
}

}  // namespace planning
}  // namespace prx