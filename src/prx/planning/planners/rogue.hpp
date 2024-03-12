#pragma once

#include "prx/planning/planners/dirt.hpp"

namespace prx
{
class rogue_node_t : public dirt_node_t
{
public:
  rogue_node_t() : dirt_node_t()
  {
    local_goal_index = -1;
    roadmap_cost_to_go = PRX_INFINITY;
  }
  virtual ~rogue_node_t()
  {
  }
  double roadmap_cost_to_go;
  int local_goal_index;
};

typedef std::function<void(rogue_node_t*, std::vector<plan_t*>&, std::vector<trajectory_t*>&)> node_expand_t;
typedef std::function<double(const space_point_t&)> roadmap_heuristic_t;

class rogue_specification_t : public dirt_specification_t
{
public:
  rogue_specification_t(std::shared_ptr<system_group_t> sg, std::shared_ptr<collision_group_t> cg)
    : dirt_specification_t(sg, cg)
  {
    start_node_local_goal = -1;
  }
  virtual ~rogue_specification_t()
  {
  }
  node_expand_t node_expand;
  roadmap_heuristic_t roadmap_heuristic;
  int start_node_local_goal;
};

class rogue_query_t : public dirt_query_t
{
public:
  rogue_query_t(space_t* state_space, space_t* control_space) : dirt_query_t(state_space, control_space)
  {
  }
  virtual ~rogue_query_t()
  {
  }
};

class rogue_t : public dirt_t
{
public:
  rogue_t(const std::string& new_name);
  virtual ~rogue_t();

protected:
  virtual void update_goal(node_index_t node_index) override;
  virtual void _link_and_setup_spec(planner_specification_t* spec) override;
  virtual bool _link_and_setup_query(planner_query_t* query) override;
  virtual void _reset() override;
  virtual bool _preprocess() override;
  virtual void _resolve_query(condition_check_t* condition_check) override;

  virtual void bnb(node_index_t v, double cost_bound, bool delete_flag = false) override;

  rogue_specification_t* rogue_spec;
  rogue_query_t* rogue_query;

private:
  heuristic_function_t h;
  roadmap_heuristic_t roadmap_heuristic;
  node_expand_t node_expand;

  double max_radius;
  bool child_extension;
  node_index_t previous_child;

  void add_edge_to_tree(std::pair<plan_t*, trajectory_t*> eg, rogue_node_t* closest_node,
                        std::vector<rogue_node_t*> dir_updates, double new_node_dir_radius,
                        condition_check_t* condition);

  rogue_node_t* get_vertex(node_index_t v) const
  {
    return tree.get_vertex_as<rogue_node_t>(v).get();
  }

  bool is_leaf(node_index_t v)
  {
    return (get_vertex(v)->get_children().empty());
  }
};
}  // namespace prx