#pragma once

#include "prx/utilities/defs.hpp"
#include "prx/planning/planners/rrt.hpp"
#include "prx/utilities/general/timer.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/utilities/data_structures/gnn.hpp"
#include "prx/utilities/data_structures/tree.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"

#include <stack>

namespace prx
{

class aorrt_node_t : public rrt_node_t
{
public:
  aorrt_node_t() : rrt_node_t()
  {
    cost_to_go = 0;
  }
  virtual ~aorrt_node_t()
  {
  }
  virtual void copy(const aorrt_node_t& other)
  {
    this->rrt_node_t::copy(other);
    cost_to_go = other.cost_to_go;
  }

  double cost_to_go;
};

class aorrt_edge_t : public rrt_edge_t
{
public:
  aorrt_edge_t() : rrt_edge_t()
  {
  }
  virtual ~aorrt_edge_t()
  {
  }
  virtual void copy(const aorrt_edge_t& other)
  {
    this->rrt_edge_t::copy(other);
  }
};

// class cost_space_t : public space_t
// {
//   // friend class space_t;
// public:
//   cost_space_t() : space_t(), _min_cost(0.0), _max_cost(std::numeric_limits<double>::infinity()),
//   _max_multiplier(1.0)
//   {
//     owned_values = true;
//     addresses.clear();
//     addresses.push_back(&_cost);

//     dimension = addresses.size();

//     topology.clear();
//     lower_bounds.clear();
//     upper_bounds.clear();

//     topology.push_back(topology_t::EUCLIDEAN);
//     lower_bounds.push_back(&_min_cost);
//     upper_bounds.push_back(&_max_cost);

//     space_name = "CostState";

//     start_state = make_point();
//     copy(start_state, { 0 });
//   }

//   space_point_t get_start_state()
//   {
//     return start_state;
//   }

//   void set_cost(const space_point_t& pt, double cost)
//   {
//     pt->at(0) = cost;
//   }

//   virtual ~cost_space_t()
//   {
//   }

//   virtual sample(const space_point_t& point) const override
//   {
//     point->_memory[0] = prx::uniform_random(_min_cost, _max_cost * _max_multiplier);
//   }

//   double _max_cost;
//   double _max_multiplier;

// protected:
//   double _cost;
//   space_point_t start_state;
//   double _min_cost;
// };

class aorrt_specification_t : public rrt_specification_t
{
public:
  aorrt_specification_t(std::shared_ptr<system_group_t> sg, std::shared_ptr<collision_group_t> cg)
    : rrt_specification_t(sg, cg)
    , cost_multiplier(1.0)
    , c_max(std::numeric_limits<double>::infinity())
    , w_x(1.0)
    , w_c(1.0)
  {
    state_space = sg->get_state_space();

    //// Create the Y space = X U Cost

    // cost_aux1 = cost_state_space->make_point();
    // cost_aux2 = cost_state_space->make_point();
  }
  virtual ~aorrt_specification_t()
  {
  }

  virtual void init(const prx::param_loader& params) override
  {
    rrt_specification_t::init(params);
    cost_multiplier = params.exists("cost_multiplier") ? params["cost_multiplier"].as<double>() : cost_multiplier;
    w_c = params.exists("w_c") ? params["w_c"].as<double>() : w_c;
    w_x = params.exists("w_x") ? params["w_x"].as<double>() : w_x;
  }
  friend std::ostream& operator<<(std::ostream& os, const aorrt_specification_t& obj)
  {
    // rrt_specification_t::operator<<(os, obj);
    os << static_cast<rrt_specification_t>(obj);
    os << "cost_multiplier: " << obj.cost_multiplier << "\n";
    os << "w_c: " << obj.w_c << "\n";
    os << "w_x: " << obj.w_x << "\n";

    return os;
  }

  space_t* state_space;
  // space_t* Y_state_space;
  // space_t* control_space;

  // space_point_t X_aux1, X_aux2;
  // space_point_t cost_aux1, cost_aux2;

  double c_max;
  double w_c;
  double w_x;
  double cost_multiplier;  // Cost space will be sampled in [0, c_max * cost_multiplier]
};

class aorrt_query_t : public rrt_query_t
{
public:
  aorrt_query_t(space_t* state_space, space_t* control_space) : rrt_query_t(state_space, control_space)
  {
    clear_outputs();
    goal_check = [&](const space_point_t& x) { return space_t::euclidean_2d(x, goal_state) < goal_region_radius; };
  }
  virtual ~aorrt_query_t()
  {
  }
};

class aorrt_t : public rrt_t
{
public:
  using Node = aorrt_node_t;
  using Edge = aorrt_edge_t;

  aorrt_t(const std::string& new_name);
  virtual ~aorrt_t();
  virtual std::vector<double> get_statistics() override;

  double get_sln_cost()
  {
    return Y_min_cost;
  };

  void print_statics();

  virtual std::shared_ptr<prx::tree_t> tree_of_solutions() override
  {
    const double radius{ aorrt_query->goal_region_radius };
    // _cost_state_space->copy(_cost_aux_pt, { Y_min_cost });
    // Y_state_space->point_union(aorrt_query->goal_state, _cost_aux_pt, Y_aux_pt);
    const int total_solutions{ aorrt_query->total_solutions };

    std::vector<prx::proximity_node_t*> goal_nodes{};

    auto pair_iters_vertices = _tree.vertices();
    for (auto iter = pair_iters_vertices.first; iter != pair_iters_vertices.second; iter++)
    {
      std::shared_ptr<tree_node_t> tree_node{ *iter };
      std::shared_ptr<Node> curr_node{ std::dynamic_pointer_cast<Node>(tree_node) };
      Y_state_space->split_point(curr_node->point, X_aux_pt, _cost_aux_pt);
      if (distance_function(aorrt_query->goal_state, X_aux_pt) < radius)
      {
        goal_nodes.push_back(tree_node.get());
      }
    }

    // const std::vector<prx::proximity_node_t*> goal_nodes{ metric->multi_query(Y_aux_pt, total_solutions) };
    return _tree_of_solutions<Node, Edge>(goal_nodes);
  }

protected:
  distance_function_t Y_distance_function = [&](const space_point_t& s1, const space_point_t& s2) {
    double cost{ 0.0 };

    if (Y_state_space->is_point_in_space(s1) && Y_state_space->is_point_in_space(s2))
    {
      Y_state_space->split_point(s1, _X_aux1, _cost_aux1);
      Y_state_space->split_point(s2, _X_aux2, _cost_aux2);
      cost += aorrt_spec->w_x * std::pow(Y_distance_function(_X_aux1, _X_aux2), 2);
      cost += aorrt_spec->w_c * std::pow(Y_distance_function(_cost_aux1, _cost_aux2), 2);
    }
    else if (X_state_space->is_point_in_space(s1) && X_state_space->is_point_in_space(s2))
    {
      cost += distance_function(s1, s2);
    }
    else if (_cost_state_space->is_point_in_space(s1) && _cost_state_space->is_point_in_space(s2))
    {
      cost += space_t::l2_norm(s1, s2);
    }
    else
    {
      printf("Points not in the same space!\n");
      exit(1);
    }

    return std::sqrt(cost);
  };

  virtual void update_goal(node_index_t node_index) override;

  virtual void _link_and_setup_spec(planner_specification_t* spec) override;
  virtual bool _preprocess() override;
  virtual bool _link_and_setup_query(planner_query_t* query) override;
  virtual void _resolve_query(condition_check_t* condition) override;
  virtual void _fulfill_query() override;
  virtual void _reset() override;

  virtual void bnb(node_index_t v, double cost_bound, bool delete_flag = false) override;

  aorrt_query_t* aorrt_query;
  aorrt_specification_t* aorrt_spec;

  heuristic_function_t heuristic;

  space_t* Y_state_space;
  space_t* X_state_space;
  // space_t* control_space;

  space_point_t X_sample_point;
  space_point_t Y_sample_point;

  space_point_t X_aux_pt;
  space_point_t Y_aux_pt;
  space_point_t _X_aux1, _X_aux2;

  double Y_min_cost;
  double Y_min_g;
  space_point_t Y_min;

  std::vector<space_point_t> trajectory_costs;

  bool use_heuristic;

  int print_statics_count;

  // Cost space
  std::vector<double*> _cost_memory;
  space_point_t _cost_sample_point;
  space_t* _cost_state_space;
  space_point_t _cost_aux_pt;
  space_point_t _cost_aux1, _cost_aux2;

  double _c_new;
  double _w_x_bk;
  double _w_c_bk;
  double _c_max;
};
}  // namespace prx
