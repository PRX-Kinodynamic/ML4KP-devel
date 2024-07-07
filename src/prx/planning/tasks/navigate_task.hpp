#pragma once

#include "prx/utilities/defs.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/utilities/general/param_loader.hpp"

namespace prx
{

typedef std::function<double(const space_point_t&, const space_point_t&, std::vector<unsigned>)> task_distance_function_t;

// TODO: Inherit this from an abstract task_t class
class navigate_task_t
{
public:
  navigate_task_t(param_loader params, simulation_context planning_context, std::vector<double> goal_vec);
  ~navigate_task_t(){};
  // std::shared_ptr<dirt_specification_t>  get_specification();
  // std::shared_ptr<dirt_query_t> get_query();

  dirt_specification_t* get_specification();
  dirt_query_t* get_query();

  plan_t get_solution_plan();
  simulation_context context;
  param_loader params;

private:
  std::shared_ptr<dirt_specification_t> spec;
  std::shared_ptr<dirt_query_t> query;
  std::vector<unsigned> pos_indices;
  std::vector<unsigned> vel_indices;
  task_distance_function_t goal_distance_function;
  double goal_pos_tolerance, goal_vel_tolerance;
  void _prepare_specification();
  void _prepare_query(std::vector<double> goal_vec);
  void _set_bounds(std::vector<double> env_xlim, std::vector<double> env_ylim);
};
}  // namespace prx