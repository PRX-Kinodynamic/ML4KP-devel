#include <fstream>

#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planner_functions/planner_functions.cpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"

int main(int argc, char* argv[])
{
  prx::param_loader params("executables/peg_in_hole.yaml", argc, argv);

  prx::simulation_step = params["simulation_step"].as<double>();
  prx::init_random(params["random_seed"].as<int>());

  prx::PairNameObstacles obstacles{ prx::load_obstacles(params["environment"].as<>()) };
  const std::vector<std::shared_ptr<prx::movable_object_t>> obstacle_list{ obstacles.second };
  const std::vector<std::string> obstacle_names{ obstacles.first };

  const std::string plant_name{ params["/plant/name"].as<>() };
  const std::string plant_path{ params["/plant/path"].as<>() };
  prx::system_ptr_t plant{ prx::system_factory_t::create_system(plant_name, plant_path) };
  prx_assert(plant != nullptr, "Plant is nullptr!");

  prx::world_model_t world_model({ plant }, { obstacle_list });
  world_model.create_context("context", { plant_name }, { obstacle_names });
  auto context = world_model.get_context("context");
  std::shared_ptr<prx::system_group_t> sys_group{ context.first };

  prx::space_t* ss{ sys_group->get_state_space() };
  prx::space_t* cs{ sys_group->get_control_space() };
  prx::space_t* ps{ sys_group->get_parameter_space() };

  const std::vector<double> ss_lower_bounds{ params["/plant/state_space/lower_bound"].as<std::vector<double>>() };
  const std::vector<double> ss_upper_bounds{ params["/plant/state_space/upper_bound"].as<std::vector<double>>() };

  const std::vector<double> cs_lower_bounds{ params["/plant/control_space/lower_bound"].as<std::vector<double>>() };
  const std::vector<double> cs_upper_bounds{ params["/plant/control_space/upper_bound"].as<std::vector<double>>() };

  const std::vector<double> ps_values{ params["/plant/parameter_space/values"].as<std::vector<double>>() };

  ss->set_bounds(ss_lower_bounds, ss_upper_bounds);
  cs->set_bounds(cs_lower_bounds, cs_upper_bounds);

  ps->copy_from(ps_values);

  prx::space_point_t query_point = ss->make_point();
  ss->copy(query_point, params["/plant/start_state"].as<std::vector<double>>());

  auto pqp_distance_result = prx::default_obstacle_distance_function(query_point, ss, context.second);
  std::cout << "Distances of peg to all the rigid bodies: " << std::endl;
  for (auto d : pqp_distance_result.distances)
  {
    std::cout << d << std::endl;
  }

  std::cout << "Closest points to all the rigid bodies from the peg: " << std::endl;
  for (auto p : pqp_distance_result.closest_points)
  {
    for (auto c : p)
    {
      std::cout << c << " ";
    }
    std::cout << std::endl;
  }
}
