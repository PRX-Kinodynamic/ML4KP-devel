#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>

#include <boost/algorithm/string/join.hpp>
#include <boost/optional.hpp>

#include "prx/planning/loaders/planner_loader.hpp"
#include "prx/planning/planners/rrt.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"
#include "prx/planning/planner_statistics.hpp"
#include "prx/planning/world_model.hpp"

#include "prx/simulation/controllers/custom_controller.hpp"
#include "prx/simulation/general/condition_check.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/simulation/plants/types/noisy_plant.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/simulation/system_factory.hpp"
#include "prx/simulation/system_group.hpp"

#include "prx/utilities/geometry/regular_grid.hpp"
#include "prx/utilities/general/range.hpp"
#include "prx/utilities/math/first_order_derivative.hpp"

#include "prx/visualization/three_js_group.hpp"

#include "prx/factor_graphs/defs.hpp"
#include "prx/factor_graphs/utilities/fg_logger.hpp"
#include "prx/factor_graphs/factors/friction_fusion_factor.hpp"
#include "prx/factor_graphs/factors/parameter_fusion_factor.hpp"

#include "prx/factor_graphs/utilities/utilities_functions.hpp"
#include "prx/factor_graphs/utilities/formatter.hpp"

#include <gtsam/nonlinear/Marginals.h>

#include "friction_maps.hpp"
namespace fs = std::filesystem;
using namespace prx;

const int X_DIM{ 3 };
const int U_DIM{ 2 };
const int TH_DIM{ 1 };
const int GRID_DIVISIONS{ 4 };
const int BASIS_DIM{ (GRID_DIVISIONS + 1) * (GRID_DIVISIONS + 1) };

const double x_max{ 3.0 };
const double y_max{ 3.0 };

using state_t = Eigen::Vector<double, X_DIM>;
using friction_vector_t = Eigen::Vector<double, TH_DIM>;
using basis_vector_t = Eigen::Vector<double, BASIS_DIM>;
// using friction_factor_t = fg::friction_fusion_factor_t<TH_DIM, BASIS_DIM>;

// using friction_vector_t = Eigen::Vector4d;

template <typename FileMap, typename LogMap>
void init_files(FileMap& files, LogMap& logs)
{
  const std::string fm_out_dir = out_path + "friction_maps/ekf/";
  files["traj_real_file"] = fm_out_dir + "fmbasis_trajs_real.txt";
  for (auto f : files)
  {
    std::remove(files[f.first].c_str());
    logs.emplace(f.first, f.second);
  }
}

int main(int argc, char* argv[])
{
  auto params = param_loader("executables/factor_graphs/ekf_friction_map.yaml", argc, argv);
  simulation_step = params["simulation_step"].as<double>();
  init_random(params["random_seed"].as<int>());

  std::string plant_name = params["/plant/name"].as<>();
  std::string plant_path = params["/plant/path"].as<>();

  auto obstacles = load_obstacles(params["environment"].as<>());

  auto obstacle_list = obstacles.second;
  auto obstacle_names = obstacles.first;

  auto system = system_factory_t::create_system(plant_name, plant_path);
  auto plant = std::dynamic_pointer_cast<plant_t>(system);
  world_model_t world_model({ plant }, {});
  world_model.create_context("context", { plant_name }, {});

  auto context = world_model.get_context("context");
  auto sg = context.first;
  space_t* ss = sg->get_state_space();
  space_t* cs = sg->get_control_space();
  space_t* ps = sg->get_parameter_space();
  const std::size_t ss_dim = ss->get_dimension();
  const std::size_t cs_dim = cs->get_dimension();
  const std::size_t ps_dim = ps->get_dimension();
  prx_assert(ps != nullptr, "Parameter space is null!!!");

  auto cs_lb = params["/plant/control_space_lower_bound"].as<std::vector<double>>();
  auto cs_up = params["/plant/control_space_upper_bound"].as<std::vector<double>>();
  cs->set_bounds(cs_lb, cs_up);
  const int map_id{ params["map_id"].as<int>() };

  std::unordered_map<std::string, std::string> files;
  std::unordered_map<std::string, prx::logger_t> logs;
  init_files(files, logs);

  const std::vector<std::pair<double, double>> env_bounds{
    std::make_pair(0.0, friction_maps::friction_maps_t::x_max_map[map_id]),
    std::make_pair(0.0, friction_maps::friction_maps_t::y_max_map[map_id])
  };

  prx::regular_grid_t<friction_maps::FrictionVector, 2> frictions_grid{ env_bounds, GRID_DIVISIONS };

  world_functions_t world_functions(sg);
  WorldFunction real_world = world_functions.get_real_world(friction_maps::friction_map1_at);
  // WorldFunction real_world = world_functions.get_real_world(friction_maps::friction_map2_at);
  // WorldFunction real_world = get_real_world(ss, ps, friction_maps::friction_map1_at);
  WorldFunction unaware_world = []() {};

  world_model.world_change_function = real_world;

  const double x_max{ friction_maps::friction_maps_t::x_max_map[map_id] };
  const double y_max{ friction_maps::friction_maps_t::y_max_map[map_id] };
  // const double grid_stepping{ 1.0 / static_cast<double>(GRID_DIVISIONS) };
  friction_maps::friction_map_to_file(map_id, 0.01, friction_maps::friction_map1_at);
}