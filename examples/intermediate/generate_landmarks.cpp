#ifndef TORCH_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/simulation/plants/plants.hpp"

#include <boost/filesystem.hpp>
using namespace boost::filesystem;
using namespace prx;

int main(int argc, char* argv[])
{
  std::string params_file;
  if (argc < 2)
  {
    params_file = "examples/replanning/planning_simulation.yaml";
  }
  else
  {
    params_file = argv[1];
  }

  auto params = param_loader(params_file);
  simulation_step = params["simulation_step"].as<double>();
  init_random(params["random_seed"].as<int>());

  auto obstacles = load_obstacles(params["environment"].as<>());
  std::vector<std::shared_ptr<movable_object_t>> obstacle_list = obstacles.second;
  std::vector<std::string> obstacle_names = obstacles.first;

  std::string plant_name = params["/plant/name"].as<>();
  std::string plant_path = params["/plant/path"].as<>();
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
  prx_assert(plant != nullptr, "Plant is nullptr!");

  world_model_t world_model({ plant }, { obstacle_list });
  world_model.create_context("dirt_context", { plant_name }, { obstacle_names });

  auto context = world_model.get_context("dirt_context");
  auto ss = context.first->get_state_space();

  std::vector<double> theta_vals = { 0, PRX_PI / 2, PRX_PI, -PRX_PI / 2 };

  std::vector<double> environment_xlims = params["/roadmap/xlims"].as<std::vector<double>>();
  std::vector<double> environment_ylims = params["/roadmap/ylims"].as<std::vector<double>>();
  int xres = params["/roadmap/xres"].as<int>();
  int yres = params["/roadmap/yres"].as<int>();
  std::cout << xres << " " << yres << std::endl;
  std::vector<double> x_vals = linspace(environment_xlims[0], environment_xlims[1], xres);
  std::vector<double> y_vals = linspace(environment_ylims[0], environment_ylims[1], yres);

  dirt_specification_t dirt_spec(context.first, context.second);

  dirt_query_t dirt_query(context.first->get_state_space(), context.first->get_control_space());
  dirt_query.goal_state = context.first->get_state_space()->make_point();
  dirt_query.start_state = context.first->get_state_space()->make_point();
  dirt_query.goal_region_radius = params["goal_region_radius"].as<double>();
  dirt_query.goal_check = [&](space_point_t point) {
    double diff2 = (point->at(0) - dirt_query.goal_state->at(0)) * (point->at(0) - dirt_query.goal_state->at(0)) +
                   (point->at(1) - dirt_query.goal_state->at(1)) * (point->at(1) - dirt_query.goal_state->at(1));
    double angdiff2 = norm_angle_pi(point->at(2) - dirt_query.goal_state->at(2)) *
                      norm_angle_pi(point->at(2) - dirt_query.goal_state->at(2));
    diff2 += angdiff2;
    return std::sqrt(diff2) < dirt_query.goal_region_radius;
  };

  std::string roadmap_dir = lib_path + "resources/roadmaps/" + params["/roadmap/dir"].as<std::string>();
  if (!exists(roadmap_dir))
  {
    create_directories(roadmap_dir);
  }
  std::string roadmap_path = roadmap_dir + "landmarks.txt";

  std::ofstream landmark_file;
  landmark_file.open(roadmap_path);

  space_point_t config_space_point = ss->make_point();
  for (auto x : x_vals)
  {
    for (auto y : y_vals)
    {
      for (auto t : theta_vals)
      {
        config_space_point->at(0) = x;
        config_space_point->at(1) = y;
        config_space_point->at(2) = t;

        if (dirt_spec.valid_state(config_space_point))
        {
          landmark_file << ss->print_point(config_space_point, 4) << std::endl;
        }
      }
    }
  }
  landmark_file.close();
}
#else
#endif