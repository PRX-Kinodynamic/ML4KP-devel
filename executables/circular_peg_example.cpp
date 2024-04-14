#include <fstream>

#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/rrt_star.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/utilities/geometry/basic_geoms/box.hpp"
using ObstacleList = std::vector<std::shared_ptr<prx::movable_object_t>>;
using ObstacleNames = std::vector<std::string>;


void generate_peg(ObstacleList& obstacle_list, ObstacleNames& obstacle_names)
{
  prx::transform_t pose;
  const double step{ 0.2 };
  const double radius{ 10 };
  const std::string blue{ "0xE0FF0000" };
  const Eigen::Vector3d box(2.0, 2.0, 24.0);
  for (double th = 0.0; th < 2 * prx::constants::pi; th += step)
  {
    const double x{ std::cos(th) * (radius + box[0] / 2) };
    const double y{ std::sin(th) * (radius + box[1] / 2) };
    pose.setIdentity();
    pose.translation() = (prx::vector_t(x, y, 11.5));
    pose.linear() = Eigen::AngleAxisd(th, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    const std::string name{ "box_" + std::to_string(th) };
    obstacle_names.push_back(name);
    obstacle_list.push_back(prx::create_obstacle(new prx::box_t(name, box[0], box[1], box[2], pose)));
  }
  // Create walls
  const std::string wall_A{ "box_A" };
  obstacle_names.push_back(wall_A);
  pose.setIdentity();
  pose.translation() = (prx::vector_t(11.5 + box[0] / 2.0, 0, 11.5));
  obstacle_list.push_back(prx::create_obstacle(new prx::box_t(wall_A, 2.0, 25.0, 24.0, pose, blue)));
  const std::string wall_B{ "box_B" };
  obstacle_names.push_back(wall_B);
  pose.setIdentity();
  pose.translation() = (prx::vector_t(0, -11 - box[0] / 2.0, 11.5));
  obstacle_list.push_back(prx::create_obstacle(new prx::box_t(wall_B, 22, 2, 24.0, pose, blue)));
  const std::string wall_C{ "box_C" };
  obstacle_names.push_back(wall_C);
  pose.setIdentity();
  pose.translation() = (prx::vector_t(-11.5 - box[0] / 2.0, 0, 11.5));
  obstacle_list.push_back(prx::create_obstacle(new prx::box_t(wall_C, 2.0, 25.0, 24.0, pose, blue)));
  const std::string wall_D{ "box_D" };
  obstacle_names.push_back(wall_D);
  pose.setIdentity();
  pose.translation() = (prx::vector_t(0, 11 + box[0] / 2.0, 11.5));
  obstacle_list.push_back(prx::create_obstacle(new prx::box_t(wall_D, 22, 2, 24.0, pose, blue)));
  //
  // Create base
  const std::string base_name{ "base" };
  obstacle_names.push_back(base_name);
  pose.setIdentity();
  pose.translation() = (prx::vector_t(0.0, 0.0, -1.5));
  obstacle_list.push_back(prx::create_obstacle(new prx::box_t(base_name, 40.0, 40.0, 3.0, pose)));
}


int main(int argc, char* argv[])
{
  prx::param_loader params("executables/peg_in_hole_round.yaml", argc, argv);

  prx::simulation_step = params["simulation_step"].as<double>();
  prx::init_random(params["random_seed"].as<int>());

  // prx::PairNameObstacles obstacles{ prx::load_obstacles(params["environment"].as<>()) };
  ObstacleList obstacle_list{};
  ObstacleNames obstacle_names{};

  const std::string plant_name{ params["/plant/name"].as<>() };
  const std::string plant_path{ params["/plant/path"].as<>() };
  prx::system_ptr_t plant{ prx::system_factory_t::create_system(plant_name, plant_path) };
  prx_assert(plant != nullptr, "Plant is nullptr!");

  generate_peg(obstacle_list, obstacle_names);

  prx::world_model_t world_model({ plant }, { obstacle_list });
  world_model.create_context("context", { plant_name }, { obstacle_names });
  auto context = world_model.get_context("context");
  std::shared_ptr<prx::system_group_t> sys_group{ context.first };

  prx::space_t* ss{ sys_group->get_state_space() };
  prx::space_t* cs{ sys_group->get_control_space() };
  prx::space_t* ps{ sys_group->get_parameter_space() };

  // Visualization
  prx::three_js_group_t* vis_group = new prx::three_js_group_t({ plant }, { obstacle_list });

  std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();

  vis_group->set_floor_plane(std::vector<double>({ 0, 0, -3 }), std::vector<double>({ 0.707, 0, 0, 0.707 }),
                             std::vector<double>({ 500, 500 }), "0xbbbbbb");
  // vis_group->add_vis_infos(prx::info_geometry_t::LINE, rrt_star_query.tree_visualization, body_name, ss);
  // vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, rrt_star_query.solution_traj, body_name, ss);
  // vis_group->add_vis_infos(prx::info_geometry_t::SPHERE, { Vec(rrt_star_query.goal_state).head(3) }, "0xffff00",
  // rrt_star_query.goal_region_radius);
  // vis_group->add_animation(rrt_star_query.solution_traj, ss, rrt_star_query.start_state);
  vis_group->output_html("circular_peg.html");

  delete vis_group;

  std::cout << "End of program" << std::endl;
}
