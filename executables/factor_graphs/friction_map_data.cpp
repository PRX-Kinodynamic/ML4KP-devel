#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/simulation/general/condition_check.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"

using namespace prx;

int main(int argc, char* argv[])
{
  auto params = param_loader("executables/factor_graphs/friction_map.yaml", argc, argv);

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
  world_model.create_context("context", { plant_name }, { obstacle_names });
  auto context = world_model.get_context("context");

  const auto ss = context.first->get_state_space();
  const auto cs = context.first->get_control_space();
  const auto ps = context.first->get_parameter_space();

  // auto lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
  // auto upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
  ss->set_bounds({ 0.0, 0.0, -M_PI }, { 10.0, 10.0, M_PI });

  auto start_state = ss->make_point();

  ss->copy_point_from_vector(start_state, params["/plant/start_state"].as<std::vector<double>>());
  ss->copy_from_point(start_state);

  trajectory_t solution_traj(ss);
  solution_traj.copy_onto_back(ss);

  condition_check_t checker(params["checker_type"].as<>(), params["checker_value"].as<int>());

  std::ofstream ofs_trajs, ofs_frmap, ofs_plans;
  ofs_trajs.open("mecanum_analytical_trajs.txt", std::ofstream::trunc);
  ofs_plans.open("mecanum_analytical_plan.txt", std::ofstream::trunc);
  ofs_frmap.open("mecanum_analytical_friction_map.txt", std::ofstream::trunc);
  auto friction_map_gt = [&]() {
    const double x{ ss->at(0) };
    const double y{ ss->at(1) };
    Eigen::Vector4d friction_params{ 0.25 + x / 10.0, 0.5 + x / 10.0, 0.25 + y / 10.0, 0.5 + y / 10.0 };
    ps->copy_from_vector(friction_params);
    ofs_frmap << x << " " << y << " " << friction_params.transpose() << std::endl;
  };

  const int num_trajs{ 10 };
  auto cs_pt = cs->make_point();
  auto ss_pt = ss->make_point();
  for (int i = 0; i < num_trajs; ++i)
  {
    cs->sample(cs_pt);
    ss->sample(ss_pt);

    cs->copy_from_point(cs_pt);
    ss->copy_from_point(ss_pt);
    checker.reset();
    do
    {
      friction_map_gt();
      plant->propagate(simulation_step);

      ofs_trajs << simulation_step << " " << ss->print_memory() << "\n";
      ofs_plans << simulation_step << " " << cs_pt << "\n";
      // solution_traj.copy_onto_back(ss);
    } while (!checker.check());
    ofs_trajs << "\n";
    ofs_plans << "\n";
  }

  three_js_group_t* vis_group = new three_js_group_t({ plant }, { obstacle_list });

  std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();

  vis_group->add_detailed_vis_infos(info_geometry_t::FULL_LINE, solution_traj, body_name, ss);

  vis_group->add_animation(solution_traj, ss, start_state);

  vis_group->output_html("no_control.html");

  delete vis_group;

  params.print();
}