#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/simulation/general/condition_check.hpp"
#include "prx/simulation/controllers/lqr.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"

using namespace prx;

int main(int argc, char* argv[])
{
  auto params = param_loader("examples/basic/execute_plan.yaml", argc, argv);

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

  const auto ss = context.first->get_state_space();
  const auto cs = context.first->get_control_space();
  const auto ps = plant->get_parameter_space();
  const auto ss_dim = ss->get_dimension();
  const auto cs_dim = cs->get_dimension();
  const auto ps_dim = ps->get_dimension();
  auto sg = context.first;

  auto lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
  auto upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
  ss->set_bounds(lower_bounds, upper_bounds);

  auto cs_lb = params["/plant/control_space_lower_bound"].as<std::vector<double>>();
  auto cs_up = params["/plant/control_space_upper_bound"].as<std::vector<double>>();
  cs->set_bounds(cs_lb, cs_up);

  auto start_state = ss->make_point();
  auto goal_state = ss->make_point();
  auto u_goal = cs->make_point();

  ss->copy_point_from_vector(start_state, params["/plant/start_state"].as<std::vector<double>>());
  ss->copy_point_from_vector(goal_state, params["/plant/goal_state"].as<std::vector<double>>());
  ss->copy_from_point(start_state);

  ps->copy_from_vector(params["/plant/parameters"].as<std::vector<double>>());

  trajectory_t solution_traj(ss);
  plan_t plan(cs);

  auto v_ctrl = params["/plant/constant_ctrl"].as<std::vector<double>>();

  // Eigen::Vector<double, 4> control{ v_ctrl[0], v_ctrl[1], v_ctrl[2], v_ctrl[3] };
  // plan.copy_onto_back(control, 5);
  // plan.from_file(params["data_dir"].as<>() + params["plan_to_execute"].as<>());
  std::string file_name{ "/Users/Gary/pracsys/omnibot_45_data/output/ctrl_front_0000.csg" };
  std::ifstream ifs(file_name);
  std::string line;
  char sep = ' ';
  double time, time_prev, dt;
  auto aux = cs->make_point();

  while (std::getline(ifs, line))
  {
    std::istringstream ss(line);
    std::string token;
    int i = 0;
    std::string ctrl = "";
    while (std::getline(ss, token, sep))
    {
      if (i == 0)
      {
        time_prev = time;
        time = std::stod(token);
        dt = time - time_prev;
      }
      else
        ctrl += token + sep;
      i++;
    }
    std::cout << "ctrl:" << ctrl << std::endl;
    cs->copy_point_from_string(aux, ctrl, sep);
    plan.copy_onto_back(aux, dt);
  }

  sg->propagate(start_state, plan, solution_traj);
  // solution_traj.copy_onto_back(ss);

  // solution_traj.to_file(out_path + "traj_" + plant_name + "_" + params["out_suffix"].as<>() + ".txt");

  // std::cout << "Last state: " << solution_traj.back() << " distance: " << space_t::euclidean_2d(solution_traj.back(),
  // goal_state, 0, ss_dim) << std::endl;

  three_js_group_t* vis_group = new three_js_group_t({ plant }, { obstacle_list });

  std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();

  vis_group->add_detailed_vis_infos(info_geometry_t::FULL_LINE, solution_traj, body_name, ss);

  vis_group->add_animation(solution_traj, ss, start_state);

  vis_group->output_html("execute_plan.html");

  delete vis_group;

  params.print();
}