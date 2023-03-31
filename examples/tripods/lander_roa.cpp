#include <iostream>
#include <fstream>
#include "prx/utilities/defs.hpp"
#include "prx/simulation/world_model.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/simulation/general/condition_check.hpp"
#include "prx/simulation/controllers/lqr.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"

using namespace prx;

space_point_t start_state = nullptr;
space_point_t goal_state = nullptr;

bool state_increment(space_point_t pt, double step_inc, std::vector<double> lower_bounds,
                     std::vector<double> upper_bounds)
{
  for (int i = 0; i < pt->get_dim(); ++i)
  {
    pt->at(i) = pt->at(i) + step_inc;
    if (pt->at(i) <= upper_bounds[i])
    {
      return true;
    }
    pt->at(i) = lower_bounds[i];
  }
  return false;
}

int main(int argc, char* argv[])
{
  auto params = param_loader("plants/lander_LD.yaml", argc, argv);

  simulation_step = 0.01;
  init_random(231192);

  auto obstacles = load_obstacles("environments/empty.yaml");
  std::vector<std::shared_ptr<movable_object_t>> obstacle_list = obstacles.second;
  std::vector<std::string> obstacle_names = obstacles.first;
  PRX_DEBUG_PRINT

  std::string plant_name = params["name"].as<>();
  std::string plant_path = params["path"].as<>();
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
  prx_assert(plant != nullptr, "Plant is nullptr!");
  PRX_DEBUG_PRINT

  world_model_t world_model({ plant }, { obstacle_list });
  world_model.create_context("context", { plant_name }, { obstacle_names });
  auto context = world_model.get_context("context");

  auto sg = context.first;
  PRX_DEBUG_PRINT
  const auto ss = context.first->get_state_space();
  const auto cs = context.first->get_control_space();
  const auto ps = plant->get_parameter_space();

  PRX_DEBUG_PRINT
  auto lower_bounds = params["state_space_lower_bound"].as<std::vector<double>>();
  auto upper_bounds = params["state_space_upper_bound"].as<std::vector<double>>();
  ss->set_bounds(lower_bounds, upper_bounds);

  auto cs_lb = params["control_space_lower_bound"].as<std::vector<double>>();
  auto cs_up = params["control_space_upper_bound"].as<std::vector<double>>();
  cs->set_bounds(cs_lb, cs_up);

  PRX_DEBUG_PRINT
  if (ps->get_dimension() > 0)
  {
    ps->copy_from_vector(params["parameters"].as<std::vector<double>>());
    std::cout << "params: " << ps->print_memory(2) << std::endl;
  }
  PRX_DEBUG_PRINT

  start_state = ss->make_point();
  goal_state = ss->make_point();
  auto u_goal = cs->make_point();

  ss->copy_point_from_vector(start_state, params["start_state"].as<std::vector<double>>());
  ss->copy_point_from_vector(goal_state, params["goal_state"].as<std::vector<double>>());

  trajectory_t solution_traj(ss);

  // auto ltv = std::dynamic_pointer_cast<prx::ltv_t>(plant);

  int ss_dim = ss->get_dimension();
  int cs_dim = cs->get_dimension();

  // auto lqr = create_controller( plant,  params);
  controller_ptr_t ctrl = std::make_shared<lander_meditch_ctrl_t>(plant, "ctrl");
  ss->copy_from_point(start_state);
  solution_traj.copy_onto_back(ss);

  // do
  // {
  //     std::cout << "[plant] " << plant << std::endl;
  //     ctrl -> compute_controls();
  //     cs -> enforce_bounds();
  //     plant -> propagate(simulation_step);
  //     solution_traj.copy_onto_back(ss);

  // }
  // while(!checker.check()); //&& space_t::euclidean_2d(solution_traj.back(), goal_state, 0, ss_dim) > 0.01);

  // std::cout << "Last state: " << solution_traj.back() << " distance: " << space_t::euclidean_2d(solution_traj.back(),
  // goal_state, 0, ss_dim) << std::endl;

  custom_check_t custom_check = [&]() {
    // auto x1 = ss -> at(0);
    // auto x2 = ss -> at(1);
    // return std::fabs(x1) <= -1;
    return ss->at(0) <= -1;
  };

  auto lander_check = [&](space_point_t pt) {
    auto x1 = pt->at(0);
    auto x2 = pt->at(1);
    return std::fabs(x1) <= 0.1 && std::fabs(x2) <= 0.5;
  };

  condition_check_t checker("sim_time", 20);
  condition_check_t checker_2(custom_check);

  checker.add_condition(&checker_2);

  std::ofstream fout_roa;
  std::string roa_file_name = out_path + "roas/" + plant_name + ".txt";
  fout_roa.open(roa_file_name.c_str());

  auto starting_lower_bound = params["state_space_lower_bound"].as<std::vector<double>>();
  auto ending_upper_bound = params["state_space_upper_bound"].as<std::vector<double>>();

  double total_states = 1;
  double l, u;
  double step_inc = 0.2;
  for (auto b : prx::zip_iters(starting_lower_bound, ending_upper_bound))
  {
    std::tie(l, u) = prx::unzip(b);

    total_states *= 1. + std::floor((u - l) / step_inc);
  }
  std::cout << "Total states: " << total_states << std::endl;

  auto end_state = ss->make_point();
  progress_bar_t bar(total_states, "");
  space_point_t state = ss->make_point();
  ss->copy_point_from_vector(state, starting_lower_bound);
  int traj_id = 0;
  do
  {
    checker.reset();
    sg->propagate(state, ctrl, checker, end_state);
    // std::cout << "Memory: " << ss -> print_memory(3) << std::endl;
    // std::cout << "state: " << state << std::endl;
    // std::cout << "end_state: " << end_state << std::endl;
    fout_roa << std::setprecision(4) << std::fixed << state << " ";
    fout_roa << (lander_check(end_state) ? 1 : 0) << " ";
    fout_roa << std::setprecision(4) << std::fixed << end_state << " ";
    fout_roa << "\n";

    bar.update(traj_id);
    traj_id++;
  } while (state_increment(state, step_inc, starting_lower_bound, ending_upper_bound));

  three_js_group_t* vis_group = new three_js_group_t({ plant }, { obstacle_list });

  delete vis_group;

  params.print();
}