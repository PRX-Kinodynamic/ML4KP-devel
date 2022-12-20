#ifndef TORCH_NOT_BUILT
#include <iostream>
#include <fstream>

#include "prx/planning/condition_check.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"
#include "prx/planning/world_model.hpp"

#include "prx/simulation/controllers/lqr.hpp"
#include "prx/simulation/controllers/noisy_controller.hpp"
#include "prx/simulation/controllers/torch_controller.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/simulation/plants/types/noisy_plant.hpp"

#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/noise.hpp"
#include "prx/utilities/general/param_loader.hpp"

#include "prx/visualization/three_js_group.hpp"

using prx::condition_check_t;
using prx::controller_t;
using prx::custom_check_t;
using prx::lqr_t;
using prx::noisy_uniform_controller_t;
using prx::param_loader;
using prx::plant_t;
using prx::progress_bar_t;
using prx::simulation_step;
using prx::space_point_t;
using prx::space_t;
using prx::system_group_t;
using prx::system_ptr_t;
using prx::torch_controller_t;
using prx::uniform_noise_t;
using prx::world_model_t;
// using prx::;
// using prx::;

using uniform_noisy_plant_t = prx::noisy_plant_t<prx::uniform_noise_t>;
// using prx::;

std::size_t get_total_states(std::vector<double>& starting_lower_bound, std::vector<double>& ending_upper_bound,
                             const double& step_inc)
{
  std::size_t total_states{ 1 };
  double l, u;
  // for l,u in zip(starting_lower_bound, ending_upper_bound):
  for (auto lu : prx::zip_iters(starting_lower_bound, ending_upper_bound))
  {
    std::tie(l, u) = prx::unzip(lu);
    total_states *= 1. + std::floor((u - l) / step_inc);
  }
  return std::size_t(total_states);
}

struct time_map_t
{
  time_map_t(const param_loader& params)
    : duration(params["duration"].as<double>()), checker("sim_time", params["duration"].as<double>())
  {
    simulation_step = params["simulation_step"].as<double>();
    prx::init_random(params["random_seed"].as<int>());

    // auto obstacles = load_obstacles(params["environment"].as<>());
    // std::vector<std::shared_ptr<movable_object_t>> obstacle_list = obstacles.second;
    // std::vector<std::string> obstacle_names = obstacles.first;

    std::string plant_name = params["/plant/name"].as<>();
    std::string plant_path = params["/plant/path"].as<>();
    plant = prx::system_factory_t::create_system(plant_name, plant_path);
    prx_assert(plant != nullptr, "Plant is nullptr!");

    world_model = new world_model_t({ plant }, {});
    world_model->create_context("context", { plant_name }, {});
    auto context = world_model->get_context("context");

    sg = context.first;

    _ss = context.first->get_state_space();
    _cs = context.first->get_control_space();

    auto lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
    auto upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
    _ss->set_bounds(lower_bounds, upper_bounds);

    const double step_inc{ params["state_increment"].as<double>() };

    start_state = _ss->make_point();
    goal_state = _ss->make_point();
    u_goal = _cs->make_point();
    end_state = _ss->make_point();

    _ss->copy(start_state, params["/plant/start_state"].as<std::vector<double>>());
    _ss->copy(goal_state, params["/plant/goal_state"].as<std::vector<double>>());

    _ss->copy(start_state, lower_bounds);

    std::vector<double> noise_params;

    noise_params = params["/plant/x_noise_params"].as<std::vector<double>>();
    x0_noise = std::make_shared<uniform_noise_t>(noise_params[0], noise_params[1]);

    noise_params = params["/plant/t_noise_params"].as<std::vector<double>>();
    t_noise = std::make_shared<uniform_noise_t>(noise_params[0], noise_params[1]);

    f_noise_params = params["/plant/f_noise_params"].as<std::vector<double>>();
    ut_noise_params = params["/plant/u_t_noise_params"].as<std::vector<double>>();

    goal_check = prx::create_default_goal_check(_ss, goal_state, params["goal_region_radius"].as<double>());
    goal_checker = new condition_check_t(goal_check);
    checker.add_condition(goal_checker);
  }
  void get_noisy_controller()
  {
    controller = std::make_shared<noisy_uniform_controller_t>(controller_base, ut_noise_params[0], ut_noise_params[1]);
  }

  void get_noisy_system()
  {
    noisy_plant = std::make_shared<uniform_noisy_plant_t>(plant, f_noise_params[0], f_noise_params[1]);
  }

  double duration;

  space_t* _ss;
  space_t* _cs;
  space_point_t start_state;
  space_point_t goal_state;
  space_point_t u_goal;
  space_point_t end_state;

  std::shared_ptr<uniform_noise_t> x0_noise;
  std::shared_ptr<uniform_noise_t> t_noise;

  std::vector<double> f_noise_params;
  std::vector<double> ut_noise_params;

  std::shared_ptr<controller_t> controller_base;
  system_ptr_t plant;

  std::shared_ptr<uniform_noisy_plant_t> noisy_plant;
  std::shared_ptr<noisy_uniform_controller_t> controller;

  condition_check_t checker;
  condition_check_t* goal_checker;

  std::shared_ptr<system_group_t> sg;
  custom_check_t goal_check;

  world_model_t* world_model;
};

using time_map_function_t = std::function<void(const space_point_t&, time_map_t&)>;
std::map<std::string, time_map_function_t> time_map_functions;

time_map_function_t pendulum_lqr = [](const space_point_t& s, time_map_t& tmv)  // no-lint
{
  const space_t* ss = tmv._ss;  // no-lint
  ss->copy_point(tmv.start_state, s);
  tmv.x0_noise->add_noise(tmv.start_state);
  ss->copy_from(tmv.start_state);
  ss->enforce_bounds();

  if (tmv.noisy_plant == nullptr)
    tmv.get_noisy_system();

  if (tmv.controller == nullptr)
  {
    auto Q = Eigen::Matrix<double, 2, 2>::Identity();
    auto R = Eigen::Matrix<double, 1, 1>::Identity();
    auto lqr = std::make_shared<lqr_t>(tmv.noisy_plant, Q, R, "LQR");
    lqr->set_goal(tmv.goal_state, tmv.u_goal);
    lqr->compute_K();
    tmv.controller_base = lqr;
    tmv.get_noisy_controller();
  }
  double total_time{ tmv.duration };
  total_time = tmv.t_noise->add_noise(total_time);

  tmv.checker.set_check_value(total_time);
  tmv.checker.reset();
  tmv.sg->propagate(tmv.start_state, tmv.controller, tmv.checker, tmv.end_state);
  // return tmv.end_state
};

time_map_function_t pendulum_lc = [](const space_point_t& s, time_map_t& tmv)  // no-lint
{
  const space_t* ss = tmv._ss;  // no-lint
  ss->copy_point(tmv.start_state, s);
  tmv.x0_noise->add_noise(tmv.start_state);
  ss->copy_from(tmv.start_state);
  ss->enforce_bounds();

  if (tmv.noisy_plant == nullptr)
    tmv.get_noisy_system();

  if (tmv.controller == nullptr)
  {
    auto lc = std::make_shared<torch_controller_t>(tmv.noisy_plant, "LC", torch::kCPU, 1, 4, 0, 2,
                                                   prx::lib_path + "/examples/tripods/lc/pendulum_traced.pt");
    lc->set_goal(tmv.goal_state);
    tmv.controller_base = lc;
    tmv.get_noisy_controller();
  }

  // tmv.add_duration_noise();
  double total_time{ tmv.duration };
  // total_time = tmv.t_noise->add_noise(total_time);

  tmv.checker.set_check_value(total_time);
  tmv.checker.reset();
  tmv.sg->propagate(tmv.start_state, tmv.controller, tmv.checker, tmv.end_state);
  // return tmv.end_state
};

time_map_function_t ackermann_lc = [](const space_point_t& s, time_map_t& tmv)  // no-lint
{
  const space_t* ss = tmv._ss;
  ss->copy_point(tmv.start_state, s);
  tmv.x0_noise->add_noise(tmv.start_state);
  ss->copy_from(tmv.start_state);
  ss->enforce_bounds();

  if (tmv.noisy_plant == nullptr)
    tmv.get_noisy_system();

  if (tmv.controller == nullptr)
  {
    const long long num_predictions{ 1 };
    const long long input_size{ 6 };
    const std::size_t xi_offset{ 0 };
    const std::size_t xgi_offset{ 3 };
    const std::string network_path{ prx::lib_path + "/examples/tripods/lc/ackermann_l.pt" };
    auto lc = std::make_shared<torch_controller_t>(tmv.noisy_plant, "LC", torch::kCPU, num_predictions, input_size,
                                                   xi_offset, xgi_offset, network_path);
    lc->set_goal(tmv.goal_state);
    tmv.controller_base = lc;
    tmv.get_noisy_controller();
  }
  double total_time{ tmv.duration };

  tmv.checker.set_check_value(total_time);
  tmv.checker.reset();
  tmv.sg->propagate(tmv.start_state, tmv.controller, tmv.checker, tmv.end_state);
  // return tmv.end_state
};

time_map_function_t acrobot_lqr = [](const space_point_t& s, time_map_t& tmv)  // no-lint
{
  const space_t* ss = tmv._ss;  // no-lint
  ss->copy_point(tmv.start_state, s);
  tmv.x0_noise->add_noise(tmv.start_state);
  ss->copy_from(tmv.start_state);
  ss->enforce_bounds();

  if (tmv.noisy_plant == nullptr)
    tmv.get_noisy_system();

  if (tmv.controller == nullptr)
  {
    Eigen::Matrix<double, 4, 4> Q{ Eigen::Matrix<double, 4, 4>::Identity() };
    auto R = Eigen::Matrix<double, 1, 1>::Identity();
    Q.diagonal() = Eigen::Vector4d(10, 10, 1, 1);
    auto lqr = std::make_shared<lqr_t>(tmv.noisy_plant, Q, R, "LQR");
    lqr->set_goal(tmv.goal_state, tmv.u_goal);
    lqr->compute_K();
    tmv.controller_base = lqr;
    tmv.get_noisy_controller();
  }
  double total_time{ tmv.duration };
  total_time = tmv.t_noise->add_noise(total_time);

  tmv.checker.set_check_value(total_time);
  tmv.checker.reset();
  tmv.sg->propagate(tmv.start_state, tmv.controller, tmv.checker, tmv.end_state);
  // return tmv.end_state
};

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
  param_loader params = param_loader("examples/tripods/compute_roa.yaml", argc, argv);
  params.print();

  time_map_functions["pendulum_lqr"] = pendulum_lqr;
  time_map_functions["pendulum_lc"] = pendulum_lc;
  time_map_functions["ackermann_lc"] = ackermann_lc;
  time_map_functions["acrobot_lqr"] = acrobot_lqr;

  const std::string system_name{ params["system_name"].as<>() };
  const int num_samples{ params["num_samples"].as<int>() };

  // space_point_t state = ss.make_point();
  const double step_inc{ params["state_increment"].as<double>() };
  // auto lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
  // auto upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();

  auto lower_bounds = params["/plant/starting_lower_bound"].as<std::vector<double>>();
  auto upper_bounds = params["/plant/ending_upper_bound"].as<std::vector<double>>();

  // Get the end bounds from the total number of files it will be written.
  // One process will only do 1/total_files % of the state space
  const int total_files{ params["total_files"].as<int>() };
  const int file_number{ params["file_number"].as<int>() };
  prx_assert(file_number < total_files, "File number has to be less than total_files!");

  time_map_t tmv(params);
  std::vector<double> file_nums_vec = { static_cast<double>(total_files), static_cast<double>(file_number) };
  std::vector<double> bounds{ prx::merge_container<std::vector<double>>(tmv.f_noise_params, file_nums_vec) };
  std::size_t bounds_hash{ 0 };

  for (int i = 0; i < bounds.size(); ++i)
  {
    prx::hash_combine(bounds_hash, bounds[i]);
  }

  const std::string roa_file_name{ prx::out_path + params["out_dir"].as<>() + "/" + system_name + "_" +
                                   std::to_string(bounds_hash) + ".txt" };

  std::ofstream fout_roa(roa_file_name, std::ios::binary | std::ios::trunc);
  std::cout << "Output file:\t" << roa_file_name << std::endl;

  // for _ in tqdm(range(total_states)):
  std::stringstream line;
  auto g_tm = time_map_functions[system_name];

  std::size_t state_num{ 0 };
  const std::size_t total_states{ get_total_states(lower_bounds, upper_bounds, step_inc) };
  std::cout << "Total states:\t" << total_states << std::endl;

  const std::size_t states_per_file{ total_states / total_files };
  const std::size_t initial_state_num{ states_per_file * file_number };
  const std::size_t final_state_num{ states_per_file * (file_number + 1) };
  PRX_DEBUG_VAR_3(states_per_file, initial_state_num, final_state_num);
  progress_bar_t bar(final_state_num - initial_state_num, "");
  do
  {
    state_num++;
    if (state_num < initial_state_num)
      continue;
    if (state_num > final_state_num)
      continue;
    bar.update(state_num - initial_state_num);

    line.str(std::string());
    line << tmv.start_state;
    int reached = 0;
    for (int i = 0; i < num_samples; ++i)
    {
      g_tm(tmv.start_state, tmv);
      if (tmv.goal_check())
      {
        reached += 1;
      }
    }
    const double succesful_samples{ reached / static_cast<double>(num_samples) };
    line << succesful_samples;
    line << "\n";
    const std::string str{ line.str() };
    fout_roa.write(str.c_str(), str.size());

  } while (state_increment(tmv.start_state, step_inc, lower_bounds, upper_bounds));

  fout_roa.close();
  return 0;
}
#else
#include "prx/utilities/defs.hpp"
int main(int argc, char* argv[])
{
  PRX_NOT_IMPLEMENTED;
}
#endif
