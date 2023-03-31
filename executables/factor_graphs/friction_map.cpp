#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>

#include <boost/algorithm/string/join.hpp>
#include <boost/optional.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>

#include "prx/factor_graphs/defs.hpp"
#include "prx/factor_graphs/utilities/fg_logger.hpp"
// #include "prx/factor_graphs/planning/trajectory_fg.hpp"
// #include "prx/factor_graphs/planning/trajectory_optimizer.hpp"
#include "prx/factor_graphs/utilities/utilities_functions.hpp"
#include "prx/factor_graphs/planning/initialization_trajs_fg.hpp"

#include "prx/planning/loaders/planner_loader.hpp"
#include "prx/planning/planner_statistics.hpp"
#include "prx/planning/world_model.hpp"

#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/simulation/system_factory.hpp"
#include "prx/simulation/system_group.hpp"

#include "prx/utilities/defs.hpp"

#include "prx/visualization/three_js_group.hpp"

namespace fs = std::filesystem;
using namespace prx;

int main(int argc, char* argv[])
{
  auto params = param_loader("executables/factor_graphs/friction_map.yaml", argc, argv);

  simulation_step = params["simulation_step"].as<double>();
  init_random(params["random_seed"].as<int>());

  std::string plant_name = params["/plant/name"].as<>();
  std::string plant_path = params["/plant/path"].as<>();

  auto obstacles = load_obstacles(params["environment"].as<>());

  auto obstacle_list = obstacles.second;
  auto obstacle_names = obstacles.first;

  auto plant = system_factory_t::create_system(plant_name, plant_path);
  world_model_t world_model({ plant }, {});
  world_model.create_context("context", { plant_name }, {});
  PRX_DEBUG_PRINT

  auto context = world_model.get_context("context");
  auto sg = context.first;
  const auto ss = sg->get_state_space();
  const auto cs = sg->get_control_space();
  const auto ps = sg->get_parameter_space();
  const auto ss_dim = ss->get_dimension();
  const auto cs_dim = cs->get_dimension();
  const auto ps_dim = ps->get_dimension();
  prx_assert(ps != nullptr, "Parameter space is null!!!");

  // auto lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
  // auto upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
  // ss->set_bounds(lower_bounds, upper_bounds);
  ss->set_bounds({ 0.0, 0.0, -M_PI }, { 10.0, 10.0, M_PI });
  PRX_DEBUG_PRINT

  auto cs_lb = params["/plant/control_space_lower_bound"].as<std::vector<double>>();
  auto cs_up = params["/plant/control_space_upper_bound"].as<std::vector<double>>();
  cs->set_bounds(cs_lb, cs_up);

  // std::string path = "/Users/Gary/pracsys/omnibot_45_data/output/";
  // std::unordered_map<std::string, int> files;
  // for (const auto& entry : fs::directory_iterator(path))
  // {
  //   if (files.count(entry.path().stem()) == 0)
  //   {
  //     files[entry.path().stem()] = 1;
  //     // std::cout << entry.path().stem() << std::endl;
  //   }
  //   else
  //   {
  //     files[entry.path().stem()] += 1;
  //   }
  // }
  // std::vector<std::string> plan_files;
  // std::vector<std::string> traj_files;
  std::vector<plan_t> plans{};
  std::vector<trajectory_t> trajs{};
  const int total_files = params["total_files"].as<int>();
  for (int i = 0; i < total_files; ++i)
  {
    std::string plan_file{ out_path + "friction_maps/omnirobot_mecanum_fixed_goals_plan_" + std::to_string(i) +
                           ".txt" };
    std::string traj_file{ out_path + "friction_maps/omnirobot_mecanum_fixed_goals_traj_" + std::to_string(i) +
                           ".txt" };

    plans.emplace_back(cs);
    trajs.emplace_back(ss);

    plans.back().from_file(plan_file);
    trajs.back().from_file(traj_file);
  }

  // std::ifstream ifs_plan("mecanum_analytical_plan.txt");
  // std::string line;
  // auto aux_cs = cs->make_point();
  // double dt;
  // plans.emplace_back(cs);
  // while (std::getline(ifs_plan, line))
  // {
  //   PRX_DEBUG_PRINT
  //   if (line.size() < 2)
  //   {
  //     PRX_DEBUG_PRINT
  //     plans.emplace_back(cs);
  //   }
  //   else
  //   {
  //     const char sep = ' ';
  //     std::istringstream iss(line);
  //     std::string token;
  //     int i = 0;
  //     std::string ctrl = "";
  //     while (std::getline(iss, token, sep))
  //     {
  //       if (i == 0)
  //       {
  //         // time_prev = time;
  //         dt = std::stod(token);
  //         // dt = time - time_prev;
  //       }
  //       else
  //       {
  //         (*aux_cs)[i - 1] = std::stod(token);
  //       }
  //       i++;
  //     }
  //     // std::cout << "ctrl:" << ctrl << std::endl;
  //     // cs->copy_point_from_string(aux, ctrl, sep);
  //     plans.back().copy_onto_back(aux_cs, dt);
  //   }
  // }
  // sg->propagate(start_state, plan, solution_traj);

  // std::vector<trajectory_t> trajs{};

  // trajs.emplace_back(ss);
  // std::ifstream ifs_traj("mecanum_analytical_trajs.txt");
  // auto aux_ss = ss->make_point();

  // trajs.emplace_back(ss);
  // while (std::getline(ifs_traj, line))
  // {
  //   if (line.size() < 2)
  //   {
  //     trajs.emplace_back(ss);
  //   }
  //   else
  //   {
  //     const char sep = ' ';
  //     std::istringstream iss(line);
  //     std::string token;
  //     int i = 0;
  //     std::string state = "";
  //     while (std::getline(iss, token, sep))
  //     {
  //       if (i == 0)
  //       {
  //         // time_prev = time;
  //         dt = std::stod(token);
  //         // dt = time - time_prev;
  //       }
  //       else if (i < 4)
  //       {
  //         state += token + sep;
  //       }
  //       i++;
  //     }
  //     // std::cout << "state:" << state << std::endl;
  //     ss->copy_point_from_string(aux_ss, state, sep);
  //     for (int i = 0; i < ss->get_dimension(); ++i)
  //     {
  //       (*aux_ss)[i] += uniform_random(-0.1, 0.1);
  //     }
  //     trajs.back().copy_onto_back(aux_ss);
  //   }
  // }
  // sg->propagate(start_state, plan, solution_traj);
  // return solution_traj;
  auto dm = gtsam::noiseModel::Isotropic::Sigma(ss_dim, 1);

  std::function<std::string(int, int)> gen_symbol_str = [](const int x, const int y) { return "param_symbol"; };

  const double scale{ params["scale"].as<double>() };

  std::set<std::pair<int, int>> grid;
  gtsam::NonlinearFactorGraph graph;
  int f_count = 0;
  int increment = 100;
  for (int i = 0; i < trajs.size(); ++i)
  {
    trajectory_t traj{ trajs[i] };
    plan_t plan{ plans[i] };

    increment = uniform_int_random(100, 250);
    for (unsigned i = 0; i < traj.size(); i += increment)
    {
      if (i < traj.size() - increment - 1)
      {
        const int x = scale * traj[i]->vector<>()[0];
        const int y = scale * traj[i]->vector<>()[1];
        grid.emplace(x, y);
        graph.add(propagation_factor_1_t(dm, traj[i]->vector<>(), traj[i + increment]->vector<>(),
                                         plan[i].control->vector<>(),
                                         Eigen::Vector<double, 1>{ plan[i].duration * increment },
                                         symbol_factory_t::create_symbol(gen_symbol_str(x, y), x, y), sg));
      }
    }
  }
  for (auto e : grid)
  {
    const int x{ std::get<0>(e) };
    const int y{ std::get<1>(e) };
    // std::cout << x << " " << y << std::endl;
    graph.add(space_limit_factor_t<1>(symbol_factory_t::create_symbol(gen_symbol_str(x, y), x, y),
                                      gtsam::noiseModel::Isotropic::Sigma(ps_dim, 1e0), ps));
  }

  PRX_DEBUG_PRINT
  gtsam::LevenbergMarquardtParams lm_params;
  lm_params.setVerbosityLM("SUMMARY");
  lm_params.setlambdaUpperBound(1e32);
  lm_params.setUseFixedLambdaFactor(false);
  lm_params.setDiagonalDamping(false);
  lm_params.setlambdaFactor(0.1);
  lm_params.setlambdaInitial(1e-7);
  lm_params.setMaxIterations(params["max_iterations"].as<int>());
  lm_params.setRelativeErrorTol(1e-9);
  lm_params.setAbsoluteErrorTol(1e-9);
  lm_params.setLogFile(out_path + "friction_maps/lm_params_log.txt");

  gtsam::Values init_vals;

  for (auto e : grid)
  {
    Eigen::VectorXd param_guess = Eigen::VectorXd::Ones(ps->get_dimension());  // * uniform_random(0.5, 1.5);
    const int x{ std::get<0>(e) };
    const int y{ std::get<1>(e) };
    // std::cout << x << " " << y << std::endl;

    auto symbol = prx::symbol_factory_t::create_symbol(gen_symbol_str(x, y), x, y);
    // std::cout << prx::key_formatter(symbol) << std::endl;
    init_vals.insert(symbol, param_guess);
  }

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_vals, lm_params);
  auto results = optimizer.optimize();

  std::ofstream friction_map_file;
  friction_map_file.open(out_path + "friction_maps/friction_map.txt");

  for (auto e : grid)
  {
    const int x{ std::get<0>(e) };
    const int y{ std::get<1>(e) };
    auto pi = results.at<Eigen::VectorXd>(symbol_factory_t::create_symbol(gen_symbol_str(x, y), x, y));
    friction_map_file << x / scale << " " << y / scale << " " << pi.transpose() << std::endl;
    // results.at<Eigen::VectorXd>(symbol_factory_t::create_symbol("param_symbol", 0))
  }
}