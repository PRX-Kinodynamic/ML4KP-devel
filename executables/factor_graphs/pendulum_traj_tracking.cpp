#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/aorrt.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/utilities/general/noise.hpp"

#include "prx/factor_graphs/defs.hpp"
#include "prx/factor_graphs/utilities/fg_logger.hpp"
#include "prx/factor_graphs/factors/friction_fusion_factor.hpp"
#include "prx/factor_graphs/factors/parameter_fusion_factor.hpp"
#include "prx/factor_graphs/factors/quadratic_cost_factor.hpp"

#include "prx/factor_graphs/utilities/utilities_functions.hpp"
#include "prx/factor_graphs/utilities/formatter.hpp"

#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/linear/GaussianEliminationTree.h>

#include <fstream>

using namespace prx;

const std::size_t X_DIM{ 2 };
const std::size_t U_DIM{ 1 };

using X = Eigen::Vector<double, X_DIM>;
using U = Eigen::Vector<double, U_DIM>;

using Q = Eigen::Matrix<double, X_DIM, X_DIM>;
using R = Eigen::Matrix<double, U_DIM, U_DIM>;

gtsam::LevenbergMarquardtParams fg_params()
{
  gtsam::LevenbergMarquardtParams lm_params;
  lm_params.setVerbosityLM("SUMMARY");
  lm_params.setlambdaUpperBound(1e32);
  lm_params.setUseFixedLambdaFactor(false);
  lm_params.setDiagonalDamping(true);
  lm_params.setlambdaFactor(2);
  lm_params.setlambdaInitial(1e-7);
  lm_params.setMaxIterations(10);
  lm_params.setRelativeErrorTol(1e-6);
  lm_params.setAbsoluteErrorTol(1e-6);
  return lm_params;
}

// using BlockOrdering = std::vector<gtsam::Ordering>;
std::shared_ptr<gtsam::GaussianBayesNet> BlockEliminateSequential(gtsam::GaussianFactorGraph graph,
                                                                  const std::vector<gtsam::Ordering>& ordering)
{
  // setup
  gtsam::VariableIndex variableIndex(graph);  // maps keys to factor indices
  auto bn = std::make_shared<gtsam::GaussianBayesNet>();

  // loop
  for (auto keys : ordering)
  {
    // collect factors
    gtsam::GaussianFactorGraph factors;
    for (auto key : keys)
      for (size_t factorindex : variableIndex[key])
      {
        factors.push_back(graph.at(factorindex));
        graph.remove(factorindex);
      }
    // eliminate
    auto [conditional, newfactor] =
        gtsam::EliminationTraits<gtsam::GaussianFactorGraph>::DefaultEliminate(factors, keys);
    bn->push_back(conditional);
    // add new joint factor
    graph.push_back(newfactor);
    variableIndex.augment(gtsam::GaussianFactorGraph(newfactor));
  }
  return bn;
}
int main(int argc, char* argv[])
{
  auto params = param_loader("examples/tripods/pendulum_traj_tracking.yaml", argc, argv);

  simulation_step = params["simulation_step"].as<double>();
  prx::precision = 10;
  init_random(params["random_seed"].as<int>());

  auto obstacles = load_obstacles(params["environment"].as<>());
  std::vector<std::shared_ptr<movable_object_t>> obstacle_list = obstacles.second;
  std::vector<std::string> obstacle_names = obstacles.first;

  std::string plant_name = params["/plant/name"].as<>();
  std::string plant_path = params["/plant/path"].as<>();
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
  prx_assert(plant != nullptr, "Plant is nullptr!");

  world_model_t world_model({ plant }, { obstacle_list });
  world_model.create_context("aorrt_context", { plant_name }, { obstacle_names });
  auto context = world_model.get_context("aorrt_context");

  aorrt_t aorrt("AORRT");
  std::shared_ptr<system_group_t> sg = world_model_t::get_system_group(context);
  std::shared_ptr<collision_group_t> cg = world_model_t::get_collision_group(context);
  aorrt_specification_t aorrt_spec(sg, cg);

  // rrt_spec.valid_state = [](space_point_t& s)
  // {
  // Custom valid_state can be added here.
  // };

  // rrt_spec.valid_check = [&rrt_spec](trajectory_t& traj)
  // {
  // Custom valid_check goes here...
  // Basically for x in traj, call valid_state
  // };

  // Two ways of accessing lengthy parameter paths
  int min_steps = params["plant"]["min_steps"].as<int>();
  int max_steps = params["/plant/max_steps"].as<int>();

  // rrt_spec.sample_plan = [&](plan_t& plan, space_point_t pose)
  // {
  // Add custom sample plan here
  // };

  aorrt_spec.min_control_steps = min_steps;
  aorrt_spec.max_control_steps = max_steps;

  auto ss = context.first->get_state_space();
  auto cs = context.first->get_control_space();

  aorrt_query_t aorrt_query(ss, cs);
  aorrt_query.start_state = ss->make_point();
  aorrt_query.goal_state = ss->make_point();

  auto lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
  auto upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
  ss->set_bounds(lower_bounds, upper_bounds);

  ss->copy_point_from_vector(aorrt_query.start_state, params["/plant/start_state"].as<std::vector<double>>());
  ss->copy_point_from_vector(aorrt_query.goal_state, params["/plant/goal_state"].as<std::vector<double>>());

  aorrt_query.goal_region_radius = params["goal_region_radius"].as<double>();

  // Alternatively, change the goal_check function
  // rrt_query.goal_check = [&](space_point_t pt)
  // {
  //    // Default is:
  // return space_t::euclidean_2d(pt, rrt_query.goal_state) < goal_region_radius;
  // }

  aorrt_query.get_visualization = params["visualize"].as<bool>();

  aorrt.link_and_setup_spec(&aorrt_spec);
  aorrt.preprocess();
  aorrt.link_and_setup_query(&aorrt_query);

  condition_check_t checker(params["checker_type"].as<>(), params["checker_value"].as<int>());

  std::cout << "Running AORRT for " << params["checker_value"].as<int>() << " (" << params["checker_type"].as<>() << ")"
            << std::endl;

  aorrt.resolve_query(&checker);
  aorrt.fulfill_query();

  aorrt.get_tree().to_file(out_path + "aorrt_tree.txt");
  params.print();

  std::cout << "Solution traj: " << aorrt_query.solution_traj << std::endl;
  std::cout << "Solution plan: " << aorrt_query.solution_plan << std::endl;

  auto dm = gtsam::noiseModel::Isotropic::Sigma(X_DIM, 1e-0);
  auto x_nm = gtsam::noiseModel::Isotropic::Sigma(X_DIM, 1e-3);
  auto u_nm = gtsam::noiseModel::Isotropic::Sigma(U_DIM, 1e0);

  gtsam::Values ilqr_values;
  gtsam::NonlinearFactorGraph ilqr_graph;
  std::vector<gtsam::Ordering> ilqr_ordering;

  const std::size_t tot_ctrls{ aorrt_query.solution_plan.size() };

  double duration_so_far{ 0 };
  const std::size_t steps{ static_cast<std::size_t>(1 / simulation_step) };
  std::vector<std::pair<X, X>> local_goals_ks;
  PRX_DEBUG_VAR_1(steps);

  const std::string nominal_traj_filename{ prx::out_path + "pend_nominal_traj_" +
                                           std::to_string(aorrt_query.start_state->at(0)) +
                                           std::to_string(aorrt_query.start_state->at(1)) + ".txt" };
  const std::string nominal_plan_filename{ prx::out_path + "pend_nominal_plan_" +
                                           std::to_string(aorrt_query.start_state->at(0)) +
                                           std::to_string(aorrt_query.start_state->at(1)) + ".txt" };
  aorrt_query.solution_traj.to_file(nominal_traj_filename);
  aorrt_query.solution_plan.to_file(nominal_plan_filename);

  PRX_DEBUG_VAR_1(nominal_traj_filename);
  PRX_DEBUG_VAR_1(nominal_plan_filename);
  for (unsigned i = 0; i < tot_ctrls; ++i)
  {
    // const auto xi{ aorrt_query.solution_traj[i] };
    // const auto ui{ aorrt_query.solution_plan[i] };

    const U ui{ aorrt_query.solution_plan[i].control->vector<U>() };
    const double tau_i{ aorrt_query.solution_plan[i].duration };

    const std::size_t traj_idx{ static_cast<std::size_t>(duration_so_far * steps) };
    PRX_DEBUG_VAR_1(traj_idx);
    const X xi{ aorrt_query.solution_traj[traj_idx]->vector<X>() };
    prx::prx_symbol_t state_symbol = symbol_factory_t::create_symbol("state_symbol", i);
    prx::prx_symbol_t next_state_symbol = symbol_factory_t::create_symbol("state_symbol", i + 1);
    prx::prx_symbol_t control_symbol = symbol_factory_t::create_symbol("control_symbol", i);
    local_goals_ks.push_back(std::make_pair(xi, X::Zero()));

    ilqr_ordering.emplace(ilqr_ordering.begin());
    ilqr_ordering.begin()->push_back(state_symbol);
    ilqr_ordering.emplace(ilqr_ordering.begin());
    ilqr_ordering.begin()->push_back(control_symbol);

    ilqr_values.insert(state_symbol, xi);
    ilqr_values.insert(control_symbol, ui);

    ilqr_graph.addPrior(state_symbol, xi, x_nm);
    ilqr_graph.addPrior(control_symbol, ui, u_nm);

    // ilqr_graph.add(propagation_factor_XU_t<space_point_t, std::shared_ptr<plan_t>>(state_symbol, next_state_symbol,
    // control_symbol, dm, sg));
    ilqr_graph.add(
        propagation_factor_XU_t<X_DIM, U_DIM>(state_symbol, next_state_symbol, control_symbol, tau_i, dm, sg));

    duration_so_far += tau_i;
    ilqr_graph.add(fg::quadratic_cost_factor_t<X_DIM>(state_symbol, xi, Q::Ones(), 1e0));
    ilqr_graph.add(fg::quadratic_cost_factor_t<U_DIM>(control_symbol, ui, R::Ones(), 1e0));
  }
  auto test_res = ss->make_point();

  PRX_DEBUG_VAR_1(aorrt_query.solution_traj.size());
  PRX_DEBUG_VAR_1(aorrt_query.solution_plan.duration());
  PRX_DEBUG_VAR_1(aorrt_query.solution_plan.size());
  sg->propagate(aorrt_query.start_state, aorrt_query.solution_plan, test_res);
  PRX_DEBUG_VAR_1(test_res);

  const std::size_t traj_idx{ static_cast<std::size_t>(duration_so_far * steps) };
  const X xT{ aorrt_query.solution_traj[traj_idx]->vector<X>() };
  local_goals_ks.push_back(std::make_pair(xT, X::Zero()));

  prx::prx_symbol_t state_symbol = symbol_factory_t::create_symbol("state_symbol", tot_ctrls);
  ilqr_ordering.emplace(ilqr_ordering.begin());
  ilqr_ordering.begin()->push_back(state_symbol);
  ilqr_graph.addPrior(state_symbol, xT, x_nm);
  ilqr_values.insert(state_symbol, xT);

  for (auto ordering : ilqr_ordering)
  {
    for (auto k : ordering)
    {
      PRX_DEBUG_VAR_1(prx::key_formatter(k));
    }
  }
  fg_logger_t friction_map_logger("pendulum_traj_tracking.log", ' ', "-");

  gtsam::LevenbergMarquardtParams lm_params{ fg_params() };
  gtsam::LevenbergMarquardtOptimizer optimizer(ilqr_graph, ilqr_values, lm_params);
  gtsam::Values results = fg::utilities::optimize_and_log(optimizer, lm_params, friction_map_logger, 0);

  auto linearized_fg = ilqr_graph.linearize(results);
  // gtsam::GaussianEliminationTree ilqr_elimination_tree(*linearized_fg, ilqr_ordering);
  auto ilqr_elimination_tree = BlockEliminateSequential(*linearized_fg, ilqr_ordering);

  std::size_t ci{ 0 };
  for (auto c : *ilqr_elimination_tree)
  {
    PRX_DEBUG_VAR_1(ci);
    PRX_DEBUG_VAR_1(c->R());
    PRX_DEBUG_VAR_1(c->S());
    PRX_DEBUG_VAR_1(c->getA());
    PRX_DEBUG_VAR_1(c->nrFrontals());

    c->print("cond: ", prx::key_formatter);
    // auto x = c->R().triangularView<Eigen::Upper>().solve(c->S());
    // PRX_DEBUG_VAR_1(x);
    ci++;

    //   // c->print("ilqr conditional", prx::key_formatter);
  }

  // remove the initial state as there is no K for it
  local_goals_ks.erase(local_goals_ks.begin());
  std::cout << "-~-~-~-~-~-~-~-~\n";
  auto ilqr_ptr = ilqr_elimination_tree->end();
  for (std::size_t i = 0; i < tot_ctrls;)
  {
    ilqr_ptr--;
    // PRX_DEBUG_VAR_1(i);
    auto cond = *ilqr_ptr;
    prx::prx_symbol_t control_symbol = symbol_factory_t::create_symbol("control_symbol", i);
    if (cond->nrFrontals() == 1 && cond->firstFrontalKey() == control_symbol)
    {
      PRX_DEBUG_VAR_1(prx::key_formatter(cond->firstFrontalKey()));
      cond->print("cond: ", prx::key_formatter);
      auto R_inv = cond->R().inverse();
      auto Ti = cond->S();
      X ki = (R_inv * Ti).transpose();
      PRX_DEBUG_VAR_1(ki);
      local_goals_ks[i].second = ki;

      i++;
    }
  }
  // std::cout << "local_goals_ks:\n";
  const std::string local_goals_ks_filename{ prx::out_path + "local_goals_ks_" +
                                             std::to_string(aorrt_query.start_state->at(0)) +
                                             std::to_string(aorrt_query.start_state->at(1)) + ".txt" };
  logger_t local_goals_ks_log(local_goals_ks_filename);

  for (auto p : local_goals_ks)
  {
    local_goals_ks_log.log(p.first.transpose(), p.second.transpose());
    // std::cout << p.first.transpose() << "\t";
    // std::cout << p.second.transpose() << "\n";
  }

  auto state_ks = ss->make_point();
  auto control_ks = cs->make_point();
  // ss->copy(state_ks, aorrt_query.start_state->vector() + X(0.05, 0.05));
  // PRX_DEBUG_VAR_1(state_ks);

  trajectory_t traj_i(ss);
  std::function<void()> compute_traj = [&]() {
    std::size_t u_idx{ 0 };
    std::size_t curr_k{ 0 };
    double segment_duration{ 0.0 };
    traj_i.copy_onto_back(state_ks);
    for (std::size_t x_idx = 0; x_idx < aorrt_query.solution_traj.size() - 1; x_idx++)
    {
      const X x_traj{ aorrt_query.solution_traj[x_idx]->vector() };
      const U u_plan{ aorrt_query.solution_plan[u_idx].control->vector() };
      const double u_dur{ aorrt_query.solution_plan[u_idx].duration };

      const X ki{ local_goals_ks[curr_k].second };
      const U du{ -ki.transpose() * (state_ks->vector() - x_traj) };
      // const U du{ U::Zero() };
      cs->copy(control_ks, u_plan + du);
      sg->propagate_once(state_ks, control_ks, state_ks);

      segment_duration += simulation_step;
      // Can't do segment_duration
      if (std::fabs(segment_duration - u_dur) < std::pow(simulation_step, 2))
      {
        segment_duration = 0.0;
        u_idx++;
        curr_k++;
      }
      traj_i.copy_onto_back(ss);
    }
    // traj_i.to_file(tracking_trajs);
  };

  // uniform_noise_t noise(-.5, .5);
  uniform_noise_t noise(-.25, .25);

  ss->copy(state_ks, aorrt_query.start_state->vector());
  compute_traj();

  const std::string gt_tracking_trajs(prx::out_path + "gt_tracking_trajs.txt");
  const std::string fail_tracking_trajs(prx::out_path + "fail_tracking_trajs.txt");
  const std::string succ_tracking_trajs(prx::out_path + "succ_tracking_trajs.txt");

  traj_i.to_file(gt_tracking_trajs);
  traj_i.clear();
  traj_i.to_file(fail_tracking_trajs, std::ofstream::trunc);
  traj_i.to_file(succ_tracking_trajs, std::ofstream::trunc);

  const std::string ss_es_filename{ prx::out_path + "/pend_tracking_ss_es.txt" };
  const std::string fail_trajs_filename{ prx::out_path + "/pend_tracking_fail_trajs.txt" };
  const std::string succ_trajs_filename{ prx::out_path + "/pend_tracking_succ_trajs.txt" };
  logger_t ss_es_log(ss_es_filename);

  std::ios_base::openmode tracking_trajs_openmode = std::ofstream::trunc;

  for (int i = 0; i < 100000; ++i)
  {
    traj_i.clear();
    ss->copy(state_ks, aorrt_query.start_state->vector());
    noise.add_noise(state_ks);
    ss_es_log.add_values(*state_ks, " ");
    // PRX_DEBUG_VAR_1(state_ks);
    compute_traj();
    ss_es_log.add_values(*state_ks, " ");
    if (space_t::euclidean_2d(aorrt_query.goal_state, state_ks) < 0.1)
    {
      ss_es_log.log(1);
      traj_i.to_file(succ_trajs_filename, tracking_trajs_openmode);
    }
    else
    {
      ss_es_log.log(0);
      traj_i.to_file(fail_trajs_filename, tracking_trajs_openmode);
    }
    // PRX_DEBUG_VAR_1(state_ks);
    tracking_trajs_openmode = std::ofstream::app;
  }

  three_js_group_t* vis_group = new three_js_group_t({ plant }, { obstacle_list });

  std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();

  vis_group->add_vis_infos(info_geometry_t::LINE, aorrt_query.tree_visualization, body_name, ss);

  vis_group->add_detailed_vis_infos(info_geometry_t::FULL_LINE, aorrt_query.solution_traj, body_name, ss);

  vis_group->add_animation(aorrt_query.solution_traj, ss, aorrt_query.start_state);

  vis_group->output_html("output.html");

  delete vis_group;

  std::cout << "End of program" << std::endl;
}
