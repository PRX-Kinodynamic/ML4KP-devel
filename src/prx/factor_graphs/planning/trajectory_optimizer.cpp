#include "prx/factor_graphs/planning/trajectory_optimizer.hpp"

namespace prx
{
void trajectory_optimizer::sample_trajectory()
{
  traj_internal.clear();

  traj_internal.copy_onto_back(traj_original.front());
  for (unsigned i = 1; i < traj_original.size() - 1; ++i)
  {
    if (uniform_random(0, 1) > original_rate)
      continue;
    traj_internal.copy_onto_back(traj_original[i]);
  }
  traj_internal.copy_onto_back(traj_original.back());
}

// void trajectory_optimizer::sample_trajectory_and_plan()
// {
// 	traj_internal.clear();
// 	plan.clear();

// 	traj_internal.copy_onto_back(traj_original.front());
// 	plan_input
// 	for (unsigned i = 1; i < traj_original.size()-1; ++i)
// 	{
// 		if (uniform_random(0,1) > original_rate) continue;
// 		traj_internal.copy_onto_back(traj_original[i]);
// 	}
// 	traj_internal.copy_onto_back(traj_original.back());
// }

void trajectory_optimizer::init_plan()
{
  plan.clear();
  int num_steps = traj_internal.size() - 1;
  auto cs = sys_group->get_control_space();
  auto cs_dim = cs->get_dimension();
  for (int i = 0; i < num_steps; ++i)
  {
    plan.copy_onto_back(Eigen::VectorXd::Zero(cs_dim), simulation_step);
  }
}

void trajectory_optimizer::optimize(trajectory_t& traj_out, plan_t& plan_out)
{
  auto start = std::chrono::steady_clock::now();

  auto ss = sys_group->get_state_space();
  auto cs = sys_group->get_control_space();
  auto cs_dim = cs->get_dimension();

  trajectory_fg_t tfg(plant);

  std::vector<double> start_v;
  std::vector<double> goal_v;

  gtsam::LevenbergMarquardtParams lm_params;
  lm_params.setVerbosityLM("SUMMARY");
  lm_params.setlambdaUpperBound(1e32);
  lm_params.setUseFixedLambdaFactor(false);
  lm_params.setDiagonalDamping(false);
  lm_params.setlambdaFactor(2);
  lm_params.setlambdaInitial(1e-6);
  lm_params.setMaxIterations(100);
  lm_params.setRelativeErrorTol(1e-7);
  lm_params.setAbsoluteErrorTol(1e-7);

  ss->copy_vector_from_point(start_v, traj_original.front());
  ss->copy_vector_from_point(goal_v, traj_original.back());

  tfg.set_initial_state(start_v);
  tfg.set_goal_state(goal_v);

  trajectory_fg_params_t fg_params;
  fg_params.limits_factors = true;
  fg_params.goal_state_as_prior = true;
  fg_params.propagation_factors_type = 3;
  fg_params.use_goal_factors = false;
  fg_params.initial_state_as_prior = true;

  int total_iters = 0;
  original_rate = traj_opt_params.traj_rate;
  double original_rate_prev = original_rate;
  gtsam::Values init_vals;

  init_vals.clear();
  sample_trajectory();
  init_plan();
  fg_params.num_steps = traj_internal.size() - 1;

  auto graph = tfg.get_smoothing_fg(fg_params);

  init_vals = initialization_trajs_fg_t::init_from_traj(plant, traj_internal, plan);

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_vals, lm_params);
  auto results = fg_utilities::optimize_and_log(optimizer, lm_params, logger, total_iters);
  total_iters += optimizer.iterations();

  fg_utilities::values_to_plan_and_traj(results, &traj_out, &plan_out, traj_internal.size() - 1);

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";
}

}  // namespace prx
