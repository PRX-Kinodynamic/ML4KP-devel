#pragma once
#include "prx/utilities/defs.hpp"

#include "prx/factor_graphs/defs.hpp"

#include "prx/factor_graphs/utilities/fg_logger.hpp"
#include "prx/factor_graphs/planning/initialization_trajs_fg.hpp"
#include "prx/factor_graphs/factors/parameter_fusion_factor.hpp"

#include "prx/factor_graphs/utilities/utilities_functions.hpp"

namespace prx
{
namespace fg
{
using values_t = gtsam::Values;
using graph_t = gtsam::NonlinearFactorGraph;

template <Eigen::Index X_DIM, Eigen::Index U_DIM, Eigen::Index TH_DIM>
struct ilqr_parameters_t
{
  Eigen::Vector<double, X_DIM> start_state;
  Eigen::Vector<double, X_DIM> goal_state;
  Eigen::Vector<double, U_DIM> goal_control;
  Eigen::Vector<double, TH_DIM> param_vector;
  Eigen::Matrix<double, X_DIM, X_DIM> Q;
  Eigen::Matrix<double, U_DIM, U_DIM> R;
};

template <Eigen::Index X_DIM, Eigen::Index U_DIM, Eigen::Index TH_DIM>
static std::pair<graph_t, values_t> ilqr_factor_graph(const trajectory_t& traj, const plan_t& plan,
                                                      const std::shared_ptr<system_group_t> sg,
                                                      const ilqr_parameters_t<X_DIM, U_DIM, TH_DIM>& params)
{
  prx_assert(traj.size() == plan.size() + 1, "Trajectory size must be 1 greater than plan size. Got trajectory: "
                                                 << traj.size() << ", plan: " << plan.size());
  const Eigen::VectorXd start_state{ params.start_state };
  const Eigen::VectorXd goal_state{ params.goal_state };
  const Eigen::VectorXd goal_control{ params.goal_control };
  const Eigen::VectorXd param_vector{ params.param_vector };
  const Eigen::Matrix<double, X_DIM, X_DIM> Q{ params.Q };
  const Eigen::Matrix<double, U_DIM, U_DIM> R{ params.R };

  // const std::size_t total_states{ traj.size() };
  const space_t* state_space = sg->get_state_space();
  const space_t* control_space = sg->get_control_space();
  const space_t* parameter_space = sg->get_parameter_space();

  const std::size_t ss_dim{ state_space->get_dimension() };
  const std::size_t cs_dim{ control_space->get_dimension() };
  const std::size_t ps_dim{ parameter_space->get_dimension() };

  auto propagation_noise = gtsam::noiseModel::Constrained::All(ss_dim);

  auto initial_state_noise = gtsam::noiseModel::Constrained::All(ss_dim);
  auto goal_state_noise = gtsam::noiseModel::Constrained::All(ss_dim);

  auto param_prior_noise = gtsam::noiseModel::Constrained::All(ps_dim);
  auto time_prior_noise = gtsam::noiseModel::Constrained::All(1);

  auto space_limit_noise = gtsam::noiseModel::Constrained::All(ss_dim);
  auto control_limit_noise = gtsam::noiseModel::Constrained::All(cs_dim);

  auto x_cost_noise = gtsam::noiseModel::Diagonal::Sigmas(Q.diagonal());
  auto x_final_cost_noise = gtsam::noiseModel::Gaussian::Information(R);
  auto u_cost_noise = gtsam::noiseModel::Isotropic::Sigma(cs_dim, 1e0);

  graph_t ilqr_graph;
  values_t ilqr_values;

  const prx_symbol_t initial_state_symbol{ symbol_factory_t::create_symbol("state_symbol", 0) };
  ilqr_graph.addPrior(initial_state_symbol, start_state, initial_state_noise);

  std::size_t i = 0;

  for (; i < plan.size(); ++i)
  {
    const prx_symbol_t state_symbol{ symbol_factory_t::create_symbol("state_symbol", i) };
    const prx_symbol_t next_state_symbol{ symbol_factory_t::create_symbol("state_symbol", i + 1) };
    const prx_symbol_t control_symbol{ symbol_factory_t::create_symbol("control_symbol", i) };
    const prx_symbol_t time_symbol{ symbol_factory_t::create_symbol("time_symbol", i) };
    const prx_symbol_t param_symbol{ symbol_factory_t::create_symbol("param_symbol", i) };

    const Eigen::VectorXd init_state{ traj[i]->vector() };
    const Eigen::VectorXd init_control{ plan[i].control->vector() };
    const Eigen::VectorXd init_time{ (Eigen::VectorXd(1) << simulation_step).finished() };

    ilqr_values.insert_or_assign(state_symbol, init_state);
    ilqr_values.insert_or_assign(control_symbol, init_control);
    ilqr_values.insert_or_assign(time_symbol, init_time);
    ilqr_values.insert_or_assign(param_symbol, param_vector);

    ilqr_graph.add(fg::quadratic_cost_factor_t<X_DIM>(state_symbol, goal_state, Q));
    ilqr_graph.add(fg::quadratic_cost_factor_t<U_DIM>(control_symbol, goal_control, R));
    ilqr_graph.add(propagation_factor_5_t<X_DIM, U_DIM, TH_DIM>(state_symbol, next_state_symbol, control_symbol,
                                                                time_symbol, param_symbol, propagation_noise, sg));

    ilqr_graph.addPrior(time_symbol, init_time, time_prior_noise);
    ilqr_graph.addPrior(param_symbol, param_vector, param_prior_noise);

    ilqr_graph.add(space_limit_factor_t<U_DIM>(control_symbol, control_limit_noise, control_space));
    ilqr_graph.add(space_limit_factor_t<X_DIM>(state_symbol, space_limit_noise, state_space));
  }
  const prx_symbol_t final_state_symbol{ symbol_factory_t::create_symbol("state_symbol", i) };
  ilqr_graph.addPrior(final_state_symbol, goal_state, goal_state_noise);
  ilqr_values.insert_or_assign(final_state_symbol, goal_state);

  return std::make_pair(ilqr_graph, ilqr_values);
}
}  // namespace fg
}  // namespace prx