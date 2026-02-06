#ifndef TORCH_NOT_BUILT

#include <iostream>

#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/type_conversions.hpp"
#include "prx/factor_graphs/lie_groups/se3.hpp"
#include "prx/factor_graphs/factors/se3_observation.hpp"

#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/rrt_star.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/simulation/collision_checking/collision_group.hpp"

#include "prx/simulation/plants/first_order_free_body.hpp"
#include "prx/factor_graphs/factors/obstacle_factor.hpp"
#include "prx/factor_graphs/utilities/default_parameters.hpp"
#include "prx/factor_graphs/lie_groups/screw_axis.hpp"
#include "prx/factor_graphs/lie_groups/lie_integrator.hpp"
#include "prx/factor_graphs/factors/screw_smoothing.hpp"
#include "prx/factor_graphs/utilities/values_utilities.hpp"
#include "prx/factor_graphs/lie_groups/lie_ode_observation.hpp"
#include "prx/factor_graphs/lie_groups/se2.hpp"
#include "prx/factor_graphs/lie_groups/lie_integrator.hpp"
#include "prx/factor_graphs/plants/plants.hpp"

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/basis/FitBasis.h>
#include <gtsam/basis/Chebyshev2.h>

using SF = prx::fg::symbol_factory_t;
using CsvReader = prx::utilities::csv_reader_t;
using SE2 = prx::fg::SE2_t;
using Velocity = Eigen::Vector3d;
using Acceleration = Eigen::Vector3d;
using Control = Eigen::Vector2d;
using prx::utilities::convert_to;
using FG = gtsam::NonlinearFactorGraph;
using Values = gtsam::Values;
using LieIntegrator = prx::fg::lie_integration_factor_t<SE2, Velocity>;
using LieObservation = prx::fg::lie_ode_observation_factor_t<SE2, Velocity>;
using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

using BikeDynamics = prx::fg::bike_dynamics_t;
using BikeSysidFactor = prx::fg::dynamic_bicycle_sysid_factors_t;
using Params = prx::fg::bike_dynamics_t::Params;

inline gtsam::Key q_key(const std::size_t t)
{
  return SF::create_hashed_symbol("X^{", t, "}");
}
inline gtsam::Key qdot_key(const std::size_t t)
{
  return SF::create_hashed_symbol("\\dot{X}^{", t, "}");
}
inline gtsam::Key params_key()
{
  return SF::create_hashed_symbol("Params");
}

// X0, X1, U01, Params
// struct dynamic_bycicle_factors_t
//   : public gtsam::NoiseModelFactorN<bike_dynamics_t::Velocity, bike_dynamics_t::Velocity, bike_dynamics_t::Params>
// {
//   using Base = gtsam::NoiseModelFactorN<bike_dynamics_t::Velocity, bike_dynamics_t::Velocity,
//   bike_dynamics_t::Params>; using Velocity = bike_dynamics_t::Velocity; using Acceleration =
//   bike_dynamics_t::Acceleration; using Params = bike_dynamics_t::Params; using OptDeriv =
//   boost::optional<Eigen::MatrixXd&>;

//   using EulerIntegrator = prx::fg::euler_integration_factor_t<Velocity, Acceleration, double>;

//   dynamic_bycicle_factors_t(const gtsam::Key key_xdot1, const gtsam::Key key_xdot0, const gtsam::Key key_params,
//                             const NoiseModel& cost_model, const bike_dynamics_t::Control u, const double dt)
//     : Base(cost_model, key_xdot0, key_xdot1, key_params), _u(u), _dt(dt)
//   {
//   }

//   // X1 = X0 + f(x,u) dt
//   virtual Eigen::VectorXd evaluateError(const Velocity& x1, const Velocity& x0, const Params& params,
//                                         OptDeriv H1 = boost::none, OptDeriv H0 = boost::none,
//                                         OptDeriv Hparams = boost::none) const override
//   {
//     const Acceleration x0dd{ bike_dynamics_t::acceleration(x0, _u, params, _static_params, x0dd_H_x0, boost::none,
//                                                            x0dd_H_params) };
//     const Velocity x1p{ EulerIntegrator::integrate(x0, x0dd, _dt, x1p_H_x0, x1p_H_x0dd) };

//     const Velocity error{ x1p - x1 };
//     if (H1)
//     {
//       *H1 = -1.0 * Eigen::Matrix<double, 3, 3>::Ones();
//     }
//     if (H0)
//     {
//       //  err_H_x1p = I ;
//       *H0 = x1p_H_x0 + x1p_H_x0dd * x0dd_H_x0;
//     }
//     if (Hparams)
//     {
//       //  err_H_x1p = I ;
//       *Hparams = x1p_H_x0dd * x0dd_H_params;
//     }

//     return error;
//   }

//   const bike_dynamics_t::Control _u;
//   const double _dt;
// };

bool read_trajectory(CsvReader& reader, prx::plan_t& plan, prx::trajectory_t& traj, const std::string plant_name)
{
  using prx::utilities::convert_to;
  //  xi[0], xi[1], ui, dt, Gt, accel

  prx::fg::SE2_t x0;
  Eigen::VectorXd state;
  Eigen::VectorXd control;
  double dt{ 0 };

  bool trajs_eof{ true };

  // PRX_DBG_VARS(trajs_eof);
  while (reader.has_next_line())
  {
    auto line = reader.next_line();
    if (line.size() == 0)
      break;
    trajs_eof = false;

    // PRX_DBG_VARS(trajs_eof, line.size());

    const double x{ convert_to<double>(line[1]) };
    const double y{ convert_to<double>(line[2]) };
    const double th{ convert_to<double>(line[3]) };

    const double xd{ convert_to<double>(line[4]) };
    const double yd{ convert_to<double>(line[5]) };
    const double thd{ convert_to<double>(line[6]) };

    const double xdd{ convert_to<double>(line[7]) };
    const double ydd{ convert_to<double>(line[8]) };
    const double thdd{ convert_to<double>(line[9]) };

    const double u0{ convert_to<double>(line[10]) };
    const double u1{ convert_to<double>(line[11]) };

    state = Eigen::Vector<double, 6>(x, y, th, xd, yd, thd);
    // state = Eigen::Vector<double, 6>::Zero();
    control = Eigen::Vector<double, 2>(u0, u1);

    // accel.push_back(Eigen::Vector3d(xdd, ydd, thdd));
    // if (dt == 0)
    // {
    // x0 = prx::fg::SE2_t(x, y, th).inverse();
    // x0 = prx::euler_to_rotation<Eigen::Matrix3d>(Eigen::Vector<double, 1>(th), "Z");
    // x0(0, 2) = x;
    // x0(1, 2) = y;
    // PRX_DBG_VARS(x0);
    // x0 = x0.inverse().eval();
    // PRX_DBG_VARS(x0);
    // }

    // auto xp = x0 * prx::fg::SE2_t(state.head(3)).matrix();
    // auto xp = x0 * prx::fg::SE2_t(x, y, th);
    // state[0] = xp[0];
    // state[1] = xp[1];
    // state[2] = xp[2];
    // PRX_DBG_VARS(xp, state.transpose());

    dt = convert_to<double>(line[0]);

    traj.push_back(state);
    plan.copy_onto_back(control, dt);
    // PRX_DBG_VARS(dt);
  }
  // PRX_DBG_VARS(trajs_eof);

  return trajs_eof;
}

double test(std::shared_ptr<prx::system_group_t> sg, prx::plan_t& plan, prx::trajectory_t& traj_in)
{
  prx::space_t* ss{ sg->get_state_space() };
  prx::space_t* cs{ sg->get_control_space() };

  prx::trajectory_t traj{ ss };
  prx::space_point_t start_state{ ss->make_point() };
  ss->copy(start_state, traj_in.front());
  sg->propagate(start_state, plan, traj);

  double tot_err{ 0.0 };
  // for (double i{ 0.0 }; i < 1.0; i += 0.1)
  // {
  double i{ 0.5 };
  auto state_in = traj_in.at(i, false);
  auto state_out = traj.at(i, false);
  double err{ (Vec(state_in).head(2) - Vec(state_out).head(2)).norm() };
  tot_err += err;
  // }
  return tot_err;
}

int main(int argc, char* argv[])
{
  // ${ML4KP_ROS}/data/mujoco/MjMushr_fg_10trajs.txt
  prx::param_loader params{};
  params["filename"].set("${ML4KP_ROS}/data/mujoco/MjMushr_fg_10trajs.txt");
  // params["sensors"].set("");
  // params["dt"].set(0.1);
  // params["out"].set(prx::out_path + "/mushr_traj.txt");
  // params["dbg_out"].set(prx::out_path + "/mushr_smooth_dbg.txt");
  params["max_error"].set(0.1);
  params["total_trajectories"].set(1);
  params.add_opts(argc, argv);

  // std::ofstream ofs_map(params["out"].as<>().c_str());
  // std::ofstream ofs_dbg(params["dbg_out"].as<>().c_str());
  // const double dt{ params["dt"].as<double>() };

  const std::string plant_name{ "dynamic_vehicle" };
  const std::string plant_path{ "dynamic_vehicle" };

  prx::system_ptr_t plant{ prx::system_factory_t::create_system(plant_name, plant_path) };
  prx_assert(plant, "Plant is nullptr!");
  gtsam::noiseModel::Base::shared_ptr integration_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-2) };

  CsvReader reader(params["filename"].as<>(), ' ');

  prx::world_model_t world_model({ plant }, {});
  world_model.create_context("context", { plant_name }, {});
  prx::simulation_context context{ world_model.get_context("context") };

  std::shared_ptr<prx::system_group_t> sg{ prx::system_group(context) };
  std::shared_ptr<prx::collision_group_t> cg{ prx::collision_group(context) };

  prx::space_t* ss{ sg->get_state_space() };
  prx::space_t* cs{ sg->get_control_space() };

  BikeDynamics::StaticParams static_params{ BikeDynamics::init_static_params(3.5, 0.155, 0.155) };

  const gtsam::Key pK{ params_key() };
  const double dt{ 0.1 };
  prx::simulation_step = dt;
  std::size_t idx{ 0 };
  const int total_trajectories{ params["total_trajectories"].as<int>() };

  prx::space_point_t ctrl;
  prx::space_point_t state0;
  prx::space_point_t state1;

  gtsam::noiseModel::Base::shared_ptr noise_model{ gtsam::noiseModel::Isotropic::Sigma(3, 1) };
  gtsam::noiseModel::Base::shared_ptr vel_nm{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-1) };

  Params params_init{ Params::Ones() };
  std::vector<prx::plan_t> train_plans, test_plans;
  std::vector<prx::trajectory_t> train_trajs_in, test_trajs_in;

  const double train_ratio{ 0.8 };

  // while (true)
  for (int i = 0; i < total_trajectories; ++i)
  {
    prx::plan_t plan(cs);
    prx::trajectory_t traj_in(ss);

    const bool eof{ read_trajectory(reader, plan, traj_in, plant_name) };
    PRX_DBG_VARS(eof, plan.size());
    // PRX_DBG_VARS(traj_in);
    if (eof)  // No more data
    {
      i--;
      continue;
    }
    if (prx::uniform_random() < train_ratio)
    {
      train_plans.push_back(plan);
      train_trajs_in.push_back(traj_in);
    }
    else
    {
      test_plans.push_back(plan);
      test_trajs_in.push_back(traj_in);
    }
  }

  for (int i = 0; i < train_plans.size(); ++i)
  {
    gtsam::Values values;
    gtsam::NonlinearFactorGraph graph;

    double taccum{ 0.0 };

    prx::plan_t& plan{ train_plans[i] };
    prx::trajectory_t& traj_in{ train_trajs_in[i] };
    for (int i = 0; i < traj_in.size() - 1; ++i)
    {
      // PRX_DBG_VARS(i, taccum);
      ctrl = plan.at(taccum);
      state0 = traj_in.at(taccum, false);
      state1 = traj_in.at(taccum + dt, false);

      const Control u01{ Vec(ctrl) };
      gtsam::Key x0K{ qdot_key(idx) };
      gtsam::Key x1K{ qdot_key(idx + 1) };
      graph.emplace_shared<BikeSysidFactor>(x1K, x0K, pK, noise_model, u01, dt, static_params);
      Velocity v0{ Vec(state0).tail(3) };
      graph.addPrior(x0K, v0, vel_nm);
      values.insert(x0K, v0);

      idx++;
      taccum += dt;
    }
    gtsam::Key xTK{ qdot_key(idx) };
    Velocity vT{ Vec(state1).tail(3) };
    graph.addPrior(xTK, vT);
    values.insert(xTK, vT);
    idx++;

    values.insert(pK, params_init);
    // PRX_DBG_VARS(traj_in);

    gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
    lm_params.setMaxIterations(100);
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
    gtsam::Values result{ optimizer.optimize() };

    const Params param_sysid{ result.at<Params>(pK) };
    sg->get_parameter_space()->copy_from(param_sysid);
    double error{ 0 };
    for (int i = 0; i < test_trajs_in.size(); ++i)
    {
      error += test(sg, test_plans[i], test_trajs_in[i]);
    }
    PRX_DBG_VARS(error);
  }

  // std::ofstream ofs(prx::out_path + "/bike_sysid.txt");
  // for (int i = 0; i < idx; ++i)
  // {
  //   const gtsam::Key xi{ qdot_key(i) };
  //   const Velocity vin{ values.at<Velocity>(xi) };
  //   const Velocity vout{ result.at<Velocity>(xi) };

  //   ofs << vin.transpose() << " " << vout.transpose() << "\n";
  // }
  // const Params param_sysid{ result.at<Params>(pK) };
  // // PRX_DBG_VARS(param_sysid);
  // std::string sysid_parameters{ "" };
  // for (auto p : param_sysid)
  // {
  //   sysid_parameters += convert_to<std::string>(p) + ",";
  // }
  // sysid_parameters.pop_back();
  // PRX_DBG_VARS(sysid_parameters);
  // ofs.close();

  double error;
  for (int i = 0; i < test_trajs_in.size(); ++i)
  {
    error += test(sg, test_plans[i], test_trajs_in[i]);
  }
  PRX_DBG_VARS(error);

  // vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, traj, body_name, ss);
  // vis_group->add_animation(traj, ss, start_state);
  // vis_group->output_html("dynamic_bike_open_loop.html");

  return 0;
};
#else
int main(int argc, char* argv[])
{
}
#endif
