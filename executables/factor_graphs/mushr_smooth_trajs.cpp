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

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/basis/FitBasis.h>
#include <gtsam/basis/Chebyshev2.h>

using SF = prx::fg::symbol_factory_t;
using CsvReader = prx::utilities::csv_reader_t;
using SE2 = prx::fg::SE2_t;
using Velocity = Eigen::Vector3d;
using Acceleration = Eigen::Vector3d;
using prx::utilities::convert_to;
using FG = gtsam::NonlinearFactorGraph;
using Values = gtsam::Values;
using LieIntegrator = prx::fg::lie_integration_factor_t<SE2, Velocity>;
using LieObservation = prx::fg::lie_ode_observation_factor_t<SE2, Velocity>;
using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

inline gtsam::Key q_key(const std::size_t t)
{
  return SF::create_hashed_symbol("X^{", t, "}");
}
inline gtsam::Key qdot_key(const std::size_t t)
{
  return SF::create_hashed_symbol("\\dot{X}^{", t, "}");
}

bool read_file(CsvReader& reader, std::vector<double>& ts, std::vector<std::vector<double>>& vecs,
               std::vector<int> indices)
{
  using Line = CsvReader::Line<std::string>;

  bool valid{ false };
  const std::size_t dim{ indices.size() };
  while (reader.has_next_line())
  {
    const Line line{ reader.next_line() };
    if (line.size() == 0)
      break;

    valid = true;
    const double ti{ convert_to<double>(line[0]) };
    ts.emplace_back(ti);
    // PRX_DBG_VARS(line);

    vecs.emplace_back();
    std::vector<double>& vaux{ vecs.back() };
    for (int i = 0; i < dim; ++i)
    {
      const int idx{ indices[i] };
      // PRX_DBG_VARS(i, idx, line[idx]);
      vaux.emplace_back(convert_to<double>(line[idx]));
    }
    // PRX_DBG_VARS(vaux);
  }
  return valid;
}

void create_fg(FG& graph, const std::size_t tF, const double dt)
{
  PRX_DBG_VARS(tF, dt);
  gtsam::noiseModel::Base::shared_ptr integration_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-2) };
  for (int i = 0; i < tF; ++i)
  {
    const gtsam::Key q0key{ q_key(i) };
    const gtsam::Key q1key{ q_key(i + 1) };
    const gtsam::Key qdotkey{ qdot_key(i) };
    graph.emplace_shared<LieIntegrator>(q1key, q0key, qdotkey, integration_noise, dt);
  }
}

void find_t_closest(double& ti, std::size_t& idx, std::vector<double>& tplan, const double dt)
{
  while (ti > tplan[idx])
  {
    // const std::string sti{ convert_to<std::string>(ti) };
    // const std::string stplan{ convert_to<std::string>(tplan[idx]) };
    // PRX_DBG_VARS(sti, idx, stplan);
    idx++;
    // ti += dt;
  }
}

void to_file(std::ofstream& ofs_map, gtsam::Values& values, const double tot_states, const double t0,
             std::vector<double>& tplan, std::vector<std::vector<double>>& plan, const double dt)
{
  double ti{ t0 };
  std::size_t idx{ 0 };
  // PRX_DBG_VARS(convert_to<std::string>(t0));
  for (int i = 0; i < tot_states - 1; ++i)
  {
    find_t_closest(ti, idx, tplan, dt);

    // const std::string sti{ convert_to<std::string>(ti) };
    // PRX_DBG_VARS(i, idx, tot_states, sti, plan.size());

    const gtsam::Key qkey{ q_key(i) };
    const gtsam::Key qdot0key{ qdot_key(i) };
    const gtsam::Key qdot1key{ qdot_key(i + 1) };

    const SE2 q{ values.at<SE2>(qkey) };
    const Velocity qdot0{ values.at<Velocity>(qdot0key) };
    const Velocity qdot1{ values.at<Velocity>(qdot1key) };
    const Eigen::RowVector2d u(plan[idx][0], plan[idx][1]);

    const Acceleration accel{ (qdot1 - qdot0) / dt };
    ofs_map << dt << " ";                 // 0
    ofs_map << q << " ";                  // 1,2,3
    ofs_map << qdot0.transpose() << " ";  // 4,5,6
    ofs_map << accel.transpose() << " ";  // 7,8,9
    ofs_map << u << "\n";                 // 10,11

    ti += dt;
  }
  ofs_map << "\n";
}

std::pair<double, std::vector<double>> find_closest_control(const double& t_need, const std::vector<double>& tplan,
                                                            const std::vector<std::vector<double>>& plans)
{
  std::vector<double> ctrl;
  std::size_t idx{ 0 };
  double ti{ tplan[idx] };
  if (ti > t_need)
  {
    return { -1, ctrl };
  }
  while (ti < t_need)
  {
    prx_assert(idx < tplan.size(), "Searching control out of bounds");
    ti = tplan[idx];
    ctrl = plans[idx];
    idx++;
  }
  return { ti, ctrl };
}

std::vector<std::vector<double>> match_sensors_plan(const std::vector<double>& tplan,
                                                    const std::vector<std::vector<double>>& plans,
                                                    std::vector<double>& tsensors,
                                                    std::vector<std::vector<double>>& sensors, double dt)
{
  std::vector<std::vector<double>> curr_plan;

  double first{ tsensors[0] };
  auto ctrl_pair = find_closest_control(first, tplan, plans);
  while (std::fabs(ctrl_pair.first - first) > dt)
  {
    tsensors.erase(tsensors.begin());
    sensors.erase(sensors.begin());
    first = tsensors[0];
    ctrl_pair = find_closest_control(first, tplan, plans);
  }

  PRX_DBG_VARS(convert_to<std::string>(first), convert_to<std::string>(ctrl_pair.first));
  // Find the last one
  double last{ tsensors.back() };
  ctrl_pair = find_closest_control(last, tplan, plans);
  while (std::fabs(ctrl_pair.first - last) > dt)
  {
    tsensors.pop_back();
    sensors.pop_back();
    last = tsensors.back();
    ctrl_pair = find_closest_control(last, tplan, plans);
  }
  PRX_DBG_VARS(convert_to<std::string>(last), convert_to<std::string>(ctrl_pair.first));

  for (auto ti : tsensors)
  {
    ctrl_pair = find_closest_control(ti, tplan, plans);
    prx_assert(ctrl_pair.first > 0, "Control not found!");
    curr_plan.push_back(ctrl_pair.second);
  }
  prx_assert(curr_plan.size() == tsensors.size(), "Mismatch plans and sensors");
  // exit(-1);

  return curr_plan;
  // double first;
  // std::vector<double> out;
  // while (tsensors[0] < tplan[0])
  // {
  //   first = tsensors[0];
  //   tsensors.erase(tsensors.begin());
  //   sensors.erase(sensors.begin());
  // }
  // tsensors.insert(tsensors.begin(), first);
  // while (tsensors.back() > tplan.back())
  // {
  //   tsensors.pop_back();
  //   sensors.pop_back();
  // }
}

int main(int argc, char* argv[])
{
  prx::param_loader params{};
  params["plans"].set("");
  params["sensors"].set("");
  params["dt"].set(0.1);
  params["out"].set(prx::out_path + "/mushr_traj.txt");
  params["dbg_out"].set(prx::out_path + "/mushr_smooth_dbg.txt");
  params["max_error"].set(0.1);
  params.add_opts(argc, argv);

  CsvReader sensors_reader(params["sensors"].as<>());
  CsvReader plans_reader(params["plans"].as<>());
  std::ofstream ofs_map(params["out"].as<>().c_str());
  std::ofstream ofs_dbg(params["dbg_out"].as<>().c_str());
  const double dt{ params["dt"].as<double>() };
  const double max_acccepted_error{ params["max_error"].as<double>() };
  // 0                    1    2
  // 1742875049.395943000 58 world
  //   x-3        y-4      z-5      qx-6       qy-7        qz-8     qw-9
  // 0.997866 -0.533005 0.0315802 0.701439 0.000633755 0.00124616 0.712728
  //   xd-10       yd-11        zd-12      wx-13      wy-14     wz-15
  // -0.0485842 0.00537508 -0.000326119 -0.0408894 0.0325718 0.0208394
  std::vector<int> sensors_indices({ 3, 4, 6, 7, 8, 9, 10, 11, 15 });
  std::vector<int> control_indices({ 3, 4 });
  gtsam::noiseModel::Base::shared_ptr integration_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1e-2) };

  bool valid_plan{ true };
  std::vector<double> plan_times;
  std::vector<std::vector<double>> plans;
  while (valid_plan)
  {
    std::vector<double> ts({});
    std::vector<std::vector<double>> plan;
    valid_plan = read_file(plans_reader, ts, plan, control_indices);
    plan_times.insert(plan_times.end(), ts.begin(), ts.end());
    plans.insert(plans.end(), plan.begin(), plan.end());
  }
  prx_assert(plan_times.size() == plans.size(), "Plan and time does not match!");

  while (true)
  {
    std::vector<double> ts({});
    std::vector<std::vector<double>> q_qdot({});

    const bool valid_sensors{ read_file(sensors_reader, ts, q_qdot, sensors_indices) };
    // const bool valid_plans{ read_file(plans_reader, ts, plan, control_indices) };

    if (not valid_sensors)
      break;
    // std::vector<std::vector<double>> plan;

    // Remove trailing states after the plan finishes
    // match_sensors_plan(ts, plan, ts, q_qdot);
    std::vector<std::vector<double>> curr_plan{ match_sensors_plan(plan_times, plans, ts, q_qdot, dt) };

    const double T{ ts.back() - ts[0] };
    // const std::size_t tot_states{ static_cast<std::size_t>(std::ceil(T / dt)) };
    FG graph;
    Values values;
    // PRX_DBG_VARS(T, tot_states);

    const double t0{ ts[0] };
    double ti{ 0 };
    std::size_t idx{ 0 };
    std::size_t fg_idx{ 0 };
    gtsam::Key q0key{ q_key(0) };
    gtsam::Key qdotkey{ qdot_key(0) };

    const double angle0{ prx::quaternion_to_angle(q_qdot[0][2], q_qdot[0][3], q_qdot[0][4], q_qdot[0][5], 'z') };
    SE2 q{ SE2(q_qdot[0][0], q_qdot[0][1], angle0) };
    // Eigen::Vector3d qdot{ Eigen::Vector3d(q_qdot[0][6], q_qdot[0][7], q_qdot[0][8]) };
    Eigen::Vector3d qdot0{ Eigen::Vector3d::Zero() };

    values.insert(q0key, q);
    values.insert(qdotkey, qdot0);

    PRX_DBG_VARS(q_qdot.size());
    PRX_DBG_VARS(q_qdot.back());
    for (int i = 0; i < q_qdot.size(); ++i)
    {
      const std::vector<double>& v{ q_qdot[i] };
      const double curr_ti{ idx * dt };
      const double ts_0{ ts[i] - t0 };  //
      const double dt_z{ ts_0 - curr_ti };
      const double angle{ prx::quaternion_to_angle(v[2], v[3], v[4], v[5], 'z') };
      SE2 qt{ SE2(v[0], v[1], angle) };
      // const Eigen::Vector3d qdot{ v[6], v[7], v[8] };
      // const Eigen::Vector3d qdot{ prx::fg::lie_operators::right_minus(qt, q) / dt };
      const Eigen::Vector3d qdot{ Eigen::Vector3d::Zero() };

      // PRX_DBG_VARS(idx, dt_z, ts_0, curr_ti);
      ti = dt_z;
      // PRX_DBG_VARS(curr_ti, ts_0, dt_z);
      // while (ti > dt)
      if (dt_z > dt)
      {
        // PRX_DBG_VARS(ti, idx);
        ti = dt_z - dt;
        idx++;

        q0key = q_key(idx);
        qdotkey = qdot_key(idx);
        values.insert(q0key, qt);
        values.insert(qdotkey, qdot);
        ofs_dbg << qt << "\n";
        // PRINT_KEYS(q0key, qdotkey)
        // PRX_DBG_VARS(q, qt);
        // PRX_DBG_VARS(values.at<SE2>(q0key));
        // PRX_DBG_VARS(values.at<Velocity>(qdotkey).transpose());
        q = qt;
      }
      // PRX_DBG_VARS(dt, ti, idx);
      graph.emplace_shared<LieObservation>(q0key, qdotkey, nullptr, qt, ti);

      // const std::string qk{ SF::formatter(q0key) };
      // const std::string qdotk{ SF::formatter(qdotkey) };
      // PRX_DBG_VARS(qk, qdotk, qt, ti);
    }
    idx++;
    q0key = q_key(idx);
    values.insert(q0key, q);
    SF::symbols_to_file();
    // graph.print("graph", SF::formatter);

    create_fg(graph, idx, dt);

    PRX_DBG_VARS(idx, q_qdot.size());
    gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
    lm_params.setMaxIterations(100);
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
    gtsam::Values result{ optimizer.optimize() };

    const double error{ graph.error(result) };
    PRX_DBG_VARS(error);

    if (error < max_acccepted_error)
    {
      to_file(ofs_map, result, idx, ts[0], ts, curr_plan, dt);
    }

    // break;
  }

  ofs_map.close();
  ofs_dbg.close();

  return 0;
};