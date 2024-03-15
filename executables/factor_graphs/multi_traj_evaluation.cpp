#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>

#include <boost/algorithm/string/join.hpp>
#include <boost/optional.hpp>

#include "prx/simulation/plants/plants.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/simulation/system_factory.hpp"
#include "prx/simulation/system_group.hpp"
#include "prx/planning/world_model.hpp"

#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/csv_reader.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/visualization/three_js_group.hpp"

#include <gtsam/inference/Key.h>
#include <gtsam/linear/NoiseModel.h>
// #include "prx/factor_graphs/defs.hpp"
#include "prx/factor_graphs/utilities/default_parameters.hpp"
#include "prx/factor_graphs/factors/SE3.hpp"
#include "prx/factor_graphs/factors/screw_axis.hpp"
#include "prx/factor_graphs/factors/preintegration.hpp"
#include "prx/factor_graphs/factors/position_velocity_factor.hpp"
#include "prx/factor_graphs/factors/smooth_factor.hpp"
#include "prx/factor_graphs/factors/mushr_factors.hpp"
#include "prx/factor_graphs/utilities/symbols_factory.hpp"
#include "prx/factor_graphs/utilities/common_functions.hpp"
using SF = prx::fg::symbol_factory_t;

using prx::fg::mushr_ub_u_xdot_param_t;
using prx::fg::mushr_ub_u_xdot_t;
using prx::fg::mushr_x_async_observation_t;
using prx::fg::mushr_x_observation_t;
using prx::utilities::convert_to;
using namespace prx::fg::mushrTypes;

using ObservedTrajectory = std::vector<std::pair<double, Eigen::Vector3d>>;

void read_observations(const std::string& filename, ObservedTrajectory& observations)
{
  PRX_DEBUG_VAR_1(filename);
  prx::utilities::csv_reader_t reader(filename, ' ');
  while (reader.has_next_line())
  {
    auto line = reader.next_line();
    if (line.size() > 0)
    {
      const std::string frame{ line[1] };
      if (frame != "robot_0")
        continue;
      const double t{ convert_to<double>(line[2]) };
      const double x{ convert_to<double>(line[3]) };
      const double y{ convert_to<double>(line[4]) };
      const double qw{ convert_to<double>(line[6]) };
      const double qx{ convert_to<double>(line[7]) };
      const double qy{ convert_to<double>(line[8]) };
      const double qz{ convert_to<double>(line[9]) };
      const double theta{ prx::yaw(Eigen::Quaterniond(qw, qx, qy, qz)) };

      // ts.emplace_back(t);
      observations.emplace_back(std::piecewise_construct, std::forward_as_tuple(t), std::forward_as_tuple(x, y, theta));
    }
  }
  PRX_DEBUG_VAR_1(observations.size());
}

double read_ros_plan(const std::string& filename, prx::plan_t& plan)
{
  plan.clear();
  prx::utilities::csv_reader_t reader(filename, ' ');
  std::vector<double> ts;
  std::vector<Eigen::Vector2d> observations;
  while (reader.has_next_line())
  {
    auto line = reader.next_line();
    if (line.size() > 0)
    {
      const double t{ convert_to<double>(line[0]) };
      const double u0{ convert_to<double>(line[1]) };
      const double u1{ convert_to<double>(line[2]) };
      ts.emplace_back(t);
      observations.emplace_back(u0, u1);
    }
  }
  double ti{ 0.0 };
  Eigen::Vector2d ut{};
  double tprev{ ts[0] };
  for (auto tuple : prx::zip_iters(ts, observations))
  {
    std::tie(ti, ut) = prx::unzip(tuple);
    const double duration{ ti - tprev };
    plan.copy_onto_back(ut, duration);
    tprev = ti;
  }
  return ts[0];
}

// Duration of observed trajectory is higher than plan (perception runs before plan publisher / controller and after...)
// Append zeros before and after assuming the robot was not moving before/after.
void increase_plan_to_match_trajectory(const ObservedTrajectory& traj, const double plan_t0, prx::plan_t& plan,
                                       const std::size_t idx)
{
  const double traj_t0{ traj.front().first };
  const double traj_duration{ traj.back().first - traj_t0 };
  const double plan_duration{ plan.duration() };
  prx_assert(plan_duration < traj_duration, "plan duration is not less than traj duration!");

  const double init_diff{ plan_t0 - traj_t0 };
  // const double end_diff{ traj_duration - (init_diff + plan_duration) + 2 * prx::simulation_step };
  // const double end_diff{ 2 * prx::simulation_step };

  plan.copy_onto_front(Eigen::Vector2d::Zero(), init_diff);
  // plan.copy_onto_back(Eigen::Vector2d::Zero(), end_diff);
  PRX_DEBUG_VAR_3(traj_duration, plan_duration, plan.duration());

  const double new_duration{ plan.duration() };
  std::ios_base::openmode mode{ idx == 0 ? std::ofstream::trunc : std::ofstream::app };
  std::ofstream ofs(prx::out_path + "mushr/eval_input_observations.txt", mode);
  for (auto z : traj)
  {
    if (z.first - traj_t0 < new_duration)
      ofs << z.second.transpose() << "\n";
  }
  ofs << "\n";
  ofs.close();
}

void create_traj_graph(prx::trajectory_t& traj, prx::plan_t& plan, std::shared_ptr<prx::system_group_t> sys_group,
                       std::size_t idx, std::string observations_file, std::string plans_file,
                       prx::space_point_t start_state)
{
  traj.clear();
  ObservedTrajectory observations{};

  read_observations(observations_file, observations);
  const double plan_t0{ read_ros_plan(plans_file, plan) };
  increase_plan_to_match_trajectory(observations, plan_t0, plan, idx);

  Vec(start_state) = Eigen::Vector<double, 6>::Zero();
  Vec(start_state).head(3) = observations[0].second;

  sys_group->propagate(start_state, plan, traj);
  std::ios_base::openmode mode{ idx == 0 ? std::ofstream::trunc : std::ofstream::app };
  traj.to_file(prx::out_path + "mushr/result_eval_trajs.txt", mode);
}

int main(int argc, char* argv[])
{
  const std::string params_file{ "executables/factor_graphs/multi_traj_evaluation.yaml" };
  prx::param_loader params{ params_file, argc, argv };
  prx::simulation_step = 0.01;

  const std::string plant_name{ "mushrFG" };
  const std::string plant_path{ "mushrFG" };
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);

  prx::world_model_t world_model({ plant }, {});
  const std::string context_name{ "mushrFG" };
  world_model.create_context(context_name, { plant_name }, {});
  prx::world_model_context context{ world_model.get_context(context_name) };
  std::shared_ptr<prx::system_group_t> sys_group{ context.first };

  prx::space_t* ss{ sys_group->get_state_space() };
  prx::space_t* cs{ sys_group->get_control_space() };
  prx::space_t* ps{ sys_group->get_parameter_space() };

  prx::fg::mushrTypes::ParamsUbarU init_params{};

  ps->copy(init_params, params["params"].as<std::vector<double>>());
  ps->copy_from(init_params);

  const std::vector<std::string> observations_in{ params["observations"].as<std::vector<std::string>>() };
  const std::vector<std::string> plans_in{ params["plan"].as<std::vector<std::string>>() };

  std::vector<prx::space_point_t> start_states{};  // plans_in.size(), sys_group->get_state_space()->make_point());
  std::vector<prx::plan_t> plans(plans_in.size(), cs);
  prx::trajectory_t traj{ ss };
  // prx::space_point_t start_state{ ss->make_point() };

  for (int i = 0; i < plans_in.size(); ++i)
  {
    start_states.push_back(ss->make_point());
    create_traj_graph(traj, plans[i], sys_group, i, observations_in[i], plans_in[i], start_states[i]);
    PRX_DEBUG_VAR_1(start_states[i]);
  }

  return 0;
}