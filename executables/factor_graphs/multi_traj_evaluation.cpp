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
      observations.emplace_back(u1, u0);
      // observations.emplace_back(u0, u1);
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

int main(int argc, char* argv[])
{
  // /Users/Gary/pracsys/ML4KP-devel/resources/input_files/executables/factor_graphs/real_mushr_multi_traj.yaml
  const std::string params_file{ "executables/factor_graphs/real_mushr_multi_traj.yaml" };
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

  prx::fg::mushrTypes::Ubar::params init_params{};

  PRX_DEBUG_ITERABLE("in_params:", params["params"].as<std::vector<double>>());
  ps->copy(init_params, params["params"].as<std::vector<double>>());
  ps->copy_from(init_params);

  const std::string data_dir{ params["data_dir"].as<>() };
  std::vector<std::string> observations_in{ params["observations"].as<std::vector<std::string>>() };
  std::vector<std::string> plans_in{ params["plan"].as<std::vector<std::string>>() };

  auto transform_f = [&data_dir](const std::string& s) { return data_dir + s; };
  std::transform(observations_in.begin(), observations_in.end(),
                 observations_in.begin(),  // write to the same location
                 transform_f);
  std::transform(plans_in.begin(), plans_in.end(),
                 plans_in.begin(),  // write to the same location
                 transform_f);
  prx_assert(observations_in.size() == plans_in.size(), "observations and plans must be the same size");
  PRX_DEBUG_VAR_1(plans_in.size());
  // const std::size_t observations_to_use{ _observations.size() / 2 };

  prx::space_point_t start_state{ ss->make_point() };  // plans_in.size(), sys_group->get_state_space()->make_point());
  // prx::space_point_t start_state{ ss->make_point() };
  for (int i = 0; i < observations_in.size(); ++i)
  {
    ObservedTrajectory observations{};
    prx::plan_t plan(cs);
    prx::trajectory_t traj{ ss };
    read_observations(observations_in[i], observations);
    read_ros_plan(plans_in[i], plan);

    Vec(start_state).head(3) = observations[0].second;
    sys_group->propagate(start_state, plan, traj);

    const std::ios_base::openmode mode{ i == 0 ? std::ofstream::trunc : std::ofstream::app };
    traj.to_file(prx::out_path + "mushr/multi_traj_evals.txt", mode);
  }

  return 0;
}