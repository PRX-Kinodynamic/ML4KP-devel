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

// #include "prx/factor_graphs/defs.hpp"
#include "prx/factor_graphs/utilities/symbols_factory.hpp"
// #include "prx/factor_graphs/utilities/utilities_functions.hpp"
// #include "prx/factor_graphs/factors/factors.hpp"

#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/linear/JacobianFactor.h>

// using Q = typename prx::fg::ackermann::Q;
// using Qdot = typename prx::fg::ackermann::Qdot;
// using Qdotdot = typename prx::fg::ackermann::Qdotdot;
// using U = typename prx::fg::ackermann::U;
// using Force = typename prx::fg::ackermann::Force;
// using ModelParams = typename prx::fg::ackermann::ModelParams;
// using EnvironmentParams = typename prx::fg::ackermann::EnvironmentParams;
// using Z_Q = typename prx::fg::ackermann::Qz;

// using Position = Eigen::Vector<double, 2>;
// using Path = std::vector<Position>;
// using Graph = gtsam::NonlinearFactorGraph;
// using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
// using Values = gtsam::Values;
// using Symbol = prx::prx_symbol_t;

// using SymFactory = prx::symbol_factory_t;
// using FactorPathTraj = prx::fg::euclidean_distance_factor_t<2, 2, 5>;

void find_start_state(const std::string filename, prx::space_point_t state)
{
  using prx::utilities::convert_to;
  prx::utilities::csv_reader_t reader(filename);
  while (reader.has_next_line())
  {
    auto line = reader.next_line();
    if (line.size() > 0 and line[1] == "robot_0")
    {
      state->at(0) = convert_to<double>(line[2]);
      state->at(1) = convert_to<double>(line[3]);
      const double qw{ convert_to<double>(line[5]) };
      const double qx{ convert_to<double>(line[6]) };
      const double qy{ convert_to<double>(line[7]) };
      const double qz{ convert_to<double>(line[8]) };
      const Eigen::Quaterniond quat{ qw, qx, qy, qz };
      state->at(2) = prx::quaternion_to_euler(quat)[2];
      state->at(3) = 0.0;
      break;
    }
  }
}

int main(int argc, char* argv[])
{
  const std::string params_file{ "executables/factor_graphs/mushr_sysid.yaml" };
  prx::param_loader params{ params_file, argc, argv };
  prx::simulation_step = params["simulation_step"].as<double>();

  const std::string plant_name{ "mushr" };
  const std::string plant_path{ "mushr" };
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);

  prx::world_model_t world_model({ plant }, {});
  const std::string context_name{ "mushr" };
  world_model.create_context(context_name, { plant_name }, {});
  prx::world_model_context context{ world_model.get_context(context_name) };
  std::shared_ptr<prx::system_group_t> sys_group{ context.first };

  prx::space_t* ss{ sys_group->get_state_space() };
  prx::space_t* cs{ sys_group->get_control_space() };
  prx::space_t* ps{ sys_group->get_parameter_space() };

  ps->copy_from(params["/plant/parameter_space/values"].as<std::vector<double>>());
  prx::plan_t plan(cs);
  prx::trajectory_t traj(ss);
  plan.from_file(params["plan_file"].as<>());

  prx::space_point_t x0{ ss->make_point() };
  find_start_state(params["tf_data"].as<>(), x0);
  PRX_DEBUG_VAR_1(plan);
  sys_group->propagate(x0, plan, traj);
  traj.to_file(prx::out_path + "mushr_sysid_traj.txt");
  prx::three_js_group_t* vis_group = new prx::three_js_group_t({ plant }, {});

  std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();

  vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, traj, body_name, ss);
  vis_group->add_animation(traj, ss, x0);
  vis_group->output_html("mushr_sysid.html");

  delete vis_group;
}