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
// #include "prx/factor_graphs/utilities/utilities_functions.hpp"
// #include "prx/factor_graphs/factors/factors.hpp"

int main(int argc, char* argv[])
{
  const std::string params_file{ "executables/factor_graphs/mushr_sysid.yaml" };
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

  prx::plan_t plan{ cs };
  prx::trajectory_t traj{ ss };
  prx::space_point_t start_state{ ss->make_point() };

  plan.copy_onto_back(Eigen::Vector2d(0, 5), 1);
  plan.copy_onto_back(Eigen::Vector2d(0.4, 5), 1);
  plan.copy_onto_back(Eigen::Vector2d(-0.4, 5), 1);
  Vec(start_state) = Eigen::Vector<double, 6>::Zero();
  plan.to_file(prx::out_path + "mushr/plan.txt");
  sys_group->propagate(start_state, plan, traj);

  // traj.to_file(prx::out_path + "mushr/fg_fwd_prop.txt");
  const std::string out_filename{ prx::out_path + "mushr/fg_fwd_prop.txt" };
  std::ofstream ofs(out_filename, std::ofstream::trunc);

  double t{ 0 };
  for (unsigned i = 0; i < traj.size(); i += 3)
  {
    const Eigen::Vector2d pos{ Vec(traj[i]).head(2) };
    const double theta{ traj[i]->at(2) };
    const Eigen::Quaterniond q{ Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
                                Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()) };
    ofs << "r w ";
    ofs << t << " " << pos[0] << " " << pos[1] << " 0 ";
    ofs << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
    ofs << "\n";
    t += prx::simulation_step * 3;
  }

  prx::three_js_group_t* vis_group = new prx::three_js_group_t({ plant }, {});
  std::string body_name = plant_name + "/body";
  vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, traj, body_name, ss);
  vis_group->add_animation(traj, ss, start_state);
  vis_group->output_html("mushrFG.html");

  delete vis_group;
  return 0;
}