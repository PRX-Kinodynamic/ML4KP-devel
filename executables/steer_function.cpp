#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/aorrt.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"

#include <fstream>

int main(int argc, char* argv[])
{
  prx::param_loader params("executables/steer_function.yaml", argc, argv);

  prx::simulation_step = params["simulation_step"].as<double>();
  prx::init_random(params["random_seed"].as<int>());

  const std::string plant_name{ params["/plant/name"].as<>() };
  const std::string plant_path{ params["/plant/path"].as<>() };
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
  prx_assert(plant != nullptr, "Plant is nullptr!");

  prx::world_model_t world_model({ plant }, {});
  world_model.create_context("context", { plant_name }, {});
  prx::world_model_context context{ world_model.get_context("context") };
  std::shared_ptr<prx::system_group_t> sys_group{ context.first };

  prx::space_t* ss{ sys_group->get_state_space() };
  prx::space_t* cs{ sys_group->get_control_space() };
  prx::space_t* ps{ sys_group->get_parameter_space() };

  const std::vector<double> ss_lower_bounds{ params["/plant/state_space/lower_bound"].as<std::vector<double>>() };
  const std::vector<double> ss_upper_bounds{ params["/plant/state_space/upper_bound"].as<std::vector<double>>() };

  const std::vector<double> cs_lower_bounds{ params["/plant/control_space/lower_bound"].as<std::vector<double>>() };
  const std::vector<double> cs_upper_bounds{ params["/plant/control_space/upper_bound"].as<std::vector<double>>() };

  const std::vector<double> ps_values{ params["/plant/parameter_space/values"].as<std::vector<double>>() };

  ss->set_bounds(ss_lower_bounds, ss_upper_bounds);
  cs->set_bounds(cs_lower_bounds, cs_upper_bounds);

  ps->copy_from(ps_values);

  prx::space_point_t start_state{ ss->make_point() };
  prx::space_point_t goal_state{ ss->make_point() };
  prx::space_point_t z{ ss->make_point() };
  prx::space_point_t control{ cs->make_point() };

  ss->copy(start_state, params["/plant/start_state"].as<std::vector<double>>());
  ss->copy(goal_state, params["/plant/goal_state"].as<std::vector<double>>());

  prx::trajectory_t solution_traj(ss);

  const double eta{ params["eta"].as<double>() };
  const double epsilon{ 0.1 };

  auto distance_function = [&](const prx::space_point_t x, const prx::space_point_t y)  // no-lint
  {
    double dist{ 0.0 };
    dist = (Vec(x).head(3) - Vec(y).head(3)).norm();
    const double q0_w{ x->at(3 + 0) };
    const double q0_x{ x->at(3 + 1) };
    const double q0_y{ x->at(3 + 2) };
    const double q0_z{ x->at(3 + 3) };
    const double q1_w{ y->at(3 + 0) };
    const double q1_x{ y->at(3 + 1) };
    const double q1_y{ y->at(3 + 2) };
    const double q1_z{ y->at(3 + 3) };
    const Eigen::Quaterniond q0(q0_w, q0_x, q0_y, q0_z);
    const Eigen::Quaterniond q1(q1_w, q1_x, q1_y, q1_z);
    // dist += q0.angularDistance(q1);
    const Eigen::Matrix3d R0{ q0.toRotationMatrix() };
    const Eigen::Matrix3d R1{ q1.toRotationMatrix() };
    dist += std::acos(((R1.transpose() * R0).trace() - 1) / 2.0);
    return dist;
  };

  auto steer = [&](prx::space_point_t z, const prx::space_point_t x, const prx::space_point_t y,
                   prx::trajectory_t& traj)  // no-lint
  {
    const double total_steps{ eta / prx::simulation_step };
    std::vector<double> steps{ prx::linspace(0.0, 1.0, total_steps) };
    for (auto ti : steps)
    {
      // printf("%d ti: %.6f\n", i, ti);
      ss->interpolate(x, y, ti, z);
      traj.copy_onto_back(z);
      ti += prx::simulation_step;
      if (eta < distance_function(x, z))
        break;
    }
    // while (distance_to_x < eta && distance_to_y >= epsilon);
  };

  steer(z, start_state, goal_state, solution_traj);

  // Visualization
  prx::three_js_group_t* vis_group = new prx::three_js_group_t({ plant }, {});

  std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();

  vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, solution_traj, body_name, ss);
  vis_group->add_animation(solution_traj, ss, start_state);
  vis_group->output_html("steer_function_output.html");

  delete vis_group;

  std::cout << "End of program" << std::endl;
}
