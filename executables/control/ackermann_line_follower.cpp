#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/condition_check.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/simulation/controllers/pid.hpp"

using namespace prx;
using Control = Eigen::Vector2d;
using Error = Eigen::Vector2d;
using State = Eigen::Vector2d;
using Gain = Eigen::Matrix2d;
using PID = prx::pid_t<Control, Error, State, Gain>;

void update_x_desired(State& x_desired, const prx::trajectory_t& traj, double& current_ti,
                      const double& distance_look_ahead)
{
  const double step{ prx::simulation_step / static_cast<double>(traj.size()) };
  double distance{ 0.0 };
  double distance_to_current{ 1000 };
  double ti{ current_ti + step };
  // State x_aux{ x_current };
  State x_ref(Vec(traj.at(current_ti)).head(2));
  const State xref_aux{ x_ref };
  while (distance < distance_look_ahead && ti < 1.0)
  {
    const State look_ahead(Vec(traj.at(ti)).head(2));
    const double new_distance_to_current{ (look_ahead - x_ref).norm() };
    distance += (x_ref - look_ahead).norm();
    x_ref = look_ahead;
    if (new_distance_to_current < distance_to_current)
    {
      distance_to_current = new_distance_to_current;
      current_ti = ti;
      x_desired = Vec(traj.at(ti)).tail(2);
    }
    // x_desired = Eigen::Vector2d(0.4, 0.7);
    // x_desired = Vec(traj.at(ti)).tail(2);
    ti += step;
  }
  PRX_DEBUG_VAR_3(current_ti, xref_aux.transpose(), traj.at(current_ti));
}

int main(int argc, char* argv[])
{
  auto params = param_loader("executables/control/ackermann_line_follower.yaml", argc, argv);

  simulation_step = params["simulation_step"].as<double>();
  init_random(params["random_seed"].as<int>());

  auto obstacles = load_obstacles(params["environment"].as<>());
  std::vector<std::shared_ptr<movable_object_t>> obstacle_list = obstacles.second;
  std::vector<std::string> obstacle_names = obstacles.first;

  std::string plant_name = params["/plant/name"].as<>();
  std::string plant_path = params["/plant/path"].as<>();
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
  prx_assert(plant != nullptr, "Plant is nullptr!");

  world_model_t world_model({ plant }, { obstacle_list });
  world_model.create_context("context", { plant_name }, { obstacle_names });
  auto context = world_model.get_context("context");
  auto sg = context.system_group;

  space_t* ss{ context.first->get_state_space() };
  space_t* cs{ context.first->get_control_space() };

  auto lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
  auto upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
  ss->set_bounds(lower_bounds, upper_bounds);

  auto start_state = ss->make_point();

  ss->copy(start_state, params["/plant/start_state"].as<std::vector<double>>());
  ss->copy_from(start_state);

  trajectory_t solution_traj(ss);
  solution_traj.copy_onto_back(ss);

  const double distance_look_ahead{ params["distance_look_ahead"].as<double>() };
  State x_current{ Eigen::Vector2d::Zero() };
  State x_desired{ Eigen::Vector2d::Zero() };
  double current_theta{ 0.0 };
  prx::trajectory_t result{ ss };

  PID::ErrorFunction error_function = [&]() {
    const Error error{ x_desired - x_current };
    // const double th_desired{ std::atan2(error[1], error[0]) };
    // const double v_error{ error.norm() - distance_look_ahead };
    // const double th_error{ norm_angle_pi(th_desired - current_theta) };
    // const double th_error{ th_desired - current_theta };
    // const double th_error{ std::atan2(std::sin(th_desired - current_theta), std::cos(th_desired - current_theta)) };
    // PRX_DEBUG_VAR_2(x_current.transpose(), current_theta);
    // PRX_DEBUG_VAR_2(x_desired.transpose(), th_desired);
    // PRX_DEBUG_VAR_1(result.back());
    // PRX_DEBUG_VAR_2(th_error, v_error);
    // PRX_DEBUG_VAR_3(x_desired.transpose(), x_current.transpose(), error.transpose());
    // return Error(th_error, v_error);
    return error;
  };

  Gain kp{ Gain::Zero() };
  Gain ki{ Gain::Zero() };
  kp.diagonal() = Eigen::Vector2d(params["kp"].as<std::vector<double>>().data());
  ki.diagonal() = Eigen::Vector2d(params["ki"].as<std::vector<double>>().data());
  std::shared_ptr<PID> pid = std::make_shared<PID>(plant, "plant", error_function, kp, ki, Gain::Zero());

  prx::trajectory_t traj_in{ ss };

  traj_in.from_file(params["trajectory_file"].as<>());

  const double duration{ params["duration"].as<double>() };
  double traj_ti{ 0.0 };
  result.copy_onto_back(ss);
  for (double ti = 0; ti < duration; ti += prx::simulation_step)
  {
    update_x_desired(x_desired, traj_in, traj_ti, distance_look_ahead);
    pid->compute_controls();
    sg->propagate_once();
    result.copy_onto_back(ss);
    x_current = Vec(result.back()).tail(2);
    // current_theta = result.back()->at(2);
  }

  three_js_group_t* vis_group = new three_js_group_t({ plant }, { obstacle_list });

  std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();

  vis_group->add_detailed_vis_infos(info_geometry_t::FULL_LINE, result, body_name, ss);

  vis_group->add_animation(result, ss, start_state);

  vis_group->output_html("ackermann_line_follower.html");

  delete vis_group;
}
