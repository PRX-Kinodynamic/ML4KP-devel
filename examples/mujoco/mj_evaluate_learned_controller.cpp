#ifndef TORCH_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/simulation/controllers/learned_controller.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/mujoco/mj_simulator.hpp"

using namespace prx;

int main(int argc, char* argv[])
{
  auto params = param_loader("examples/roadmaps/mushr.yaml");
  init_random(params["random_seed"].as<int>());

  std::shared_ptr<prx::mujoco_simulator_t> sim =
      std::make_shared<prx::mujoco_simulator_t>(params["xml_path"].as<std::string>());
  sim->init_simulator();

  auto context = sim->get_context("mujoco");
  auto ss = context.first->get_state_space();
  for (int i = 0; i < 100; i++)
  {
    sim->step_simulation();
  }

  prx::system_ptr_t mj_plant = context.first->get_primary_system();
  auto learned_controller_params = param_loader(params["controller"].as<std::string>());
  learned_controller_t controller(mj_plant, learned_controller_params);

  dirt_specification_t dirt_spec(context.first, context.second);
  dirt_spec.sample_state = [&](space_point_t& s) {
    s->at(0) = uniform_random(-9., 9.);
    s->at(1) = uniform_random(-9., 9.);
    double roll = 0, pitch = 0, yaw = uniform_random(-PRX_PI, PRX_PI);
    Eigen::Quaterniond quat = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                              Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    s->at(2) = 0.0;
    s->at(3) = quat.w();
    s->at(4) = quat.x();
    s->at(5) = quat.y();
    s->at(6) = quat.z();
  };

  dirt_query_t dirt_query(context.first->get_state_space(), context.first->get_control_space());
  dirt_query.goal_state = context.first->get_state_space()->make_point();
  dirt_query.start_state = context.first->get_state_space()->make_point();
  mj_plant->get_state_space()->copy_to(dirt_query.start_state);
  dirt_query.goal_region_radius = params["goal_region_radius"].as<double>();
  dirt_query.goal_check = [&](space_point_t point) {
    double diff2 = (point->at(0) - dirt_query.goal_state->at(0)) * (point->at(0) - dirt_query.goal_state->at(0)) +
                   (point->at(1) - dirt_query.goal_state->at(1)) * (point->at(1) - dirt_query.goal_state->at(1));
    quaternion_t quat1 = Eigen::Quaterniond(point->at(3), point->at(4), point->at(5), point->at(6));
    quaternion_t quat2 = Eigen::Quaterniond(dirt_query.goal_state->at(3), dirt_query.goal_state->at(4),
                                            dirt_query.goal_state->at(5), dirt_query.goal_state->at(6));
    double angular_diff = quat1.angularDistance(quat2);
    diff2 += angular_diff * angular_diff;
    return std::sqrt(diff2) < dirt_query.goal_region_radius;
  };

  const unsigned num_trials = int(1e3);
  unsigned num_successes = 0;
  space_point_t final_state = context.first->get_state_space()->make_point();
  double linear_dist = 0.0;
  double angular_dist = 0.0;

  for (int i = 0; i < num_trials; i++)
  {
    dirt_spec.sample_state(dirt_query.goal_state);
    controller.fulfill_query(dirt_spec, dirt_query);
    if (dirt_query.solution_traj.size() > 0)
    {
      context.first->get_state_space()->copy_point(final_state, dirt_query.solution_traj.back());
      if (dirt_query.goal_check(final_state))
      {
        num_successes++;
        linear_dist += std::sqrt(
            (final_state->at(0) - dirt_query.goal_state->at(0)) * (final_state->at(0) - dirt_query.goal_state->at(0)) +
            (final_state->at(1) - dirt_query.goal_state->at(1)) * (final_state->at(1) - dirt_query.goal_state->at(1)));
        quaternion_t quat1 =
            Eigen::Quaterniond(final_state->at(3), final_state->at(4), final_state->at(5), final_state->at(6));
        quaternion_t quat2 = Eigen::Quaterniond(dirt_query.goal_state->at(3), dirt_query.goal_state->at(4),
                                                dirt_query.goal_state->at(5), dirt_query.goal_state->at(6));
        angular_dist += quat1.angularDistance(quat2);
      }
    }
    output_progress_bar(1.0 * i / num_trials);
  }
  std::cout << "Success rate: " << (double)num_successes / num_trials << std::endl;
  std::cout << "Average linear distance: " << linear_dist / num_successes << std::endl;
  std::cout << "Average angular distance: " << angular_dist / num_successes << std::endl;
}
#else
int main(int argc, char* argv[])
{
  std::cout << "Torch not built!" << std::endl;
}
#endif