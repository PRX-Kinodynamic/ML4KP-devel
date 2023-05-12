#define BOOST_AUTO_TEST_MAIN mj_simulator_test
#include <string>
#include <boost/test/unit_test.hpp>

#include "prx/simulation/controllers/bang_bang.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/simulation/playback/plan.hpp"
#include "prx/mujoco/mj_simulator.hpp"

const double TEST_TOLERANCE{ 0.001 };

BOOST_AUTO_TEST_CASE(same_propagations_result_in_same_state)
{
  const std::string mj_filename{ "ball.xml" };

  std::shared_ptr<prx::mujoco_simulator_t> sim = std::make_shared<prx::mujoco_simulator_t>(mj_filename, false);
  sim->init_simulator();

  auto context = sim->get_context("mujoco");
  prx::space_t* ss = context.first->get_state_space();
  prx::space_t* cs = context.first->get_control_space();
  prx::space_point_t start = ss->make_point();

  ss->copy_to(start);
  prx::plan_t plan(cs);
  Eigen::Vector3d ctrl{ 0.1, 0.0, 0.0 };
  const double duration{ 1.0 };
  plan.copy_onto_back(ctrl, duration);
  prx::trajectory_t traj(ss);

  Eigen::VectorXd end_state_0{ Eigen::VectorXd::Zero(ss->get_dimension()) };
  Eigen::VectorXd end_state_1{ Eigen::VectorXd::Zero(ss->get_dimension()) };

  context.first->propagate(start, plan, traj);

  PRX_DEBUG_VAR_1(traj);
  const std::size_t traj0_size{ traj.size() };
  ss->copy(end_state_0, traj.back());

  sim->reset_simulation();
  context.first->propagate(start, plan, traj);
  ss->copy(end_state_1, traj.back());
  PRX_DEBUG_VAR_1(traj);
  const std::size_t traj1_size{ traj.size() };

  BOOST_CHECK(traj0_size == traj1_size);
  BOOST_CHECK_SMALL((end_state_1 - end_state_0).norm(), TEST_TOLERANCE);
}