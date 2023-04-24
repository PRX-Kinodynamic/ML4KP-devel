#pragma once
#include "prx/simulation/controllers/lqr.hpp"
#include "prx/simulation/multivalued_map/time_map.hpp"

namespace prx
{
namespace simulation
{
static controller_ptr_t pendulum_lqr(const time_map_data_t& tm)
{
  Eigen::Matrix<double, 2, 2> Q{ Eigen::Matrix<double, 2, 2>::Identity() };
  Eigen::Matrix<double, 1, 1> R{ Eigen::Matrix<double, 1, 1>::Identity() };
  auto lqr = std::make_shared<lqr_t>(tm._system, Q, R, "pendulum_lqr");
  lqr->set_goal(tm.x_goal, tm.u_goal);
  lqr->compute_K();
  PRX_DEBUG_VAR_1(lqr->get_K());
  return lqr;
}

}  // namespace simulation
}  // namespace prx

PRX_REGISTER_TM_CONTROLLER(prx::simulation::pendulum_lqr, pendulum_lqr);
