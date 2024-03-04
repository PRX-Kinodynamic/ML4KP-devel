#include "prx/factor_graphs/factors/mushr_factors.hpp"

namespace prx
{
namespace fg
{

mushrFG_t::mushrFG_t(const std::string& path) : plant_t(path), _length(0.2965), _params_dv(0.01)
{
  state_memory = { &_state[0], &_state[1], &_state[2], &_state_dot[0], &_state_dot[1], &_state_dot[2] };
  state_space = new space_t("EEREEE", state_memory, "mushr_state");
  state_space->set_bounds({ -100, -100, -prx::constants::pi, -10, -10, -10 },
                          { 100, 100, prx::constants::pi, 10, 10, 10 });

  control_memory = { &_ctrl[0], &_ctrl[1] };
  input_control_space = new space_t("EE", control_memory, "mushr_ctrl");
  input_control_space->set_bounds({ -prx::constants::pi / 2.0, -10 }, { prx::constants::pi / 2.0, 10 });

  // derivative_memory = { &_qdot[1], &_qdot[2], &_qdot[0], &_vel_delta };
  // derivative_space = new space_t("EEEE", derivative_memory, "mushr_deriv");

  parameter_memory = { &_params[0], &_params[1], &_params[2], &_params[3], &_params[4], &_params[5], &_params_dv[0] };
  parameter_space = new space_t("EEEEEEE", parameter_memory, "mushr_params");

  geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::CONE);
  geometries["body"]->initialize_geometry({ 0.5, 1 });
  geometries["body"]->generate_collision_geometry();
  geometries["body"]->set_visualization_color("0x00ff00");
  configurations["body"] = std::make_shared<transform_t>();
  configurations["body"]->setIdentity();

  // set_integrator(integrator_t::kRK4);
}

mushrFG_t::~mushrFG_t()
{
}

void mushrFG_t::propagate(const double simulation_step)
{
  _ubar = mushr_ub_u_xdot_t::predict(_ctrl, _state_dot, _params_dv, _length);
  _state_dot = mushr_x_xdot_ub_t::predict(_state, _ubar);
  _state = mushr_x_xdot_t::predict(_state, _state_dot, simulation_step);
}

void mushrFG_t::update_configuration()
{
  auto body = configurations["body"];
  body->linear() = Eigen::Matrix3d{ Eigen::AngleAxisd(_state[2], Eigen::Vector3d::UnitZ()) };
  body->translation().head(2) = _state.head(2);
  body->translation()[2] = 0.0;
}

void mushrFG_t::compute_derivative()
{
}
}  // namespace fg
}  // namespace prx