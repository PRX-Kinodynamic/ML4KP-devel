#include "prx/simulation/plants/mushr.hpp"

namespace prx
{

mushr_t::mushr_t(const std::string& path)
  : plant_t(path)
  , _T(Transform::Identity())
  , _Tdot(Transform::Identity())
  , _position(Position::Zero())
  , _theta(0.0)
  , _rotation(0.0)
  , _u(Control::Zero())
  , _linear_v(Velocity::Zero())
  , _omega(0.0)
  , _vel_delta_max(1.0)
  , _steering_offset(0.0)
  , _steering_gain(1.0)
  , _length(0.2965)
{
  state_memory = { &_position[0], &_position[1], &_theta, &_current_vel };
  state_space = new space_t("EERE", state_memory, "mushr_state");
  state_space->set_bounds({ -100, -100, -prx::constants::pi, -10 }, { 100, 100, prx::constants::pi, 10 });

  control_memory = { &_u[0], &_u[1] };
  input_control_space = new space_t("EE", control_memory, "Torque");
  input_control_space->set_bounds({ -prx::constants::pi / 2.0, -10 }, { prx::constants::pi / 2.0, 10 });

  derivative_memory = { &_qdot[1], &_qdot[2], &_qdot[0], &_vel_delta };
  derivative_space = new space_t("EEEE", derivative_memory, "mushr_deriv");

  parameter_memory = { &_vel_delta_max, &_steering_offset, &_steering_gain, &_length };
  parameter_space = new space_t("EEEE", parameter_memory, "mushr_params");

  geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::CONE);
  geometries["body"]->initialize_geometry({ 0.5, 1 });
  geometries["body"]->generate_collision_geometry();
  geometries["body"]->set_visualization_color("0x00ff00");
  configurations["body"] = std::make_shared<transform_t>();
  configurations["body"]->setIdentity();

  _g << 0, 1, 0, 0, 0, 0;

  set_integrator(integrator_t::kRK4);
}

mushr_t::~mushr_t()
{
}

void mushr_t::propagate(const double simulation_step)
{
  integrator->integrate(simulation_step);
}

void mushr_t::update_configuration()
{
  auto body = configurations["body"];
  body->linear() = Eigen::Matrix3d{ Eigen::AngleAxisd(_theta, Eigen::Vector3d::UnitZ()) };
  body->translation() = _position;
}

void mushr_t::compute_derivative()
{
  _g(1, 0) = std::cos(_theta);
  _g(2, 0) = std::sin(_theta);
  _vel_delta = desired_velocity() - _current_vel;
  _vel_delta = std::max(std::min(_vel_delta, _vel_delta_max), -_vel_delta_max);
  const double w{ _current_vel * std::tan(steering()) / _length };
  _qdot = _g * Eigen::Vector2d(_current_vel, w);
}

}  // namespace prx