
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
{
  state_memory = { &_position[0], &_position[1], &_theta, &_current_vel };
  state_space = new space_t("EERE", state_memory, "pendulum_state");
  state_space->set_bounds({ -100, -100, -prx::constants::pi, -10 }, { 100, 100, prx::constants::pi, 10 });

  control_memory = { &_u[0], &_u[1] };
  input_control_space = new space_t("EE", control_memory, "Torque");
  input_control_space->set_bounds({ -10, -prx::constants::pi / 2.0 }, { 10, prx::constants::pi / 2.0 });

  derivative_memory = { &_linear_v[0], &_linear_v[1], &_omega, &_vel_delta };
  derivative_space = new space_t("EEEE", derivative_memory, "pendulum_deriv");

  parameter_memory = { &_vel_delta_max };
  parameter_space = new space_t("E", parameter_memory, "pendulum_params");

  geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::CONE);
  geometries["body"]->initialize_geometry({ 0.5, 1 });
  geometries["body"]->generate_collision_geometry();
  geometries["body"]->set_visualization_color("0x00ff00");
  configurations["body"] = std::make_shared<transform_t>();
  configurations["body"]->setIdentity();

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
  _T.linear() = Eigen::Matrix3d{ Eigen::AngleAxisd(_theta, Eigen::Vector3d::UnitZ()) };
  _T.translation() = _position;

  const double desired_velocity{ _u[0] };
  _vel_delta = desired_velocity - _current_vel;
  _vel_delta = std::max(std::min(_vel_delta, _vel_delta_max), -_vel_delta_max);
  // smoothed_rpm = self.last_rpm + clipped_delta;

  const double gamma{ _u[0] * std::tan(_u[1]) / 0.23 };
  _body_twist.head(3) = Eigen::Vector3d(0.0, 0.0, gamma);
  _body_twist.tail(3) = Eigen::Vector3d(_current_vel, 0.0, 0.0);

  _spatial_twist = adjoint(_T) * _body_twist;
  _Tdot.matrix() = matrix_twist(_spatial_twist) * _T.matrix();

  _linear_v = _Tdot.translation();
  _omega = vee(_Tdot.matrix().block<3, 3>(0, 0))[2];
}

}  // namespace prx