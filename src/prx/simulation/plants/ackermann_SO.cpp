#include "prx/simulation/plants/ackermann_SO.hpp"

namespace prx
{

ackermann_SO::ackermann_SO(const std::string& path) : plant_t(path)
{
  x = y = theta = velocity = gamma = 0.0;
  state_memory = { &x, &y, &theta, &velocity, &gamma };
  state_space = new space_t("EEREE", state_memory, "ackermann_SO_ss");
  state_space->set_bounds({ -10, -10, -PRX_PI, -10, -10 }, { 10, 10, PRX_PI, 10, 10 });

  gamma_change = accel = 0;
  control_memory = { &gamma_change, &accel };
  input_control_space = new space_t("EE", control_memory, "ackermann_SO_cs");
  input_control_space->set_bounds({ -max_delta_rad, -1.0 }, { max_delta_rad, 1.0 });

  x_dot = y_dot = theta_dot = 0.0;
  derivative_memory = { &x_dot, &y_dot, &theta_dot, &accel, &gamma_change };
  derivative_space = new space_t("EEEEE", derivative_memory, "ackermann_SO_ds");

  parameter_memory = { &L };
  parameter_space = new space_t("E", parameter_memory, "ackermann_SO_ps");

  const double length = L * 2.;

  geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::CONE);
  geometries["body"]->initialize_geometry({ 0.5, length });
  geometries["body"]->generate_collision_geometry();
  geometries["body"]->set_visualization_color("0x00ff00");
  configurations["body"] = std::make_shared<transform_t>();
  configurations["body"]->setIdentity();

  set_integrator(integrator_t::kRK4);
}

ackermann_SO::~ackermann_SO()
{
}

void ackermann_SO::propagate(const double simulation_step)
{
  integrator->integrate(simulation_step);
  state_space->enforce_bounds();
}

void ackermann_SO::update_configuration()
{
  auto body = configurations["body"];
  body->setIdentity();
  body->linear() = (quaternion_t(cos(theta / 2.), 0, 0, sin(theta / 2.)).toRotationMatrix());
  body->translation() = (vector_t(x, y, 0.5));
}

void ackermann_SO::compute_derivative()
{
  x_dot = velocity * std::cos(theta);
  y_dot = velocity * std::sin(theta);
  theta_dot = (velocity / L) * std::tan(gamma);
}

}  // namespace prx