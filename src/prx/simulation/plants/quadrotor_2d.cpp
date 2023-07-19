#include "prx/simulation/plants/quadrotor_2d.hpp"

namespace prx
{
quadrotor_2d_t::quadrotor_2d_t(const std::string& path) : plant_t(path)
{
  x = z = theta = xdot = thetadot = zdot = 0;
  state_memory = { &x, &z, &theta, &vx, &vz, &thetadot };
  state_space = new space_t("EEREEE", state_memory, "xzthetadxdzdtheta");
  state_space->set_bounds({ -10.0, 0.0, -PRX_PI / 4, -2, -1, -PRX_PI / 3 },
                          { 10.0, 20.0, PRX_PI / 4, 2, 1, PRX_PI / 3 });

  T1 = T2 = 0;
  control_memory = { &T1, &T2 };
  input_control_space = new space_t("EE", control_memory, "T1T2");
  input_control_space->set_bounds({ -10.0, -10.0 }, { 10.0, 10.0 });

  derivative_memory = { &xdot, &zdot, &thetadot, &vxdot, &vzdot, &thetadotdot };
  derivative_space = new space_t("EEEEEE", derivative_memory, "dxdzthetaddxddzddtheta");

  geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::SPHERE);
  geometries["body"]->initialize_geometry({ 0.1 });
  geometries["body"]->generate_collision_geometry();
  geometries["body"]->set_visualization_color("0x00ff00");
  configurations["body"] = std::make_shared<transform_t>();
  configurations["body"]->setIdentity();

  set_integrator(integrator_t::kEULER);
}

quadrotor_2d_t::~quadrotor_2d_t()
{
}

void quadrotor_2d_t::propagate(const double simulation_step)
{
  integrator->integrate(simulation_step);
}

void quadrotor_2d_t::update_configuration()
{
  auto body = configurations["body"];
  body->setIdentity();
  body->linear() = (quaternion_t(0, 0, 0, 1).toRotationMatrix());
  body->translation() = (vector_t(x, 0, z));
}

void quadrotor_2d_t::compute_derivative()
{
  xdot = vx * std::cos(theta) - vz * std::sin(theta);
  zdot = vx * std::sin(theta) + vz * std::cos(theta);
  vxdot = vz * thetadot - g * std::sin(theta);
  vzdot = -vx * thetadot - g * std::cos(theta) + (T1 + T2) / m;
  thetadotdot = l * (T1 - T2) / J;
}
}  // namespace prx