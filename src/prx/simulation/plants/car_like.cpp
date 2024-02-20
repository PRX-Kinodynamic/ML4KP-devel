#include "prx/simulation/plants/car_like.hpp"

namespace prx
{
car_like_t::car_like_t(const std::string& path) : plant_t(path)
{
  x = y = theta = phi = v = 0;
  state_memory = { &x, &y, &theta, &phi, &v };
  state_space = new space_t("EEREE", state_memory, "CarLikeState");
  state_space->set_bounds(lower_bound, upper_bound);

  dx = dy = dtheta = dphi = dv = 0;
  derivative_memory = { &dx, &dy, &dtheta, &dphi, &dv };
  derivative_space = new space_t("EEEEE", derivative_memory, "CarLikeDerivative");

  control_memory = { &dphi, &dv };
  input_control_space = new space_t("EE", control_memory, "CarLikeControl");
  input_control_space->set_bounds({ -0.5236, -0.1 }, { 0.5236, 0.1 });

  geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::BOX);
  geometries["body"]->initialize_geometry({ 1.0, .6, .25 });
  geometries["body"]->generate_collision_geometry();
  geometries["body"]->set_visualization_color("0xff00ff");
  configurations["body"] = std::make_shared<transform_t>();
  configurations["body"]->setIdentity();

  set_integrator(integrator_t::kEULER);
}

car_like_t::~car_like_t()
{
}

void car_like_t::propagate(const double simulation_step)
{
  integrator->integrate(simulation_step);
}

void car_like_t::update_configuration()
{
  auto body = configurations["body"];
  body->setIdentity();
  body->linear() = (quaternion_t(cos(theta / 2), 0, 0, sin(theta / 2)).toRotationMatrix());
  body->translation() = (vector_t(x, y, 0));
}

void car_like_t::compute_derivative()
{
  // dx = v * cos(theta) * cos(phi);
  // dy = v * sin(theta) * cos(phi);
  // dtheta = v * sin(phi) / L;
  dx = v * cos(theta);
  dy = v * sin(theta);
  dtheta = v * tan(phi) / L;
}
}  // namespace prx