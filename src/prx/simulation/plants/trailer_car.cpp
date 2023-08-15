#include "prx/simulation/plants/trailer_car.hpp"

namespace prx
{
trailer_car_t::trailer_car_t(const std::string& path) : plant_t(path)
{
  x = y = theta0 = theta1 = 0.0;
  state_memory = {&x, &y, &theta0, &theta1};
  state_space = new space_t("EERR", state_memory, "XYTT");
  state_space -> set_bounds(lower_bound, upper_bound);

  v = phi = 0.0;
  control_memory = {&v, &phi};
  input_control_space = new space_t("EE", control_memory, "VPHI");
  input_control_space -> set_bounds({-0.1,-prx::constants::pi/3}, {0.5, prx::constants::pi/3});

  dx = dy = dtheta0 = dtheta1 = 0.0;
  derivative_memory = {&dx, &dy, &dtheta0, &dtheta1};
  derivative_space = new space_t("EEEE", derivative_memory, "XYTT");

  geometries["car"] = std::make_shared<geometry_t>(geometry_type_t::BOX);
  geometries["car"]->initialize_geometry({0.5, 0.25, 0.2});
  geometries["car"]->generate_collision_geometry();
  geometries["car"]->set_visualization_color("0xff00ff");
  configurations["car"] = std::make_shared<transform_t>();
  configurations["car"]->setIdentity();

  geometries["trailer"] = std::make_shared<geometry_t>(geometry_type_t::BOX);
  geometries["trailer"]->initialize_geometry({0.3, 0.25, 0.2});
  geometries["trailer"]->generate_collision_geometry();
  geometries["trailer"]->set_visualization_color("0xff00ff");
  configurations["trailer"] = std::make_shared<transform_t>();
  configurations["trailer"]->setIdentity();

  set_integrator(integrator_t::kEULER);
}

trailer_car_t::~trailer_car_t()
{
}

void trailer_car_t::propagate(const double simulation_step)
{
  integrator->integrate(simulation_step);
  if (norm_angle_pi(theta0 - theta1) > prx::constants::pi / 4)
  {
    // Ensure theta1 is always within pi/4 of theta0
    theta1 = theta0 + prx::constants::pi / 4 * prx::sgn(theta1 - theta0);
  }
}

void trailer_car_t::update_configuration()
{
  auto car_body = configurations["car"];
  car_body->setIdentity();
  car_body->linear() = (quaternion_t(cos(theta0/2), 0, 0, sin(theta0/2)).toRotationMatrix());
  car_body->translation() = (vector_t(x, y, 0.0));

  auto trailer_body = configurations["trailer"];
  trailer_body->setIdentity();
  trailer_body->translation() = (vector_t(x - d1 * cos(theta1), y - d1 * sin(theta1), 0.0));
  trailer_body->linear() = (quaternion_t(cos(theta1/2), 0, 0, sin(theta1/2)).toRotationMatrix());
}

void trailer_car_t::compute_derivative()
{
  dx = v * std::cos(theta0);
  dy = v * std::sin(theta0);
  dtheta0 = v * std::tan(phi) / L;
  dtheta1 = v * std::sin(theta0 - theta1) / L;
}
} // namespace prx