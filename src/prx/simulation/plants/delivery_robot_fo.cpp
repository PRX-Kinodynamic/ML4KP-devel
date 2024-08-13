#include "prx/simulation/plants/delivery_robot_fo.hpp"

namespace prx
{
  delivery_robot_fo_t::delivery_robot_fo_t(const std::string& path) : plant_t(path)
  {
    x = y = theta = m = 0;
    state_memory = { &x, &y, &theta, &m };
    state_space = new space_t("EERD", state_memory, "DeliveryRobotFOState");
    state_space->set_bounds({ -11, -11, -3.15, 0 }, { 11, 11, 3.15, 10 });

    vl = vr = pick = 0;
    control_memory = { &vl, &vr, &pick };
    input_control_space = new space_t("EED", control_memory, "DeliveryRobotFOControl");
    input_control_space->set_bounds({ -0.7, -0.7, 0 }, { 0.7, 0.7, 1 });

    dx = dy = dtheta = 0;
    derivative_memory = { &dx, &dy, &dtheta, &dm };
    derivative_space = new space_t("EEEE", derivative_memory, "DeliveryRobotFODerivative");

    geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::BOX);
    geometries["body"]->initialize_geometry({ .9, .6, .25 });
    geometries["body"]->generate_collision_geometry();
    geometries["body"]->set_visualization_color("0xff00ff");
    configurations["body"] = std::make_shared<transform_t>();
    configurations["body"]->setIdentity();

    set_integrator(integrator_t::kEULER);
  }

  delivery_robot_fo_t::~delivery_robot_fo_t()
  {
  }

  void delivery_robot_fo_t::propagate(const double simulation_step)
  {
    integrator->integrate(simulation_step);
  }

  void delivery_robot_fo_t::update_configuration()
  {
    auto body = configurations["body"];
    body->setIdentity();
    body->linear() = (quaternion_t(cos(theta / 2), 0, 0, sin(theta / 2)).toRotationMatrix());
    body->translation() << x, y, 0;
  }

  void delivery_robot_fo_t::compute_derivative()
  {
    input_control_space->enforce_bounds();
    dx = cos(theta) * (vl + vr) / 2;
    dy = sin(theta) * (vl + vr) / 2;
    dtheta = (vr - vl) / 0.6;
    dm = pick;
  }
}