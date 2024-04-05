
#include "prx/simulation/plants/first_order_free_body.hpp"

namespace prx
{

first_order_free_body_t::first_order_free_body_t(const std::string& path)
  : plant_t(path)
  , _x(Position::Zero())
  , _quat(Quaternion::Identity())
  , _xdot(Velocity::Zero())
  , _omega(Velocity::Zero())
  , _width(15.957)
  , _depth(9.910)
  , _length(50.0)
{
  state_memory = {
    &_x[0],     &_x[1],     &_x[2],                 // no-lint
    &_quat.w(), &_quat.x(), &_quat.y(), &_quat.z()  // no-lint
  };

  state_space = new space_t("EEEQQQQ", state_memory, "first_order_free_body_state");
  state_space->set_bounds({ -100, -100, -100, -1, -1, -1, -1 }, { 100, 100, 100, 1, 1, 1, 1 });

  control_memory = {
    &_xdot[0],  &_xdot[1],  &_xdot[2],   // no-lint
    &_omega[0], &_omega[1], &_omega[2],  // no-lint
  };

  input_control_space = new space_t(std::string(6, 'E'), control_memory, "first_order_free_body_ctrl");
  input_control_space->set_bounds(std::vector<double>(6, -0.5), std::vector<double>(6, 0.5));

  derivative_memory = {
    &_xdot[0],     &_xdot[1],     &_xdot[2],                    // no-lint
    &_quatdot.w(), &_quatdot.x(), &_quatdot.y(), &_quatdot.z()  // no-lint
  };
  derivative_space = new space_t("EEEQQQQ", derivative_memory, "first_order_free_body_deriv");

  parameter_memory = { &_width, &_depth, &_length };
  parameter_space = new space_t("EEE", parameter_memory, "pendulum_params");

  geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::BOX);
  geometries["body"]->initialize_geometry({ _width, _depth, _length });
  geometries["body"]->generate_collision_geometry();
  geometries["body"]->set_visualization_color("0xbb00ff00");
  configurations["body"] = std::make_shared<transform_t>();
  configurations["body"]->setIdentity();

  set_integrator(integrator_t::kRK4);
}

first_order_free_body_t::~first_order_free_body_t()
{
}

void first_order_free_body_t::propagate(const double simulation_step)
{
  integrator->integrate(simulation_step);
}

void first_order_free_body_t::steer(const space_point_t x, const space_point_t y, double ti)
{
  // const double total_steps{ eta / prx::simulation_step };
  // std::vector<double> steps{ prx::linspace(0.0, 1.0, total_steps) };
  // for (auto ti : steps)
  // {
  state_space->interpolate(x, y, ti);
  // traj.copy_onto_back(z);
  //   ti += prx::simulation_step;
  //   if (eta < distance_function(x, z))
  //     break;
  // }
}

void first_order_free_body_t::update_configuration()
{
  auto body = configurations["body"];
  body->linear() = _quat.toRotationMatrix();
  body->translation() = _x;
}

void first_order_free_body_t::compute_derivative()
{
  // https://www.cs.cmu.edu/~baraff/sigcourse/notesd1.pdf
  // \dot{quat} = 0.5 * [0,\omega] * quat (4-2)
  omega_to_quaternion();
  _quat_omega.coeffs() *= 0.5;
  _quatdot = _quat_omega * _quat;
}

}  // namespace prx