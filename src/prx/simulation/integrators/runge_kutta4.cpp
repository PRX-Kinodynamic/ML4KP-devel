#include "prx/simulation/integrators/runge_kutta4.hpp"
#include "prx/utilities/defs.hpp"
namespace prx
{

// Implements the integration step for the given simulation step.
void runge_kutta4_t::integrate(const double simulation_step)
{
  _state_space->copy_to(_yn);

  _h = simulation_step;
  const double h_d2{ _h / 2.0 };
  const Eigen::VectorXd hv{ _h * Eigen::VectorXd::Ones(_dim) };
  const Eigen::VectorXd hvd2{ h_d2 * Eigen::VectorXd::Ones(_dim) };
  const Eigen::VectorXd yn_hv{ _yn + hv };
  const Eigen::VectorXd yn_hvd2{ _yn + hvd2 };

  // Compute k1 = f(xn, yn)
  _compute_derivative();
  _derivative_space->copy_to(_k1);

  // Compute k2 = f(xn + h/2, yn + (h/2) * k1)
  // ss <- yn + 0.5 * h * k1;
  _state_space->integrate(yn_hvd2, _derivative_space, h_d2);
  _compute_derivative();
  _derivative_space->copy_to(_k2);

  // Compute k3 = f(xn + h/2, yn + (h/2) * k2)
  // ss <- yn + 0.5 * h * k2;
  _state_space->integrate(yn_hvd2, _derivative_space, h_d2);
  _compute_derivative();
  _derivative_space->copy_to(_k3);

  // Compute k4 = f(xn + h, yn + h * k3)
  // ss <- yn + h * k4;
  _state_space->integrate(yn_hv, _derivative_space, _h);
  _compute_derivative();
  _derivative_space->copy_to(_k4);

  // ss <- yn + h * k14
  const Eigen::VectorXd k14{ (_k1 + 2.0 * _k2 + 2.0 * _k3 + _k4) / 6.0 };
  _derivative_space->copy_from(k14);
  _state_space->integrate(_yn, _derivative_space, _h);
}
}  // namespace prx