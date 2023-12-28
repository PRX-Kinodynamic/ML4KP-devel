#pragma once

#include "prx/simulation/integrators/integrator.hpp"

namespace prx
{
// For RK methods check https://en.wikipedia.org/wiki/Runge–Kutta_methods or any numerical algorithm book
// This implements RK4: $$ Y_{n+1} = Y_n + h * (k1 + 2 * k2 + 2 * k3 + k4) / 6 $$
// Where:
//  $$ k0 = f(Xn, Yn ) $$
//  $$ ki = f(Xn + c_i * h, Yn + ai * h * k_{i-1} ) $$
class runge_kutta4_t : public integrator_t
{
public:
  runge_kutta4_t(space_t* state_space, space_t* derivative_space, std::function<void()> deriv_f, const double h = 0.01)
    : integrator_t(state_space, derivative_space, deriv_f, h)
    , _yn(Eigen::VectorXd::Zero(_dim))
    , _k1(Eigen::VectorXd::Zero(_dim))
    , _k2(Eigen::VectorXd::Zero(_dim))
    , _k3(Eigen::VectorXd::Zero(_dim))
    , _k4(Eigen::VectorXd::Zero(_dim))
  {
  }

  ~runge_kutta4_t(){};

  void integrate(const double simulation_step = 0.0) override;

protected:
  Eigen::VectorXd _k1;
  Eigen::VectorXd _k2;
  Eigen::VectorXd _k3;
  Eigen::VectorXd _k4;

  Eigen::VectorXd _yn;

private:
};
}  // namespace prx
