#pragma once

#include "prx/simulation/plant.hpp"
#include "prx/simulation/controller.hpp"
#include "prx/simulation/plants/types/linear_time_variant.hpp"

namespace prx
{
class lander_LD_t : public plant_t
{
public:
  lander_LD_t(const std::string& path);
  virtual ~lander_LD_t();

  virtual void propagate(const double simulation_step) override final;

  virtual void update_configuration() override;

  // virtual bool linearize(space_point_t xt, space_point_t ut, double epsilon) override;

protected:
  virtual void compute_derivative() override final;

  // double _theta1,_theta2,_theta1dot,_theta2dot,_tau,_theta1dotdot,_theta2dotdot;
  double h, h_dot, h_ddot;
  double v, v_dot;
  double m, m_dot;
  double u;

  // -alpha <= u(t) <= 0
  // double alpha = ;

  // Mass of the system when no fuel is left
  double M_0 = 10334;

  // Mass of lander without fuel

  double gravity = 1.62;
  double k = 2048;  // velocity of the exhaust gases with respect to the spacecraft
  double _h_neg;
};

class lander_meditch_ctrl_t : public controller_t
{
public:
  lander_meditch_ctrl_t(system_ptr_t _plant, std::string _name = "base_controller") : controller_t(_plant, _name)
  {
    _aux = _plant->get_control_space()->make_point();
    std::cout << "control limits: " << std::endl;
    _plant->get_control_space()->print_bounds();

    auto ss = _plant->get_state_space();
    auto cs = _plant->get_control_space();
    auto ps = _plant->get_parameter_space();

    // double x1 = ss -> at(0);
    // double x2 = ss -> at(1);
    double M_0 = ps->at(0);

    PRX_DEBUG_VARS(M_0)

    double k = ps->at(1);
    double g = ps->at(2);

    double alpha = cs->get_upper_bound(0);

    PRX_DEBUG_VARS(k, g, alpha)

    a = 0.5 * (k * alpha - g * M_0) / M_0;
    b = (k * alpha * alpha) / (2.0 * M_0 * M_0);
    PRX_DEBUG_VARS(a, b)
  }

  virtual void compute_controls() override
  {
    auto ss = _plant->get_state_space();
    auto cs = _plant->get_control_space();
    auto ps = _plant->get_parameter_space();

    double x1 = ss->at(0);
    double x2 = ss->at(1);
    // double M_0 = ss -> at(2);

    // double k   = ps -> at(1);
    // double g   = ps -> at(2);

    // double alpha = cs -> get_upper_bound(0);

    auto f = (b / a) * x1 + 2 * a * std::sqrt(x1 / a) + x2;

    (*_aux)[0] = 0;
    if (f <= 0)
    {
      (*_aux)[0] = cs->get_upper_bound(0);
    }

    cs->copy_from_point(_aux);
  }

private:
  double a, b;
  space_point_t _aux;
};
}  // namespace prx

PRX_REGISTER_SYSTEM(lander_LD_t, lander_LD)