#define BOOST_TEST_MODULE plant_test
#include <boost/test/included/unit_test.hpp>

#include <string>

#include "prx/simulation/plant.hpp"

const double tolerance{ 0.01 };

// Tank example from: https://www.cds.caltech.edu/~murray/courses/cds101/fa02/caltech/pph02-ch19-23.pdf
struct tank_test_t : prx::plant_t
{
  const double Tc{ 10.0 };
  const double Th{ 90.0 };
  const double At{ 3.0 };
  const double A0{ 0.05 };
  const double cd{ 0.7 };
  const double g{ 9.81 };

  double _h, _t;
  double _hdot, _tdot;
  double _qc, _qh;
  tank_test_t(const std::string& path) : prx::plant_t(path)
  {
    _h = _t = 0;
    state_memory = { &_h, &_t };
    state_space = new prx::space_t("EE", state_memory, "state");

    control_memory = { &_qc, &_qh };
    input_control_space = new prx::space_t("EE", control_memory, "control");

    _hdot = _tdot = 0;
    derivative_memory = { &_hdot, &_tdot };
    derivative_space = new prx::space_t("EE", derivative_memory, "deriv");

    parameter_space = new prx::space_t("", parameter_memory, "params");
    set_integrator(prx::integrator_t::kRK4);
  }
  virtual void update_configuration() override{};

  virtual void compute_derivative() override
  {
    _hdot = (1.0 / At) * (_qc + _qh - cd * A0 * std::sqrt(2 * g * _h));
    _tdot = (1.0 / (_h * At)) * (_qc * (Tc - _t) + _qh * (Th - _t));
  }

  Eigen::Matrix2d analytical_A()
  {
    Eigen::Matrix2d A;
    A << -(g * cd * A0) / (At * std::sqrt(2 * g * _h)), 0,  // no-lint
        -(_qc * (Tc - _t) + _qh * (Th - _t)) / (_h * _h * At), -(_qc + _qh) / (_h * At);
    return A;
  }

  Eigen::Matrix2d analytical_B()
  {
    Eigen::Matrix2d A;
    A << (1.0 / At), (1.0 / At),  // no-lint
        (Tc - _t) / (_h * At), (Th - _t) / (_h * At);
    return A;
  }
};

BOOST_AUTO_TEST_CASE(plant_linearization_test)
{
  prx::simulation_step = 0.01;
  tank_test_t plant("plant");
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::Matrix2d A_expected;
  Eigen::Matrix2d B_expected;

  Eigen::Vector2d x0(1.0, 25.0);
  Eigen::Vector2d u0(0.126, 0.029);

  plant.get_state_space()->copy_from(x0);
  plant.get_control_space()->copy_from(u0);

  A_expected = plant.analytical_A();
  B_expected = plant.analytical_B();
  plant.linearize(A, B);

  Eigen::Vector2d x1(1.0, 75.0);
  Eigen::Vector2d u1(0.029, 0.126);

  plant.get_state_space()->copy_from(x1);
  plant.get_control_space()->copy_from(u1);

  A_expected = plant.analytical_A();
  B_expected = plant.analytical_B();
  plant.linearize(A, B);

  BOOST_CHECK_SMALL((A - A_expected).norm(), tolerance);
  BOOST_CHECK_SMALL((B - B_expected).norm(), tolerance);

  Eigen::Vector2d x2(3.0, 25.0);
  Eigen::Vector2d u2(0.218, 0.0503);

  plant.get_state_space()->copy_from(x2);
  plant.get_control_space()->copy_from(u2);

  A_expected = plant.analytical_A();
  B_expected = plant.analytical_B();
  plant.linearize(A, B);

  BOOST_CHECK_SMALL((A - A_expected).norm(), tolerance);
  BOOST_CHECK_SMALL((B - B_expected).norm(), tolerance);

  Eigen::Vector2d x3(3.0, 75.0);
  Eigen::Vector2d u3(0.0503, 0.2181);

  plant.get_state_space()->copy_from(x3);
  plant.get_control_space()->copy_from(u3);

  A_expected = plant.analytical_A();
  B_expected = plant.analytical_B();
  plant.linearize(A, B);

  BOOST_CHECK_SMALL((A - A_expected).norm(), tolerance);
  BOOST_CHECK_SMALL((B - B_expected).norm(), tolerance);
}
