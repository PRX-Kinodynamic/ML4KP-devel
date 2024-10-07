#define BOOST_AUTO_TEST_MAIN lqr_controller_test

#include <boost/test/unit_test.hpp>
#include <string>

#include "prx/utilities/defs.hpp"
#include "prx/simulation/controllers/lqr_controller.hpp"
#include "prx/simulation/plants/pendulum.hpp"
#include "prx/simulation/plants/two_link_acrobot.hpp"

// Checking against system and K from the paper:
// Richards, Spencer M., Felix Berkenkamp, and Andreas Krause. "The lyapunov neural network: Adaptive stability
// certification for safe learning of dynamical systems." In CoRL, pp. 466-476. PMLR, 2018.
BOOST_AUTO_TEST_CASE(lqr_pendulum_test)
{
  prx::simulation_step = 0.01;

  const std::string plant_name{ "pendulum" };
  const std::string plant_path{ "pendulum" };
  prx::system_ptr_t plant = prx::system_factory_t::create_system(plant_name, plant_path);

  prx::space_t* ss{ plant->get_state_space() };
  prx::space_t* cs{ plant->get_control_space() };
  prx::space_t* ps{ plant->get_parameter_space() };

  const double length{ 0.5 };
  const double friction{ 0.1 };
  const double mass{ 0.15 };
  const double normalize{ 0 };
  std::vector<double> v{ length, friction, mass, normalize };
  ps->copy_from(v);
  const double gravity = 9.81;
  const double inertia{ mass * length * length };

  Eigen::MatrixXd A_expected{ Eigen::MatrixXd::Zero(2, 2) };
  Eigen::MatrixXd B_expected{ Eigen::MatrixXd::Zero(2, 1) };
  A_expected << 0., 1., gravity / length, -friction / inertia;
  B_expected << 0., 1. / inertia;

  Eigen::MatrixXd Q{ Eigen::MatrixXd::Identity(2, 2) };
  Eigen::MatrixXd R{ Eigen::MatrixXd::Identity(1, 1) };
  Eigen::VectorXd x0{ Eigen::VectorXd::Zero(2) };
  Eigen::VectorXd u0{ Eigen::VectorXd::Zero(1) };
  prx::simulation::lqr_controller_t ctrl(plant, "LQR", Q, R, x0, u0);

  Eigen::MatrixXd A{ ctrl.lqr().A() };
  Eigen::MatrixXd B{ ctrl.lqr().B() };
  BOOST_CHECK_MESSAGE(prx::are_matrices_approx_equal(A_expected, A, 1e-2), EXPECTED_GOT(A_expected, A));
  BOOST_CHECK_MESSAGE(prx::are_matrices_approx_equal(B_expected, B, 1e-2), EXPECTED_GOT(B_expected, B));

  // Normalizing
  Eigen::DiagonalMatrix<double, 2> Tx, Tx_inv;
  Eigen::DiagonalMatrix<double, 1> Tu, Tu_inv;
  auto ss_ub = ss->get_upper_bounds();
  auto cs_ub = cs->get_upper_bounds();
  Tx.diagonal() << ss_ub[0], ss_ub[1];
  Tu.diagonal() << cs_ub[0];
  Tx_inv.diagonal() << (1.0 / ss_ub[0]), (1.0 / ss_ub[1]);
  Tu_inv.diagonal() << (1.0 / cs_ub[0]);

  A = Tx_inv * A * Tx;
  B = Tx_inv * B * Tu;
  ctrl.lqr().A() = A;
  ctrl.lqr().B() = B;
  ctrl.lqr().compute_K();
  Eigen::MatrixXd K{ ctrl.lqr().K() };

  Eigen::Matrix<double, 1, 2> K_expected{ 7.39050619, 2.60611851 };

  BOOST_CHECK_MESSAGE(prx::are_matrices_approx_equal(K_expected, K, 1e-3), EXPECTED_GOT(K_expected, K));
}
