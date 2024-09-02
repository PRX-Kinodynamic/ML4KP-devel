#define BOOST_AUTO_TEST_MAIN lqr_controller_test

#include <boost/test/unit_test.hpp>
#include <string>

#include "prx/utilities/defs.hpp"
#include "prx/simulation/controllers/lqr.hpp"

// Example from https://www.mathworks.com/help/control/ref/lti.lqr.html
// First example "LQR Control for Inverted Pendulum Model"
BOOST_AUTO_TEST_CASE(lqr_from_known_linear_system)
{
  const Eigen::Index Xdim{ 4 };
  const Eigen::Index Udim{ 1 };
  using LQR = prx::simulation::lqr_t<Xdim, Udim>;

  LQR::MatrixA A;
  LQR::MatrixB B;
  LQR::MatrixQ Q;
  LQR::MatrixR R;

  A << 0.0, +1.0, 0.0, 0.0,  // no-lint
      +0.0, -0.1, 3.0, 0.0,  // no-lint
      +0.0, +0.0, 0.0, 1.0,  // no-lint
      +0.0, -0.5, 30., 0.0;

  B << 0.0, 2.0, 0.0, 5.0;

  Q << 1, 0, 0, 0,  // no-lint
      0, 0, 0, 0,   // no-lint
      0, 0, 1, 0,   // no-lint
      0, 0, 0, 0;

  R << 1;

  LQR lqr{ A, B, Q, R };

  LQR::MatrixK K_expected;
  K_expected << -1.0000, -1.7559, 16.9145, 3.2274;

  const LQR::MatrixK K{ lqr.K() };

  const LQR::VectorX x{ 2.0000, 0.5000, 1.0000, 0.5000 };

  const LQR::VectorU u{ lqr(x) };
  const LQR::VectorU u_expected{ -15.6502 };

  BOOST_CHECK_MESSAGE(prx::are_matrices_approx_equal(K_expected, K, 1e-4), EXPECTED_GOT(K_expected, K));
  BOOST_CHECK_MESSAGE(prx::are_matrices_approx_equal(u_expected, u, 1e-4), EXPECTED_GOT(u_expected, u));
}

// Example from https://www.mathworks.com/help/control/ref/lti.lqr.html
// First example "LQR Control using State-Space Matrices"
BOOST_AUTO_TEST_CASE(lqr_from_known_K)
{
  const Eigen::Index Xdim{ 3 };
  const Eigen::Index Udim{ 1 };
  using LQR = prx::simulation::lqr_t<Xdim, Udim>;

  LQR::MatrixK K_known;
  K_known << -0.5034, 52.8645, 1.4142;
  LQR lqr{ K_known };

  const LQR::VectorX x{ 0, 0, 2 };
  const LQR::VectorU u{ lqr(x) };

  // Should be -K * x = -1.4142 * 2
  const LQR::VectorU u_expected{ -2.8284 };

  BOOST_CHECK_MESSAGE(prx::are_matrices_approx_equal(u_expected, u, 1e-4), EXPECTED_GOT(u_expected, u));
}

// BOOS
