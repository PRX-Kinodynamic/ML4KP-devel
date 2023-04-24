#define BOOST_TEST_MODULE lqr_test
#include <string>
#include <boost/test/included/unit_test.hpp>
#include "prx/utilities/defs.hpp"
#include "prx/simulation/controllers/lqr.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/utilities/general/transforms.hpp"

BOOST_AUTO_TEST_CASE(pendulum_lqr_test)
{
  prx::simulation_step = 0.01;
  std::string plant_name = "pendulum";
  std::string plant_path = "pendulum";
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);

  const auto ss = plant->get_state_space();
  const auto cs = plant->get_control_space();
  const auto ps = plant->get_parameter_space();

  double length = 0.5;
  double friction = 0.1;
  double mass = 0.15;
  double normalize = 1;
  std::vector<double> v = { length, friction, mass, normalize };
  ps->copy_from(v);

  auto Q = Eigen::MatrixXd::Identity(2, 2);
  auto R = Eigen::MatrixXd::Identity(1, 1);

  prx::lqr_t lqr(plant, Q, R, "LQR");
  lqr.set_goal(Eigen::VectorXd::Zero(2), Eigen::VectorXd::Zero(1));
  lqr.compute_K();
  Eigen::MatrixXd K = lqr.get_K();
  // std::cout << "A: " << lqr.get_linearized_plant -> get_A() << std::endl;
  // std::cout << "B: " << lqr.get_linearized_plant -> get_B() << std::endl;
  std::cout << "K: " << K << std::endl;

  BOOST_CHECK_SMALL(K(0, 0) - 7.39050619, 1e-5);
  BOOST_CHECK_SMALL(K(0, 1) - 2.60611851, 1e-5);
}

BOOST_AUTO_TEST_CASE(acrobot_lqr_test)
{
  prx::simulation_step = 0.01;
  std::cout << "Checking acrobot..." << std::endl;
  std::string plant_name = "Acrobot";
  std::string plant_path = "Acrobot";
  auto system_ptr = prx::system_factory_t::create_system(plant_name, plant_path);
  auto plant = std::dynamic_pointer_cast<prx::plant_t>(system_ptr);

  const auto ss = plant->get_state_space();
  const auto cs = plant->get_control_space();
  const auto ps = plant->get_parameter_space();

  ss->set_bounds({ 0, -M_PI, -6, -6 }, { 2.0 * M_PI, M_PI, 6, 6 });
  cs->set_bounds({ -7 }, { 7 });

  double mass = 1.0;
  double g = 9.81;
  double l1 = 1.0;
  double l2 = 1.0;
  double I1 = 0.2;
  double I2 = 1.0;
  double d1 = 1.0;  // Damping
  double d2 = 1.0;
  double viz_length = 20;
  std::vector<double> v = { mass, g, l1, l2, I1, I2, d1, d2, viz_length };
  ps->copy_from_vector(v);

  // Eigen::Vector4d diagonal;
  Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
  Q.diagonal() << 10, 10, 1, 1;
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1);
  Eigen::Vector4d x_goal(PRX_PI, 0, 0, 0);
  Eigen::Vector<double, 1> u_goal(Eigen::Vector<double, 1>::Zero());

  prx::lqr_t lqr(plant, Q, R, "LQR");
  lqr.set_goal(x_goal, u_goal);
  lqr.compute_K();
  Eigen::MatrixXd K = lqr.get_K();

  std::cout << "K: " << K << std::endl;

  Eigen::Matrix4d A_from_matlab;
  A_from_matlab << 0, 0, 1.000000000000000, 0,       // no-lint
      0, 0, 0, 1.000000000000000,                    // no-lint
      12.907894736841627, -2.581578947368598, 0, 0,  // no-lint
      -14.456842105262695, 8.777368421053325, 0, 0;

  Eigen::MatrixXd B_from_matlab;
  B_from_matlab.resize(4, 1);
  B_from_matlab << 0, 0, -1.578947368421053, 3.368421052631580;

  Eigen::MatrixXd K_from_matlab;
  K_from_matlab.resize(1, 4);
  K_from_matlab << -1.263938740391128 * 1.0e+02, -0.328348178538952 * 1.0e+02,  // no-lint
      -0.492746311271306 * 1.0e+02, -0.189862747615557 * 1.0e+02;

  PRX_DEBUG_VAR_1(A_from_matlab);
  PRX_DEBUG_VAR_1(B_from_matlab);

  Eigen::MatrixXd A{ Eigen::Matrix4d::Zero() };
  Eigen::MatrixXd B{ Eigen::Matrix4d::Zero() };
  plant->linearize(A, B);
  PRX_DEBUG_VAR_1(A);
  PRX_DEBUG_VAR_1(B);
  BOOST_CHECK(prx::are_matrices_approx_equal(A, A_from_matlab, 1e-5));
  BOOST_CHECK(prx::are_matrices_approx_equal(B, B_from_matlab, 1e-5));
  BOOST_CHECK(prx::are_matrices_approx_equal(K, K_from_matlab, 1e-5));

  // BOOST_CHECK(prx::are_approx_equal(K(0,0), 7.39050619, 1e-5));
  // BOOST_CHECK(prx::are_approx_equal(K(0,1), 2.60611851, 1e-5));
}
