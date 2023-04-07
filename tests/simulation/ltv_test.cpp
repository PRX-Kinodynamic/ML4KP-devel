#define BOOST_TEST_MODULE ltv_test
#include <boost/test/included/unit_test.hpp>
#include <string>
#include "prx/utilities/defs.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/utilities/general/transforms.hpp"
#include "prx/simulation/plants/types/linear_time_variant.hpp"
#include "prx/simulation/controllers/lqr.hpp"

void acrobot_check()
{
  std::cout << "Checking acrobot..." << std::endl;
  std::string plant_name = "Acrobot";
  std::string plant_path = "Acrobot";
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);

  const auto ss = plant->get_state_space();
  const auto cs = plant->get_control_space();
  const auto ps = plant->get_parameter_space();

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

  // auto acrobot = std::dynamic_pointer_cast<prx::two_link_acrobot_t>(plant);
  auto acrobot = std::make_shared<prx::ltv_t>(plant);

  acrobot->linearize();
  PRX_DEBUG_PRINT

  std::cout << "A: " << acrobot->get_A() << std::endl;
  std::cout << "B: " << acrobot->get_B() << std::endl;

  Eigen::Matrix4d A_from_matlab;
  A_from_matlab << 0, 0, 1.000000000000000, 0, 0, 0, 0, 1.000000000000000, 12.907894736841627, -2.581578947368598, 0, 0,
      -14.456842105262695, 8.777368421053325, 0, 0;

  PRX_DEBUG_PRINT
  Eigen::MatrixXd B_from_matlab;
  B_from_matlab.resize(4, 1);
  B_from_matlab << 0, 0, -1.578947368421053, 3.368421052631580;

  std::cout << "A from matlab: " << A_from_matlab << std::endl;
  std::cout << "B from matlab: " << B_from_matlab << std::endl;
  PRX_DEBUG_PRINT
  BOOST_CHECK(prx::are_matrices_approx_equal(acrobot->get_A(), A_from_matlab, 1e-5));
  BOOST_CHECK(prx::are_matrices_approx_equal(acrobot->get_B(), B_from_matlab, 1e-5));

  auto x0 = ss->make_point();
  auto ut = cs->make_point();
  (*x0)[0] = M_PI;
  (*x0)[1] = 0;
  (*x0)[2] = 0;
  (*x0)[3] = 0;
  (*ut)[0] = 0;

  acrobot->linearize(x0, ut, prx::simulation_step);

  std::cout << "After linearizing:" << std::endl;
  std::cout << "A: " << acrobot->get_A() << std::endl;
  std::cout << "B: " << acrobot->get_B() << std::endl;

  Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
  Q.diagonal() << 10, 10, 1, 1;
  std::cout << "Q:" << Q << std::endl;
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1);

  // prx::lqr_t lqr(acrobot, Q, R, "LQR");
  // lqr.compute_K();
  // Eigen::MatrixXd K = lqr.get_K();
  // std::cout << "A: " << acrobot -> get_A() << std::endl;
  // std::cout << "B: " << acrobot -> get_B() << std::endl;
  // std::cout << "K: " << K << std::endl;

  std::cout << "Acrobot OK" << std::endl;
}

BOOST_AUTO_TEST_CASE(lqr_test)
{
  prx::simulation_step = 0.01;
  prx::init_random(112392);

  acrobot_check();
}