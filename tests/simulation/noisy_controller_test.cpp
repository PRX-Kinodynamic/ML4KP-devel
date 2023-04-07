#define BOOST_TEST_MODULE noisy_controller_test
#include <boost/test/included/unit_test.hpp>
#include <string>

#include "prx/simulation/plants/plants.hpp"
#include "prx/simulation/controllers/lqr.hpp"
#include "prx/simulation/controllers/noisy_controller.hpp"
#include "prx/simulation/plants/two_link_acrobot.hpp"
#include "prx/simulation/plants/two_dimensional_point.hpp"
template <typename T>
void check(prx::system_ptr_t plant)
{
  BOOST_CHECK(plant != nullptr);

  // double epsilon = 0.01;
  // auto plant = prx::system_factory_t::create_system("pendulum", "pendulum-noisy");

  auto ss = plant->get_state_space();
  auto cs = plant->get_control_space();

  auto Q = Eigen::MatrixXd::Identity(ss->get_dimension(), ss->get_dimension());
  auto R = Eigen::MatrixXd::Identity(cs->get_dimension(), cs->get_dimension());

  // auto plant_lin = std::dynamic_pointer_cast<T>(plant);
  // auto plant_lin = std::make_shared<prx::lti_t>(plant);

  // plant_lin -> linearize();

  auto lqr_ctrl = std::make_shared<prx::lqr_t>(plant, Q, R, "lqr");
  // auto lqr_ctrl_ptr = lqr_ctrl.get_ptr();
  BOOST_CHECK(lqr_ctrl != nullptr);
  lqr_ctrl->get_linearized_plant()->linearize();
  lqr_ctrl->compute_K();
  prx::noisy_controller_t<std::uniform_real_distribution<double>> n_ctrl(lqr_ctrl, -0.5, 0.5);

  auto pt = cs->make_point();
  auto pt_lqr = cs->make_point();
  auto pt_noisy = cs->make_point();

  for (int i = 0; i < 10000; ++i)
  {
    cs->sample(pt);
    cs->copy_from_point(pt);
    lqr_ctrl->compute_controls(pt_lqr);
    cs->copy_from_point(pt);
    n_ctrl.compute_controls(pt_noisy);

    double e1, e2;
    for (auto e : prx::zip_iters(pt_lqr, pt_noisy))
    {
      std::tie(e1, e2) = prx::unzip(e);
      std::cout << "e1: " << e1 << "\te2: " << e2 << std::endl;
      BOOST_CHECK(std::fabs(e1 - e2) <= 0.5);
    }
  }

  printf("%s:\t[ OK ]\n", __PRETTY_FUNCTION__);
}

BOOST_AUTO_TEST_CASE(noisy_controller_test)
{
  auto plant_1 = prx::system_factory_t::create_system("pendulum", "pendulum-noisy");
  auto plant_2 = prx::system_factory_t::create_system("Acrobot", "acrobot-noisy");

  check<prx::pendulum_t>(plant_1);
  check<prx::two_link_acrobot_t>(plant_2);
}