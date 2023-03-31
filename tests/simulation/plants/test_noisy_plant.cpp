#define BOOST_AUTO_TEST_MAIN noisy_controller_test
#include <string>
#include <boost/test/unit_test.hpp>

#include "prx/simulation/plants/plants.hpp"
#include "prx/simulation/controllers/lqr.hpp"
#include "prx/simulation/controllers/noisy_controller.hpp"
#include "prx/simulation/plants/two_link_acrobot.hpp"
#include "prx/simulation/plants/two_dimensional_point.hpp"
#include "prx/simulation/plants/types/noisy_plant.hpp"

template <typename T>
void check(prx::system_ptr_t plant)
{
  auto n_plant = new prx::noisy_plant_t<prx::uniform_noise_t>(plant, -0.5, 0.5);

  auto ns = n_plant->get_state_space();
  auto ss = plant->get_state_space();

  std::cout << "ns: " << ns << std::endl;
  std::cout << "ss: " << ss << std::endl;
  auto pt = ss->make_point();
  auto pt_n = ns->make_point();
  for (int i = 0; i < 1000; ++i)
  {
    ss->sample(pt);
    ss->copy_from(pt);
    ns->copy_to(pt_n);

    std::cout << "point: " << pt << std::endl;
    std::cout << "noisy point: " << pt_n << std::endl;
    for (int j = 0; j < ss->get_dimension(); ++j)
    {
      BOOST_CHECK(!(ss->equal_points(pt_n, pt)));
      BOOST_CHECK(std::fabs((*pt_n)[j] - (*pt)[j]) <= 0.5);
    }
  }
  printf("%s:\t[ OK ]\n", __PRETTY_FUNCTION__);
}

BOOST_AUTO_TEST_CASE(noisy_controller_test)
{
  auto plant_1 = prx::system_factory_t::create_system("pendulum", "pendulum-noisy");
  // auto plant_2 = prx::system_factory_t::create_system("Acrobot", "acrobot-noisy");

  check<prx::pendulum_t>(plant_1);
  // check<prx::two_link_acrobot_t>(plant_2);
}