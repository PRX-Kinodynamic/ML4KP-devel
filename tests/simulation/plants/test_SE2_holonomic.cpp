#define BOOST_AUTO_TEST_MAIN noisy_controller_test
#include <string>
#include <boost/test/unit_test.hpp>

#include "prx/simulation/plants/SE2_holonomic_system.hpp"
using Plant = prx::SE2_holonomic_system_t;
using PlantPtr = std::shared_ptr<Plant>;
BOOST_AUTO_TEST_CASE(plant_create_test)
{
  PlantPtr plant{ Plant::create() };

  BOOST_CHECK(plant != nullptr);
  // check<prx::two_link_acrobot_t>(plant_2);
}

BOOST_AUTO_TEST_CASE(propagate_with_derivatives_test)
{
  PlantPtr plant{ Plant::Base::create() };
  Plant::State x0{ Plant::State() };
  Plant::Control u0{ Plant::Control::Zero() };

  Eigen::Matrix<double, 6, 6> Hx;
  Eigen::Matrix<double, 6, 3> Hu;
  Eigen::Matrix<double, 6, 1> Hdt;
  auto x1 = plant->propagate(x0, u0, 0.1);
  auto x1h = plant->propagate(x0, u0, 0.1, &Hx, &Hu, &Hdt);

  PRX_DBG_VARS(Hx);
  PRX_DBG_VARS(Hu);
  PRX_DBG_VARS(Hdt);
}