#define BOOST_AUTO_TEST_MAIN noisy_controller_test
#include <string>
#include <boost/test/unit_test.hpp>

#include "prx/simulation/plants/SE2_holonomic_system.hpp"
using Plant = prx::SE2_holonomic_system_t;
using PlantPtr = std::shared_ptr<Plant>;
BOOST_AUTO_TEST_CASE(plant_create_test)
{
  PRX_DEBUG_PRINT
  PlantPtr plant{ Plant::create() };
  PRX_DEBUG_PRINT
  BOOST_CHECK(plant != nullptr);
  // check<prx::two_link_acrobot_t>(plant_2);
}

BOOST_AUTO_TEST_CASE(propagate_with_derivatives_test)
{
  PRX_DEBUG_PRINT
  PlantPtr plant{ Plant::create() };
  Plant::State x0{ Plant::State() };
  Plant::Control u0{ Plant::Control::Zero() };
  PRX_DEBUG_PRINT
  Eigen::Matrix<double, 6, 6> Hx;
  Eigen::Matrix<double, 6, 3> Hu;
  Eigen::Matrix<double, 6, 1> Hdt;
  auto x1 = plant->propagate(x0, u0, 0.1);
  auto x1h = plant->propagate(x0, u0, 0.1, &Hx, &Hu, &Hdt);
  PRX_DEBUG_PRINT
  PRX_DBG_VARS(Hx);
  PRX_DBG_VARS(Hu);
  PRX_DBG_VARS(Hdt);
}