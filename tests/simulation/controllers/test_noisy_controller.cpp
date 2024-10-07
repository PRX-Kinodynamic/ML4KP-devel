#define BOOST_AUTO_TEST_MAIN noisy_controller_test
#include <string>
#include <boost/test/unit_test.hpp>

#include "prx/simulation/plants/plants.hpp"
#include "prx/simulation/controllers/lqr.hpp"
#include "prx/simulation/controllers/noisy_controller.hpp"
#include "prx/simulation/plants/two_link_acrobot.hpp"
#include "prx/simulation/plants/two_dimensional_point.hpp"

BOOST_AUTO_TEST_CASE(noisy_controller_test)
{
  // ToDo: Revisit the noise implementation
}