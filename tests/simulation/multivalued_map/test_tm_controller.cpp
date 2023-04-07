#define BOOST_TEST_MODULE time_map_test
#include <boost/test/included/unit_test.hpp>
#include <string>
#include "prx/utilities/defs.hpp"
#include "prx/simulation/multivalued_map/systems.hpp"
#include "prx/simulation/multivalued_map/tm_controllers.hpp"

using prx::simulation::time_map_controllers_t;
BOOST_AUTO_TEST_CASE(time_map_controllers_check_available_systems)
{
  std::vector<std::string> tm_controllers_names = { "pendulum_lqr" };
  std::vector<std::string> available_systems{ time_map_controllers_t::available_systems() };
  std::set<std::string> set(available_systems.begin(), available_systems.end());

  for (auto name : tm_controllers_names)
  {
    BOOST_CHECK_MESSAGE(set.count(name) > 0, "System " << name << " not found!");
  }
}
