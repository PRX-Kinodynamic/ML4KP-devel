#define BOOST_AUTO_TEST_MAIN care_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/geometry/geometry.hpp"

BOOST_AUTO_TEST_CASE(geometry_type_from_string_test)
{
  BOOST_CHECK(prx::geometry_t::geometry_type("box") == prx::geometry_type_t::BOX);
  BOOST_CHECK(prx::geometry_t::geometry_type("SPHERE") == prx::geometry_type_t::SPHERE);
  BOOST_CHECK(prx::geometry_t::geometry_type("ELLIpsoid") == prx::geometry_type_t::ELLIPSOID);
  BOOST_CHECK(prx::geometry_t::geometry_type("Capsule") == prx::geometry_type_t::CAPSULE);
  BOOST_CHECK(prx::geometry_t::geometry_type("CONE") == prx::geometry_type_t::CONE);
  BOOST_CHECK(prx::geometry_t::geometry_type("CYLINDER") == prx::geometry_type_t::CYLINDER);
}
