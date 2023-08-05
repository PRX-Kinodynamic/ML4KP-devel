#define BOOST_AUTO_TEST_MAIN spaces_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/spaces/space.hpp"

namespace mock
{
struct space3d_t
{
  space3d_t() : x(0), y(0), theta(0), address({ &x, &y, &theta }), space("EER", address, "space_test")
  {
  }
  double x, y, theta;
  std::vector<double*> address;
  prx::space_t space;
};
}  // namespace mock

BOOST_AUTO_TEST_CASE(test_space_is_built_correctly)
{
  mock::space3d_t test;
  test.x = 1;
  test.y = 2;
  test.theta = 3;
  BOOST_CHECK(test.space.size() == 3);
  BOOST_CHECK(test.space.get_topology() == "EER");
  BOOST_CHECK(test.space.at(0) == 1);
  BOOST_CHECK(test.space.at(1) == 2);
  BOOST_CHECK(test.space.at(2) == 3);
}

BOOST_AUTO_TEST_CASE(test_space_copy_and_clone_point)
{
  mock::space3d_t test;
  prx::space_t& space = test.space;

  prx::space_point_t pt_sp1 = space.make_point();

  space.copy_to(pt_sp1);
  prx::space_point_t pt_sp2 = space.clone_point(pt_sp1);
  BOOST_CHECK(space.equal_points(pt_sp1, pt_sp2));

  (*pt_sp1)[0] = 10;
  (*pt_sp1)[1] = 20;
  (*pt_sp1)[2] = 3;
  space.copy_from(pt_sp1);
  space.copy_to(pt_sp2);
  BOOST_CHECK(space.equal_points(pt_sp1, pt_sp2));

  (*pt_sp1)[0] = -10;
  (*pt_sp1)[1] = -20;
  (*pt_sp1)[2] = -3;
  space.copy(pt_sp2, pt_sp1);
  BOOST_CHECK(space.equal_points(pt_sp1, pt_sp2));

  space.copy(pt_sp2, { 2, 5, 1 });
  std::vector<double> v(space.size());
  space.copy(v, pt_sp2);
  BOOST_CHECK(v[0] == 2 && v[1] == 5 && v[2] == 1);
}

BOOST_AUTO_TEST_CASE(test_space_bounds)
{
  mock::space3d_t test;
  prx::space_t& space = test.space;

  prx::space_point_t pt_sp1 = space.make_point();
  prx::space_point_t pt_sp2 = space.make_point();

  (*pt_sp1)[0] = (*pt_sp2)[0] = -100;
  (*pt_sp1)[1] = (*pt_sp2)[1] = -200;
  (*pt_sp1)[2] = (*pt_sp2)[2] = 0;

  space.copy_from(pt_sp2);
  space.set_bounds({ -10, -10, -3 }, { 10, 10, 3 });
  space.enforce_bounds(pt_sp1);

  BOOST_CHECK(space.satisfies_bounds(pt_sp1));
  BOOST_CHECK(!space.satisfies_bounds(pt_sp2));

  space.enforce_bounds();
  space.copy_to(pt_sp2);
  BOOST_CHECK(space.satisfies_bounds(pt_sp2));

  space.sample(pt_sp1);
  BOOST_CHECK(space.satisfies_bounds(pt_sp1));
}

BOOST_AUTO_TEST_CASE(test_space_norms)
{
  double x, y, theta;
  x = y = theta = 1;
  std::vector<double*> address_1 = { &x, &y, &theta };
  prx::space_t space_1("EER", address_1, "space_1");

  prx::space_point_t pt_sp1 = space_1.make_point();
  space_1.copy_to_point(pt_sp1);
  prx::space_point_t pt_sp2 = space_1.clone_point(pt_sp1);
  (*pt_sp1)[0] = 0;
  (*pt_sp1)[1] = -1;
  (*pt_sp1)[2] = 2;
  (*pt_sp2)[0] = 1;
  (*pt_sp2)[1] = -2;
  (*pt_sp2)[2] = 3;
  BOOST_CHECK(prx::space_t::l1_norm(pt_sp1) == 3);
  BOOST_CHECK(prx::space_t::l1_norm(pt_sp1, pt_sp2) == 3);
  BOOST_CHECK(prx::space_t::l2_norm(pt_sp1) == std::sqrt(1 + 4));
  BOOST_CHECK(prx::space_t::l2_norm(pt_sp1, pt_sp2) == std::sqrt(1 + 1 + 1));
  (*pt_sp1)[1] = 1;
  (*pt_sp2)[1] = 2;

  BOOST_CHECK(prx::space_t::lp_norm(pt_sp2, 1) == 6);
  BOOST_CHECK(prx::space_t::lp_norm(pt_sp2, 2) == std::sqrt(14));
  BOOST_CHECK(prx::space_t::lp_norm(pt_sp2, 3) == std::pow(6, 2.0 / 3.0));
  BOOST_CHECK(prx::space_t::lp_norm(pt_sp2, 4) == std::pow(2, 1.0 / 4) * std::sqrt(7));

  (*pt_sp1)[0] = 0;
  (*pt_sp1)[1] = 1;
  (*pt_sp1)[2] = 2;
  (*pt_sp2)[0] = 1;
  (*pt_sp2)[1] = 3;
  (*pt_sp2)[2] = 5;
  BOOST_CHECK(prx::space_t::lp_norm(pt_sp2, pt_sp1, 1) == 6);
  BOOST_CHECK(prx::space_t::lp_norm(pt_sp2, pt_sp1, 2) == std::sqrt(14));
  BOOST_CHECK(prx::space_t::lp_norm(pt_sp2, pt_sp1, 3) == std::pow(6, 2.0 / 3.0));
  BOOST_CHECK(prx::space_t::lp_norm(pt_sp2, pt_sp1, 4) == std::pow(2, 1.0 / 4) * std::sqrt(7));

  BOOST_CHECK(prx::space_t::euclidean_2d(pt_sp2, pt_sp1) == std::sqrt(1 + 4));
}

BOOST_AUTO_TEST_CASE(test_space_copy_to_and_copy_from_point)
{
  double x, y, theta;
  x = y = theta = 1;
  std::vector<double*> address_1 = { &x, &y, &theta };
  prx::space_t space_1("EER", address_1, "space_1");

  prx::space_point_t pt_sp1 = space_1.make_point();

  space_1.copy_to_point(pt_sp1);
  prx::space_point_t pt_sp2 = space_1.clone_point(pt_sp1);
  BOOST_CHECK(space_1.equal_points(pt_sp1, pt_sp2));
  (*pt_sp1)[0] = 10;
  (*pt_sp1)[1] = 20;
  (*pt_sp1)[2] = 3;
  space_1.copy_from(pt_sp1);
  space_1.copy_to(pt_sp2);
  BOOST_CHECK(space_1.equal_points(pt_sp1, pt_sp2));
  (*pt_sp1)[0] = -10;
  (*pt_sp1)[1] = -20;
  (*pt_sp1)[2] = -3;
  space_1.copy(pt_sp2, pt_sp1);

  BOOST_CHECK(space_1.equal_points(pt_sp1, pt_sp2));
}

BOOST_AUTO_TEST_CASE(test_space_copy)
{
  mock::space3d_t test;
  prx::space_t& space = test.space;

  prx::space_point_t pt_sp1 = space.make_point();
  prx::space_point_t pt_sp2 = space.make_point();

  // prx::space_point_t <- std::initializer_list
  space.copy(pt_sp1, { 2, 5, 1 });
  BOOST_CHECK((*pt_sp1)[0] == 2 && (*pt_sp1)[1] == 5 && (*pt_sp1)[2] == 1);

  // prx::space_point_t <- std::vector
  std::vector<double> v = { -2, -5, -1 };
  space.copy(pt_sp2, v);
  BOOST_CHECK((*pt_sp2)[0] == -2 && (*pt_sp2)[1] == -5 && (*pt_sp2)[2] == -1);

  // std::vector <- prx::space_point_t
  space.copy(v, pt_sp1);
  BOOST_CHECK(v[0] == 2 && v[1] == 5 && v[2] == 1);

  // prx::space_point_t <- prx::space_point_t
  BOOST_CHECK(!space.equal_points(pt_sp1, pt_sp2));
  space.copy(pt_sp1, pt_sp2);
  BOOST_CHECK(space.equal_points(pt_sp1, pt_sp2));
}

BOOST_AUTO_TEST_CASE(testing_copy_from_initializer_list)
{
  mock::space3d_t test;
  prx::space_t& space = test.space;

  prx::space_point_t pt_sp1 = space.make_point();
  prx::space_point_t pt_sp2 = space.make_point();

  (*pt_sp1)[0] = 10;
  (*pt_sp1)[1] = 20;
  (*pt_sp1)[2] = 3;

  space.copy_from({ 10, 20, 3 });
  space.copy_to(pt_sp2);
  BOOST_CHECK(space.equal_points(pt_sp1, pt_sp2));
}
