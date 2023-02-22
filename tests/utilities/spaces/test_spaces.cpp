#define BOOST_AUTO_TEST_MAIN spaces_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/spaces/space.hpp"

BOOST_AUTO_TEST_CASE(test_space_is_built_correctly)
{
  double x, y, theta;
  x = y = theta = 1;
  std::vector<double*> address_1 = { &x, &y, &theta };
  prx::space_t space_1("EER", address_1, "space_1");

  prx::space_point_t pt_sp1 = space_1.make_point();

  space_1.copy_to_point(pt_sp1);

  BOOST_CHECK((*pt_sp1)[0] == pt_sp1->at(0));
  BOOST_CHECK((*pt_sp1)[1] == pt_sp1->at(1));
  BOOST_CHECK((*pt_sp1)[2] == pt_sp1->at(2));
  BOOST_CHECK(pt_sp1->get_dim() == space_1.get_dimension());
}
BOOST_AUTO_TEST_CASE(test_space_point_to_string)
{
  double x, y, theta;
  x = y = theta = 1;
  std::vector<double*> address_1 = { &x, &y, &theta };
  prx::space_t space_1("EER", address_1, "space_1");

  prx::space_point_t pt_sp1 = space_1.make_point();

  const std::vector<double> point = { 0, 1, 2 };
  const std::string expected_string = "0 1 2 ";

  space_1.copy(pt_sp1, point);

  const std::string casted_value{ static_cast<std::string>(*pt_sp1) };
  BOOST_CHECK_MESSAGE(expected_string == casted_value, "Expected: " << expected_string << ". Got: " << casted_value);
}
BOOST_AUTO_TEST_CASE(test_space_operations_are_correct)
{
  double x, y, theta;
  x = y = theta = 1;
  std::vector<double*> address_1 = { &x, &y, &theta };
  prx::space_t space_1("EER", address_1, "space_1");

  prx::space_point_t pt_sp1 = space_1.make_point();
  space_1.copy_to_point(pt_sp1);
  // Checking add
  pt_sp1->add(pt_sp1);
  BOOST_CHECK((*pt_sp1)[0] == 2);
  BOOST_CHECK((*pt_sp1)[1] == 2);
  BOOST_CHECK((*pt_sp1)[2] == 2);

  // Checking multiply
  pt_sp1->multiply(2);
  BOOST_CHECK((*pt_sp1)[0] == 4);
  BOOST_CHECK((*pt_sp1)[1] == 4);
  BOOST_CHECK((*pt_sp1)[2] == 4);

  // Checking add multiply
  pt_sp1->add_multiply(2, pt_sp1);
  BOOST_CHECK((*pt_sp1)[0] == 12);
  BOOST_CHECK((*pt_sp1)[1] == 12);
  BOOST_CHECK((*pt_sp1)[2] == 12);
}
BOOST_AUTO_TEST_CASE(test_space_copy_and_clone_point)
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
  space_1.copy_from_point(pt_sp1);
  space_1.copy_to_point(pt_sp2);
  BOOST_CHECK(space_1.equal_points(pt_sp1, pt_sp2));
  (*pt_sp1)[0] = -10;
  (*pt_sp1)[1] = -20;
  (*pt_sp1)[2] = -3;
  space_1.copy_point(pt_sp2, pt_sp1);
  BOOST_CHECK(space_1.equal_points(pt_sp1, pt_sp2));

  space_1.copy(pt_sp2, { 2, 5, 1 });
  std::vector<double> v(space_1.size());
  space_1.copy(v, pt_sp2);
  BOOST_CHECK(v[0] == 2 && v[1] == 5 && v[2] == 1);
}
BOOST_AUTO_TEST_CASE(test_space_bounds)
{
  double x, y, theta;
  x = y = theta = 1;
  std::vector<double*> address_1 = { &x, &y, &theta };
  prx::space_t space_1("EER", address_1, "space_1");

  prx::space_point_t pt_sp1 = space_1.make_point();
  space_1.copy_to_point(pt_sp1);

  prx::space_point_t pt_sp2 = space_1.clone_point(pt_sp1);
  (*pt_sp1)[0] = (*pt_sp2)[0] = -100;
  (*pt_sp1)[1] = (*pt_sp2)[1] = -200;
  (*pt_sp1)[2] = (*pt_sp2)[2] = 0;
  BOOST_CHECK(!space_1.satisfies_bounds(pt_sp1));
  space_1.copy_from_point(pt_sp2);
  space_1.set_bounds({ -10, -10, -3 }, { 10, 10, 3 });
  space_1.enforce_bounds(pt_sp1);
  BOOST_CHECK(space_1.satisfies_bounds(pt_sp1));
  space_1.enforce_bounds();
  space_1.copy_to_point(pt_sp2);
  BOOST_CHECK(space_1.satisfies_bounds(pt_sp2));
  space_1.sample(pt_sp1);
  BOOST_CHECK(space_1.satisfies_bounds(pt_sp1));
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
  double x, y, theta;
  x = y = theta = 1;
  std::vector<double*> address_1 = { &x, &y, &theta };
  prx::space_t space_1("EER", address_1, "space_1");

  prx::space_point_t pt_sp1 = space_1.make_point();
  prx::space_point_t pt_sp2 = space_1.make_point();

  // prx::space_point_t <- std::initializer_list
  space_1.copy(pt_sp1, { 2, 5, 1 });
  BOOST_CHECK((*pt_sp1)[0] == 2 && (*pt_sp1)[1] == 5 && (*pt_sp1)[2] == 1);

  // prx::space_point_t <- std::vector
  std::vector<double> v = { -2, -5, -1 };
  space_1.copy(pt_sp2, v);
  BOOST_CHECK((*pt_sp2)[0] == -2 && (*pt_sp2)[1] == -5 && (*pt_sp2)[2] == -1);

  // std::vector <- prx::space_point_t
  space_1.copy(v, pt_sp1);
  BOOST_CHECK(v[0] == 2 && v[1] == 5 && v[2] == 1);

  // prx::space_point_t <- prx::space_point_t
  BOOST_CHECK(!space_1.equal_points(pt_sp1, pt_sp2));
  space_1.copy(pt_sp1, pt_sp2);
  BOOST_CHECK(space_1.equal_points(pt_sp1, pt_sp2));
}

BOOST_AUTO_TEST_CASE(test_space_point_vector_share_memory)
{
  double x, y, theta;
  x = y = theta = 1;
  std::vector<double*> address_1 = { &x, &y, &theta };
  prx::space_t space_1("EER", address_1, "space_1");

  prx::space_point_t pt_sp1 = space_1.make_point();

  space_1.copy(pt_sp1, { 1, 2, 3 });

  BOOST_REQUIRE(pt_sp1->vector()[0] == (*pt_sp1)[0]);
  BOOST_REQUIRE(pt_sp1->vector()[1] == (*pt_sp1)[1]);
  BOOST_REQUIRE(pt_sp1->vector()[2] == (*pt_sp1)[2]);

  pt_sp1->vector()[0] = 3;
  pt_sp1->vector()[1] = 2;
  pt_sp1->vector()[2] = 1;

  BOOST_REQUIRE(pt_sp1->vector()[0] == (*pt_sp1)[0]);
  BOOST_REQUIRE(pt_sp1->vector()[1] == (*pt_sp1)[1]);
  BOOST_REQUIRE(pt_sp1->vector()[2] == (*pt_sp1)[2]);

  pt_sp1->vector() = Eigen::Vector3d::Ones() + Eigen::Vector3d::Ones();
  BOOST_REQUIRE((*pt_sp1)[0] == 2);
  BOOST_REQUIRE((*pt_sp1)[1] == 2);
  BOOST_REQUIRE((*pt_sp1)[2] == 2);
  BOOST_REQUIRE(pt_sp1->vector() == Eigen::Vector3d(2, 2, 2));
}