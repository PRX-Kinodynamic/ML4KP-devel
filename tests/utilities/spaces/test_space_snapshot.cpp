#define BOOST_AUTO_TEST_MAIN spaces_snapshot_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/spaces/space_snapshot.hpp"
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

BOOST_AUTO_TEST_CASE(test_space_makes_points_correctly)
{
  mock::space3d_t test;
  prx::space_t& space{ test.space };
  prx::space_point_t pt_sp1 = space.make_point();

  space.copy_to_point(pt_sp1);

  BOOST_CHECK((*pt_sp1)[0] == pt_sp1->at(0));
  BOOST_CHECK((*pt_sp1)[1] == pt_sp1->at(1));
  BOOST_CHECK((*pt_sp1)[2] == pt_sp1->at(2));
  BOOST_CHECK(pt_sp1->size() == space.get_dimension());
}

BOOST_AUTO_TEST_CASE(test_space_snapshot_arithmetic_operations_are_correct)
{
  mock::space3d_t test;

  prx::space_point_t ones = test.space.make_point();
  prx::space_point_t pt_sp = test.space.make_point();

  ones->at(0) = 1;
  ones->at(1) = 1;
  ones->at(2) = 1;

  pt_sp->at(0) = 1;
  pt_sp->at(1) = 2;
  pt_sp->at(2) = 3;

  // Checking add
  pt_sp->add(ones);
  BOOST_CHECK((*pt_sp)[0] == 2);
  BOOST_CHECK((*pt_sp)[1] == 3);
  BOOST_CHECK((*pt_sp)[2] == 4);

  // Checking multiply
  pt_sp->multiply(2);
  BOOST_CHECK((*pt_sp)[0] == 4);
  BOOST_CHECK((*pt_sp)[1] == 6);
  BOOST_CHECK((*pt_sp)[2] == 8);

  // Checking add multiply: yn <- yn + sclr * pt
  pt_sp->add_multiply(2, ones);
  BOOST_CHECK((*pt_sp)[0] == (4 + 2 * 1));
  BOOST_CHECK((*pt_sp)[1] == (6 + 2 * 1));
  BOOST_CHECK((*pt_sp)[2] == (8 + 2 * 1));
}

BOOST_AUTO_TEST_CASE(test_space_snapshot_as_eigen_vector)
{
  mock::space3d_t test;

  prx::space_point_t ones = test.space.make_point();
  prx::space_point_t pt_map = test.space.make_point();
  prx::space_point_t pt_copy = test.space.make_point();

  // Assign (copy) from Eigen::Vector
  Vec(ones) = Eigen::Vector3d::Ones();
  BOOST_REQUIRE((*ones)[0] == 1);
  BOOST_REQUIRE((*ones)[1] == 1);
  BOOST_REQUIRE((*ones)[2] == 1);

  // Assign (copy) from Eigen::Vector (custom)
  Vec(pt_map) = Eigen::Vector3d(1, 2, 3);
  BOOST_REQUIRE((*pt_map)[0] == 1);
  BOOST_REQUIRE((*pt_map)[1] == 2);
  BOOST_REQUIRE((*pt_map)[2] == 3);

  // Use Eigen operations:
  // Addition:
  Vec(pt_map) += Vec(ones);  // in-place modification
  const Eigen::Vector3d expected(2, 3, 4);
  BOOST_REQUIRE(Vec(pt_map) == expected);
  // Norm (reduction operation)
  BOOST_REQUIRE(Vec(pt_map).norm() == expected.norm());
  // Normilize a vector:
  Vec(pt_map).normalize();                                           // in-place modification
  const Eigen::Vector3d expected_normalized(expected.normalized());  // Notice: normalize vs normalizeD
  BOOST_REQUIRE(Vec(pt_map) == expected_normalized);

  // This creates a copy: Not the same memory
  Eigen::Vector3d vec_map(Vec(ones));
  BOOST_REQUIRE(vec_map == Eigen::Vector3d::Ones());
  vec_map += Eigen::Vector3d::Ones();  // This changes 'vec_map' but not 'ones';
  BOOST_REQUIRE(Vec(ones) == Eigen::Vector3d::Ones());
  BOOST_REQUIRE(vec_map == (2 * Eigen::Vector3d::Ones()));

  // Equivalent to:
  Eigen::Vector3d vec_as(ones->as<>());
  BOOST_REQUIRE(vec_as == Eigen::Vector3d::Ones());
  vec_as += Eigen::Vector3d::Ones();  // This changes 'vec_as' but not 'ones';
  BOOST_REQUIRE(Vec(ones) == Eigen::Vector3d::Ones());
  BOOST_REQUIRE(vec_as == (2 * Eigen::Vector3d::Ones()));

  // Which can be use for other classes:
  Eigen::Array3d arr_as(ones->as<Eigen::Array3d>());
  BOOST_REQUIRE(arr_as[0] == 1);
  BOOST_REQUIRE(arr_as[1] == 1);
  BOOST_REQUIRE(arr_as[2] == 1);
}

BOOST_AUTO_TEST_CASE(test_space_snapshot_as_string)
{
  prx::constants::separating_value = ' ';
  prx::constants::precision = 0;
  mock::space3d_t test;

  prx::space_point_t pt = test.space.make_point();
  Vec(pt) = Eigen::Vector3d(1, 2, 3);
  const std::string result_0{ *pt };
  BOOST_REQUIRE(result_0 == "1 2 3 ");

  prx::constants::precision = 1;
  const std::string result_1{ *pt };
  BOOST_REQUIRE(result_1 == "1.0 2.0 3.0 ");
}

BOOST_AUTO_TEST_CASE(test_space_iterators)
{
  prx::constants::separating_value = ' ';
  mock::space3d_t test;

  prx::space_point_t pt = test.space.make_point();
  Vec(pt) = Eigen::Vector3d(1, 2, 3);
  double idx{ 1 };
  for (double e : *pt)
  {
    BOOST_REQUIRE(e == idx);
    idx += 1;
  }
}

BOOST_AUTO_TEST_CASE(space_snapshot_init_params)
{
  mock::space3d_t test;
  prx::space_point_t pt{ test.space.make_point() };

  prx::param_loader params{};
  std::vector<double> values{ { 0.5, 0.5, 0.5 } };
  params.set<std::vector<double>>(values);

  pt->init(params);

  BOOST_REQUIRE_MESSAGE(values[0] == pt->at(0), EXPECTED_GOT(values[0], pt->at(0)));
  BOOST_REQUIRE_MESSAGE(values[1] == pt->at(1), EXPECTED_GOT(values[1], pt->at(1)));
  BOOST_REQUIRE_MESSAGE(values[2] == pt->at(2), EXPECTED_GOT(values[2], pt->at(2)));
  // BOOST_CHECK(space.equal_points(pt1, result1));
  // BOOST_CHECK_MESSAGE(space.equal_points(pt_half, expected_half), EXPECTED_GOT(expected_half, pt_half));
}