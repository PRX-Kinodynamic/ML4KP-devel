#define BOOST_AUTO_TEST_MAIN spaces_test

#include <chrono>
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
struct space_quat_t
{
  space_quat_t()
    : _quat(Eigen::Quaterniond::Identity())
    , _deriv_quat(Eigen::Quaterniond::Identity())
    , _address({ &_quat.w(), &_quat.x(), &_quat.y(), &_quat.z() })
    , _derivative_address({ &_deriv_quat.w(), &_deriv_quat.x(), &_deriv_quat.y(), &_deriv_quat.z() })
    , _space("QQQQ", _address, "space_quat_test")
    , _derivative_space("QQQQ", _derivative_address, "space_deriv_quat_test")

  {
  }
  Eigen::Quaterniond _quat;
  Eigen::Quaterniond _deriv_quat;
  std::vector<double*> _address;
  std::vector<double*> _derivative_address;

  prx::space_t _space;
  prx::space_t _derivative_space;
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

BOOST_AUTO_TEST_CASE(comparing_speed_of_copy_point_vs_copy)
{
  mock::space3d_t test;
  prx::space_t& space = test.space;

  prx::space_point_t pt_from = space.make_point();
  prx::space_point_t pt_to = space.make_point();

  const std::size_t total_copies{ 100'000 };

  auto start_copy_point = std::chrono::steady_clock::now();
  for (std::size_t i = 0; i < total_copies; ++i)
  {
    space.sample(pt_from);
    space.copy_point(pt_to, pt_from);
  }
  auto end_copy_point = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_copy_point{ end_copy_point - start_copy_point };

  auto start_copy = std::chrono::steady_clock::now();
  for (std::size_t i = 0; i < total_copies; ++i)
  {
    space.sample(pt_from);
    space.copy(pt_to, pt_from);
  }
  auto end_copy = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_copy{ end_copy - start_copy };

  std::cout << "Results:\n";
  std::cout << "Copy_point: " << elapsed_copy_point.count() << "s\n";
  std::cout << "Copy: " << elapsed_copy.count() << "s\n";
}

BOOST_AUTO_TEST_CASE(topology_quaternion_enforce_bounds)
{
  mock::space_quat_t test;
  prx::space_t& space{ test._space };
  prx::space_point_t pt{ space.make_point() };

  space[0] = 2;
  space[1] = 2;
  space[2] = 2;
  space[3] = 2;
  space.enforce_bounds();

  const double norm{ std::sqrt(std::pow(space[0], 2) + std::pow(space[1], 2) + std::pow(space[2], 2) +
                               std::pow(space[3], 2)) };
  BOOST_REQUIRE_CLOSE(norm, 1, 1e-5);

  pt->at(0) = 10;
  pt->at(1) = 100;
  pt->at(2) = 1;
  pt->at(3) = 500;

  space.enforce_bounds(pt);
  BOOST_REQUIRE_CLOSE(Vec(pt).norm(), 1, 1e-5);
}

BOOST_AUTO_TEST_CASE(integration_of_topology_quaternion_rotate_on_x)
{
  const double dt{ 0.01 };
  mock::space_quat_t test;
  prx::space_t& space{ test._space };
  prx::space_t& dt_space{ test._derivative_space };
  prx::space_point_t pt{ space.make_point() };

  Eigen::Quaterniond aux_quat{ Eigen::Quaterniond::Identity() };

  space[0] = 1;
  space[1] = 0;
  space[2] = 0;
  space[3] = 0;

  // Rotating on x
  Eigen::Quaterniond quat_omega_0{ 0, 1, 0, 0 };  // [0, \omega]

  for (int i = 0; i < 100; ++i)
  {
    aux_quat.coeffs() = 0.5 * quat_omega_0.coeffs();
    test._deriv_quat = aux_quat * test._quat;
    test._deriv_quat.normalize();
    space.integrate(&dt_space, dt);
  }
  space.copy_to(pt);
  BOOST_REQUIRE_CLOSE(Vec(pt).norm(), 1, 1e-5);
  BOOST_REQUIRE(pt->at(0) > 0);
  BOOST_REQUIRE(pt->at(1) > 0);
  BOOST_REQUIRE_CLOSE(pt->at(2), 0, 1e-5);
  BOOST_REQUIRE_CLOSE(pt->at(3), 0, 1e-5);
}

BOOST_AUTO_TEST_CASE(integration_of_topology_quaternion_rotate_on_y)
{
  const double dt{ 0.01 };
  mock::space_quat_t test;
  prx::space_t& space{ test._space };
  prx::space_t& dt_space{ test._derivative_space };
  prx::space_point_t pt{ space.make_point() };

  Eigen::Quaterniond aux_quat{ Eigen::Quaterniond::Identity() };

  space[0] = 1;
  space[1] = 0;
  space[2] = 0;
  space[3] = 0;

  // Rotating on y
  Eigen::Quaterniond quat_omega_0{ 0, 0, 1, 0 };  // [0, \omega]

  for (int i = 0; i < 100; ++i)
  {
    aux_quat.coeffs() = 0.5 * quat_omega_0.coeffs();
    test._deriv_quat = aux_quat * test._quat;
    test._deriv_quat.normalize();
    space.integrate(&dt_space, dt);
  }
  space.copy_to(pt);
  BOOST_REQUIRE_CLOSE(Vec(pt).norm(), 1, 1e-5);
  BOOST_REQUIRE(pt->at(0) > 0);
  BOOST_REQUIRE_CLOSE(pt->at(1), 0, 1e-5);
  BOOST_REQUIRE(pt->at(2) > 0);
  BOOST_REQUIRE_CLOSE(pt->at(3), 0, 1e-5);
}

BOOST_AUTO_TEST_CASE(integration_of_topology_quaternion_rotate_on_z)
{
  const double dt{ 0.01 };
  mock::space_quat_t test;
  prx::space_t& space{ test._space };
  prx::space_t& dt_space{ test._derivative_space };
  prx::space_point_t pt{ space.make_point() };

  Eigen::Quaterniond aux_quat{ Eigen::Quaterniond::Identity() };

  space[0] = 1;
  space[1] = 0;
  space[2] = 0;
  space[3] = 0;

  // Rotating on z
  Eigen::Quaterniond quat_omega_0{ 0, 0, 0, 1 };  // [0, \omega]

  for (int i = 0; i < 100; ++i)
  {
    aux_quat.coeffs() = 0.5 * quat_omega_0.coeffs();
    test._deriv_quat = aux_quat * test._quat;
    test._deriv_quat.normalize();
    space.integrate(&dt_space, dt);
  }
  space.copy_to(pt);
  BOOST_REQUIRE_CLOSE(Vec(pt).norm(), 1, 1e-5);
  BOOST_REQUIRE(pt->at(0) > 0);
  BOOST_REQUIRE_CLOSE(pt->at(1), 0, 1e-5);
  BOOST_REQUIRE_CLOSE(pt->at(2), 0, 1e-5);
  BOOST_REQUIRE(pt->at(3) > 0);
}

BOOST_AUTO_TEST_CASE(interpolate_of_topology_quaternion)
{
  mock::space_quat_t test;
  prx::space_t& space{ test._space };
  prx::space_point_t pt0{ space.make_point() };
  prx::space_point_t pt1{ space.make_point() };
  prx::space_point_t result0{ space.make_point() };
  prx::space_point_t result1{ space.make_point() };

  const double t0{ 0 };
  const double t1{ 1 };
  space.interpolate(pt0, pt1, t0, result0);
  space.interpolate(pt0, pt1, t1, result1);
  BOOST_CHECK(space.equal_points(pt0, result0));
  BOOST_CHECK(space.equal_points(pt1, result1));
}