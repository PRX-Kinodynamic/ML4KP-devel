#define BOOST_AUTO_TEST_MAIN lie_integrator_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/defs.hpp"
#include "prx/factor_graphs/factors/obstacle_factor.hpp"
#include "prx/utilities/geometry/basic_geoms/sphere.hpp"

#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/SOn.h>

namespace mock
{
using State = Eigen::Vector2d;
using Rotation = Eigen::Matrix3d;
using Translation = Eigen::Vector3d;

struct configuration_from_state
{
  void operator()(Rotation& rotation, Translation& translation, const State& state)
  {
    rotation = Rotation::Identity();
    translation.head(2) = state;
    translation[2] = 0;
  }

  void operator()(const State& state, const Translation& translation, Eigen::MatrixXd& H)
  {
    H = Eigen::Matrix2d::Identity();
    H.diagonal() = translation.head(2);
  }
};

}  // namespace mock

BOOST_AUTO_TEST_CASE(obstacle_factor_in_collision_test)
{
  using State = Eigen::Vector2d;
  using Rotation = Eigen::Matrix3d;
  using Translation = Eigen::Vector3d;
  using CollisionInfoPtr = std::shared_ptr<prx::fg::collision_info_t>;

  std::vector<double> sphere_params({ 1 });
  const Rotation rotation{ Rotation::Identity() };
  const Translation translation{ Translation::Zero() };
  CollisionInfoPtr robot{ std::make_shared<prx::fg::collision_info_t>(prx::geometry_type_t::SPHERE, sphere_params,
                                                                      rotation, translation) };
  CollisionInfoPtr obstacle{ std::make_shared<prx::fg::collision_info_t>(prx::geometry_type_t::SPHERE, sphere_params,
                                                                         rotation, translation) };

  gtsam::Key key{ 0 };
  prx::fg::obstacle_factor_t<State, mock::configuration_from_state> factor(obstacle, robot, key);
  const State x0(0, 0);
  const State x1(0, 1);
  const State x2(1, 1);
  const State x3(10, 10);
  // PRX_DBG_VARS(factor.in_collision(x0));
  // PRX_DBG_VARS(factor.in_collision(x1));
  // PRX_DBG_VARS(factor.in_collision(x2));
  // PRX_DBG_VARS(factor.in_collision(x3));
  BOOST_REQUIRE(factor.in_collision(x0));
  BOOST_REQUIRE(factor.in_collision(x1));
  BOOST_REQUIRE(factor.in_collision(x2));
  BOOST_REQUIRE(not factor.in_collision(x3));
}

BOOST_AUTO_TEST_CASE(obstacle_factor_box_sphere_in_collision_test)
{
  using State = Eigen::Vector2d;
  using Rotation = Eigen::Matrix3d;
  using Translation = Eigen::Vector3d;
  using CollisionInfoPtr = std::shared_ptr<prx::fg::collision_info_t>;

  std::vector<double> box_params({ 30, 0.5, 0.5 });
  std::vector<double> sphere_params({ 1 });
  const Rotation rotation{ Rotation::Identity() };
  const Translation translation{ Translation::Zero() };
  CollisionInfoPtr robot{ std::make_shared<prx::fg::collision_info_t>(prx::geometry_type_t::SPHERE, sphere_params,
                                                                      rotation, translation) };
  CollisionInfoPtr obstacle{ std::make_shared<prx::fg::collision_info_t>(prx::geometry_type_t::BOX, box_params,
                                                                         rotation, translation) };

  gtsam::Key key{ 0 };
  prx::fg::obstacle_factor_t<State, mock::configuration_from_state> factor(obstacle, robot, key);
  const State x0(0, 0);
  const State x1(10, 0);
  // const State x3(10, 10);
  // PRX_DBG_VARS(factor.in_collision(x0));
  // PRX_DBG_VARS(factor.in_collision(x1));
  // PRX_DBG_VARS(factor.in_collision(x2));
  // PRX_DBG_VARS(factor.in_collision(x3));
  BOOST_REQUIRE(factor.in_collision(x0));
  BOOST_REQUIRE(factor.in_collision(x1));
  // BOOST_REQUIRE(factor.in_collision(x2));
  // BOOST_REQUIRE(not factor.in_collision(x3));
}

BOOST_AUTO_TEST_CASE(obstacle_factor_distance_test)
{
  using State = Eigen::Vector2d;
  using Rotation = Eigen::Matrix3d;
  using Translation = Eigen::Vector3d;
  using CollisionInfoPtr = std::shared_ptr<prx::fg::collision_info_t>;

  std::vector<double> sphere_params({ 1 });
  const Rotation rotation{ Rotation::Identity() };
  const Translation translation{ Translation::Zero() };
  CollisionInfoPtr robot{ std::make_shared<prx::fg::collision_info_t>(prx::geometry_type_t::SPHERE, sphere_params,
                                                                      rotation, translation) };
  CollisionInfoPtr obstacle{ std::make_shared<prx::fg::collision_info_t>(prx::geometry_type_t::SPHERE, sphere_params,
                                                                         rotation, translation) };

  gtsam::Key key{ 0 };
  prx::fg::obstacle_factor_t<State, mock::configuration_from_state> factor(obstacle, robot, key);
  const State x0(0, 0);
  const State x1(0, 1);
  const State x2(1, 1);
  const State x3(10, 10);

  Eigen::Vector3d closest_point0{ Eigen::Vector3d::Zero() };
  Eigen::Vector3d closest_point1{ Eigen::Vector3d::Zero() };
  Eigen::Vector3d closest_point2{ Eigen::Vector3d::Zero() };
  Eigen::Vector3d closest_point3{ Eigen::Vector3d::Zero() };

  Eigen::Vector3d p2{ Eigen::Vector3d::Zero() };

  // const double dist0{ factor.distances(x0, closest_point0) };
  // const double dist1{ factor.distances(x1, closest_point1) };
  // const double dist2{ factor.distances(x2, closest_point2) };
  const double dist3{ factor.distances(x3, closest_point3, p2) };

  // PRX_DBG_VARS(dist0, closest_point0);
  // PRX_DBG_VARS(dist1, closest_point1);
  // PRX_DBG_VARS(dist2, closest_point2);
  PRX_DBG_VARS(dist3, closest_point3);
  // PRX_DBG_VARS(factor.in_collision(x1));
  // PRX_DBG_VARS(factor.in_collision(x2));
  // PRX_DBG_VARS(factor.in_collision(x3));
  // BOOST_REQUIRE(factor.in_collision(x0));
  // BOOST_REQUIRE(factor.in_collision(x1));
  // BOOST_REQUIRE(factor.in_collision(x2));
  // BOOST_REQUIRE(not factor.in_collision(x3));
}