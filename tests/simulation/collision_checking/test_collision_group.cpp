#define BOOST_AUTO_TEST_MAIN integrators
#include <string>
#include <boost/test/unit_test.hpp>

#include "prx/simulation/collision_checking/collision_group.hpp"
#include "prx/utilities/geometry/basic_geoms/sphere.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/geometry/basic_geoms/box.hpp"
namespace mock
{
class pt2d_t : public prx::plant_t
{
public:
  pt2d_t(const std::string& path) : prx::plant_t(path), x(0), y(0)
  {
    state_memory = { &x, &y };
    state_space = new prx::space_t("EE", state_memory, "XY");
    state_space->set_bounds({ -std::numeric_limits<double>::max(), -std::numeric_limits<double>::max() },
                            { std::numeric_limits<double>::max(), std::numeric_limits<double>::max() });

    geometries["body"] = std::make_shared<prx::geometry_t>(prx::geometry_type_t::SPHERE);
    geometries["body"]->initialize_geometry({ 1.0 });
    geometries["body"]->generate_collision_geometry();
    geometries["body"]->set_visualization_color("0x00ff00");
    configurations["body"] = std::make_shared<prx::transform_t>();
    configurations["body"]->setIdentity();
  }
  virtual ~pt2d_t(){};

  virtual void propagate(const double simulation_step) override final{};

  virtual void update_configuration() override
  {
    auto body = configurations["body"];
    body->setIdentity();
    body->translation() = (prx::vector_t(x, y, 0.5));
  }

protected:
  virtual void compute_derivative() override final{};

  double x, y;
};

}  // namespace mock

BOOST_AUTO_TEST_CASE(test_empty_environment)
{
  std::vector<std::shared_ptr<prx::movable_object_t> > obstacle_list;

  prx::system_ptr_t ptr = std::make_shared<mock::pt2d_t>("pt");
  prx::collision_group_t cg{ { ptr }, obstacle_list };

  const bool expected{ false };
  const bool result{ cg.in_collision() };
  BOOST_CHECK_MESSAGE(expected == result, EXPECTED_GOT(expected, result));
}

BOOST_AUTO_TEST_CASE(test_collision)
{
  std::vector<std::shared_ptr<prx::movable_object_t> > obstacle_list;
  std::vector<std::string> obstacles_names{ "Obstacle0" };

  prx::transform_t obstacle_pose;
  obstacle_pose.setIdentity();
  obstacle_pose.translation() = (prx::vector_t(0.1, 0., 0.5));
  obstacle_list.push_back(
      prx::create_obstacle(new prx::sphere_t(obstacles_names.back(), 1.0, obstacle_pose, "0xff0000")));

  prx::system_ptr_t ptr = std::make_shared<mock::pt2d_t>("pt");
  prx::collision_group_t cg{ { ptr }, { obstacle_list } };

  const bool expected{ true };
  const bool result{ cg.in_collision() };
  BOOST_CHECK_MESSAGE(expected == result, EXPECTED_GOT(expected, result));
}

BOOST_AUTO_TEST_CASE(test_distances)
{
  std::vector<std::shared_ptr<prx::movable_object_t> > obstacle_list;
  std::vector<std::string> obstacles_names{ "Obstacle0" };

  prx::transform_t obstacle_pose;
  obstacle_pose.setIdentity();
  // Each sphere is of rad 1.0, putting the sphere at 2.1 means there is a 0.1 distance between the two
  obstacle_pose.translation() = (prx::vector_t(2.1, 0., 0.5));
  obstacle_list.push_back(
      prx::create_obstacle(new prx::sphere_t(obstacles_names.back(), 1.0, obstacle_pose, "0xff0000")));

  prx::system_ptr_t ptr = std::make_shared<mock::pt2d_t>("pt");
  prx::collision_group_t cg{ { ptr }, { obstacle_list } };

  const bool expected{ false };
  const bool result{ cg.in_collision() };
  BOOST_REQUIRE_MESSAGE(expected == result, EXPECTED_GOT(expected, result));

  const prx::collision_group_t::pqp_distance_t dist{ cg.get_distances() };

  const std::size_t expected_distances{ 1 };
  const std::size_t result_distances{ dist.distances.size() };
  BOOST_REQUIRE_MESSAGE(expected_distances == result_distances, EXPECTED_GOT(expected_distances, result_distances));

  const double expected_distance{ 0.1 };
  const double result_distance{ dist.distances[0] };
  const double epsilon{ 1e-5 };
  BOOST_REQUIRE_MESSAGE(std::abs(expected_distance - result_distance) < epsilon,
                        EXPECTED_GOT(expected_distance, result_distance));

  // The expected closest points at the center of each sphere, but at 1.0 and 1.1 as the spheres are at (0,0) and
  // (2.1,0) with rad 1
  const double expected_x_plant{ 1.0 };
  const double expected_x_obstacle{ 1.1 };
  const double expected_y{ 0.0 };
  const double expected_z{ 0.5 };

  const double result_x_plant{ dist.closest_points[0].first[0] };
  const double result_y_plant{ dist.closest_points[0].first[1] };
  const double result_z_plant{ dist.closest_points[0].first[2] };
  const double result_x_obstacle{ dist.closest_points[0].second[0] };
  const double result_y_obstacle{ dist.closest_points[0].second[1] };
  const double result_z_obstacle{ dist.closest_points[0].second[2] };

  BOOST_REQUIRE_MESSAGE(std::abs(expected_x_plant - result_x_plant) < epsilon,
                        EXPECTED_GOT(expected_x_plant, result_x_plant));
  BOOST_REQUIRE_MESSAGE(std::abs(expected_y - result_y_plant) < epsilon, EXPECTED_GOT(expected_y, result_y_plant));
  BOOST_REQUIRE_MESSAGE(std::abs(expected_z - result_z_plant) < epsilon, EXPECTED_GOT(expected_z, result_z_plant));

  BOOST_REQUIRE_MESSAGE(std::abs(expected_x_obstacle - result_x_obstacle) < epsilon,
                        EXPECTED_GOT(expected_x_obstacle, result_x_obstacle));
  BOOST_REQUIRE_MESSAGE(std::abs(expected_y - result_y_obstacle) < epsilon,
                        EXPECTED_GOT(expected_y, result_y_obstacle));
  BOOST_REQUIRE_MESSAGE(std::abs(expected_z - result_z_obstacle) < epsilon,
                        EXPECTED_GOT(expected_z, result_z_obstacle));
}
