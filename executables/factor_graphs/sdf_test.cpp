#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/type_conversions.hpp"
#include "prx/factor_graphs/lie_groups/se3.hpp"

#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/rrt_star.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/simulation/collision_checking/collision_group.hpp"

#include "prx/simulation/plants/first_order_free_body.hpp"
#include "prx/factor_graphs/factors/obstacle_factor.hpp"

using State = Eigen::Vector2d;
using Rotation = Eigen::Matrix3d;
using Translation = Eigen::Vector3d;
using CollisionInfoPtr = std::shared_ptr<prx::fg::collision_info_t>;

struct configuration_from_state
{
  void operator()(Rotation& rotation, Translation& translation, const State& state)
  {
    rotation = Rotation::Identity();
    translation[0] = state[0];
    translation[1] = state[1];
    translation[2] = 0;
  }
  void operator()(Eigen::MatrixXd& H, const Translation& translation)
  {
    H = Eigen::Matrix2d::Identity();
    H.diagonal() = -translation.head(2);
  }
};

int main(int argc, char* argv[])
{
  using ObstacleFactor = prx::fg::obstacle_factor_t<State, configuration_from_state>;

  auto obstacles = prx::load_obstacles("environments/simple_obstacle.yaml");
  std::vector<std::shared_ptr<prx::movable_object_t>> obstacle_list{ obstacles.second };
  std::vector<std::string> obstacle_names{ obstacles.first };

  std::vector<double> sphere_params({ 1 });
  const Rotation rotation{ Rotation::Identity() };
  const Translation translation{ Translation::Zero() };
  CollisionInfoPtr robot{ std::make_shared<prx::fg::collision_info_t>(prx::geometry_type_t::SPHERE, sphere_params,
                                                                      rotation, translation) };
  PRX_DBG_VARS(obstacle_list.size());

  std::vector<std::shared_ptr<ObstacleFactor>> factors;

  gtsam::Key kr(0);
  gtsam::Values values;
  for (auto obstacle : obstacle_list)
  {
    prx::movable_object_t::Geometries geometries{ obstacle->get_geometries() };
    prx::movable_object_t::Configurations configurations{ obstacle->get_configurations() };

    const std::size_t total_geoms{ geometries.size() };

    for (int i = 0; i < total_geoms; ++i)
    {
      std::shared_ptr<prx::geometry_t> g{ geometries[i].second };
      std::shared_ptr<prx::transform_t> tf{ configurations[i].second };

      const prx::geometry_type_t g_type{ g->get_geometry_type() };
      const std::vector<double> g_params{ g->get_geometry_params() };

      const Rotation rot{ tf->rotation() };
      const Translation t{ tf->translation() };

      // std::cout << static_cast<std::underlying_type<prx::geometry_type_t>::type>(g_type) << std::endl;
      PRX_DBG_VARS(g_params, t.transpose());
      // PRX_DBG_VARS(rot);

      CollisionInfoPtr obstacle{ std::make_shared<prx::fg::collision_info_t>(g_type, g_params, rot, t) };

      // ObstacleFactor factor(obstacle, robot, key);
      std::shared_ptr<ObstacleFactor> obs_factor{ std::make_shared<ObstacleFactor>(obstacle, robot, kr, 1) };
      factors.push_back(obs_factor);
      // factors.emplace_back(obstacle, robot, gtsam::Key(i));
    }
  }

  // boost::optional<std::vector<Matrix>&>
  const double step{ 0.5 };
  const std::size_t total_factors{ factors.size() };
  std::ofstream ofs_map("/Users/Gary/pracsys/ML4KP-devel/out/sdf.txt");
  for (double x = -10; x <= 30; x += step)
  {
    for (double y = -10; y <= 30; y += step)
    {
      bool colliding{ false };
      double error{ 0.0 };
      std::vector<Eigen::MatrixXd> Hvec;
      Hvec.push_back(Eigen::Matrix2d::Zero());
      Eigen::Matrix2d H{ Eigen::Matrix2d::Zero() };

      const State state(x, y);
      values.insert_or_assign(kr, state);
      // const State state(-5, -5);

      // ofs_map << x << " " << y << " ";
      for (int i = 0; i < total_factors; ++i)
      {
        // ofs_map << factors[i].in_collision(state) << " ";
        colliding |= factors[i]->in_collision(state);
        // error += factors[i]->unwhitenedError(values, Hvec)[0];
        // H += Hvec[0];
        // if (colliding)
        //   break;
      }
      H.normalize();
      // ofs_map << colliding << "\n";
      ofs_map << x << " " << y << " " << (colliding ? 1 : 0) << " " << error << " " << H(0, 0) << " " << H(1, 1)
              << "\n";
    }
  }
  ofs_map.close();

  prx::three_js_group_t* vis_group = new prx::three_js_group_t({}, { obstacle_list });
  vis_group->output_html("output.html");

  return 0;
}