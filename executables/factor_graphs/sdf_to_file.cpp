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
  void operator()(const bool collision, const State& state, const Translation& p1, const Translation& p2,
                  Eigen::MatrixXd& H)
  {
    H = Eigen::Matrix<double, 1, 2>::Zero();
    Eigen::Vector2d vec{ (p1 - p2).head(2) };
    // Eigen::Vector2d vec{ Eigen::Vector2d::Zero() };
    if (collision)
    {
      // PRX_DBG_VARS(state.transpose(), p1.transpose(), p2.transpose());
      vec = -p1.head(2);
    }

    H(0, 0) = vec[0];
    H(0, 1) = vec[1];
  }
};

int main(int argc, char* argv[])
{
  using ObstacleFactor = prx::fg::obstacle_factor_t<State, configuration_from_state>;
  prx::param_loader params{};
  params["step"].set(0.5);
  params["env"].set("forest");
  params.add_opts(argc, argv);

  const std::string environment_name{ "environments/" + params["env"].as<>() + ".yaml" };
  auto obstacles = prx::load_obstacles(environment_name);
  std::vector<std::shared_ptr<prx::movable_object_t>> obstacle_list{ obstacles.second };
  std::vector<std::string> obstacle_names{ obstacles.first };
  const prx::EnvironmentBounds bounds{ prx::obstacle_loader_t::bounds_from_yaml(environment_name) };

  std::vector<double> sphere_params({ 0.5 });
  const Rotation rotation{ Rotation::Identity() };
  const Translation translation{ Translation::Zero() };
  CollisionInfoPtr robot{ std::make_shared<prx::fg::collision_info_t>(prx::geometry_type_t::SPHERE, sphere_params,
                                                                      rotation, translation) };

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
      // PRX_DBG_VARS(g_params, t.transpose());
      // PRX_DBG_VARS(rot);

      CollisionInfoPtr obstacle{ std::make_shared<prx::fg::collision_info_t>(g_type, g_params, rot, t) };

      // ObstacleFactor factor(obstacle, robot, key);
      std::shared_ptr<ObstacleFactor> obs_factor{ std::make_shared<ObstacleFactor>(obstacle, robot, kr, 5) };
      factors.push_back(obs_factor);
      // factors.emplace_back(obstacle, robot, gtsam::Key(i));
    }
  }

  // boost::optional<std::vector<Matrix>&>
  const double step{ params["step"].as<double>() };
  const std::size_t total_factors{ factors.size() };
  const std::string filename{ prx::out_path + "/sdf_" + params["env"].as<>() + ".txt" };
  PRX_DBG_VARS(filename);
  std::ofstream ofs_map(filename);

  const Eigen::Vector3d min_bounds{ bounds.first };
  const Eigen::Vector3d max_bounds{ bounds.second };
  double min_dist{ 0.0 };
  PRX_DBG_VARS(min_bounds.transpose());
  PRX_DBG_VARS(max_bounds.transpose());

  const double max_dist{ (max_bounds - min_bounds).norm() };
  for (double x = min_bounds[0]; x <= max_bounds[0]; x += step)
  {
    for (double y = min_bounds[1]; y <= max_bounds[1]; y += step)
    {
      bool colliding{ false };
      double error{ 0.0 };
      std::vector<Eigen::MatrixXd> Hvec;
      Hvec.push_back(Eigen::Matrix2d::Zero());
      Eigen::MatrixXd H{ Eigen::MatrixXd::Zero(1, 2) };
      Eigen::MatrixXd H_min{ Eigen::MatrixXd::Zero(1, 2) };

      const State state(x, y);

      min_dist = 0.0;
      for (int i = 0; i < total_factors; ++i)
      {
        // min_dist = std::min(min_dist, factors[i]->distances(state));
        const double dist{ factors[i]->evaluateError(state, H)[0] };
        if (dist > min_dist)
        {
          min_dist = dist;
          H_min = H;
        }
      }
      H_min.normalize();
      ofs_map << x << " " << y << " " << min_dist << " ";
      ofs_map << H_min(0, 0) << " " << H_min(0, 1) << "\n";
    }
  }
  ofs_map.close();

  prx::three_js_group_t* vis_group = new prx::three_js_group_t({}, { obstacle_list });
  vis_group->output_html("output.html");

  return 0;
}