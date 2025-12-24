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
#include "prx/factor_graphs/lie_groups/se2.hpp"
#include "prx/factor_graphs/utilities/values_utilities.hpp"
#include "prx/factor_graphs/utilities/default_parameters.hpp"
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

using State = prx::fg::SE2_t;
using Rotation = Eigen::Matrix3d;
using Translation = Eigen::Vector3d;
using CollisionInfoPtr = std::shared_ptr<prx::fg::collision_info_t>;

struct configuration_from_state
{
  Eigen::VectorXd h(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1)
  {
    return (p0 - p1).head(2);
  }

  void operator()(Rotation& rotation, Translation& translation, const State& state)
  {
    const Eigen::Vector<double, 1> vec{ state.angle() };
    rotation = prx::euler_to_rotation<Rotation>(vec, "Z");
    translation[0] = state[0];
    translation[1] = state[1];
    translation[2] = 0;
  }
  void operator()(const bool collision, const State& state, const Translation& p1, const Translation& p2,
                  Eigen::MatrixXd& H)
  {
    const Eigen::Vector<double, 1> vec{ state.angle() };
    // H = Eigen::Matrix<double, 3, 3>::Identity();
    H = Eigen::Matrix<double, 1, 3>::Zero();
    H(0, 0) = (p2 - p1)[0];
    H(0, 1) = (p2 - p1)[1];
    // H.block<2, 2>(0, 0) = prx::euler_to_rotation<Rotation>(-vec, "Z").block<2, 2>(0, 0);
  }
};

int main(int argc, char* argv[])
{
  using ObstacleFactorStep =
      prx::fg::obstacle_factor_t<State, configuration_from_state, prx::fg::collision_info_t::CollisionErrorType::STEP>;
  const prx::obstacle_loader_t obstacles{ "environments/simple_obstacle.yaml" };
  const std::vector<std::shared_ptr<prx::movable_object_t>> obstacle_list{ obstacles.get_obstacles() };
  const std::vector<std::string> obstacle_names{ obstacles.get_names() };
  const Eigen::Vector3d min_bound{ obstacles.min_bounds() };
  const Eigen::Vector3d max_bound{ obstacles.max_bounds() };

  std::vector<double> geom_params({ 0.42, 0.25, 0.25 });
  // std::vector<double> geom_params({ 4.0, 2.5, 0.25 });
  const Rotation rotation{ Rotation::Identity() };
  const Translation translation{ Translation::Zero() };
  CollisionInfoPtr robot{ std::make_shared<prx::fg::collision_info_t>(prx::geometry_type_t::BOX, geom_params, rotation,
                                                                      translation) };

  std::vector<std::shared_ptr<ObstacleFactorStep>> factors;

  gtsam::Key kr(0);
  gtsam::NonlinearFactorGraph graph;
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

      CollisionInfoPtr obstacle{ std::make_shared<prx::fg::collision_info_t>(g_type, g_params, rot, t) };

      graph.emplace_shared<ObstacleFactorStep>(obstacle, robot, kr, 1);
      // std::shared_ptr<ObstacleFactorStep> obs_factor{ std::make_shared<ObstacleFactorStep>(obstacle, robot, kr, 1) };
      // factors.push_back(obs_factor);
    }
  }

  std::ofstream ofs(prx::out_path + "/collision.txt");
  // boost::optional<std::vector<Matrix>&>
  const double step{ 0.5 };
  const std::size_t total_factors{ factors.size() };

  Eigen::VectorXd error;
  std::vector<Eigen::MatrixXd> Hvec;
  Hvec.push_back(Eigen::Matrix2d::Zero());
  Eigen::MatrixXd H{ Eigen::Matrix3d::Zero() };
  const Eigen::Vector3d eps{ 0.1, 0.1, 0.1 };

  State x0p;
  State state(0, 0, 0);

  PRX_DBG_VARS(min_bound.transpose());
  PRX_DBG_VARS(max_bound.transpose());
  const std::size_t total_states{ 1000 };

  gtsam::Values values, result;
  const gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
  // graph.push_back(factors.begin(), factors.end());
  for (int k = 0; k < total_states; ++k)
  {
    state = State::random(min_bound, max_bound);
    // state = State(-3.3334, -1.1152, -0.254952);
    // state = State(2.53953, -4.0, 0.0412863);
    // state = State(23.4, -4.42187, -0.190196);
    // state = State(15, -2.0, -1.2);
    // state = State(18.7286, -4.125, prx::constants::pi / 2.0);
    values.clear();

    values.insert(kr, state);
    // for (int i = 0; i < total_factors; ++i)
    // {
    //   if (factors[i]->active(values))
    //   {
    //     error = factors[i]->evaluateError(state, H);
    //     x0p = state.vector() + H * eps;
    //     PRX_VALUES_TO_STREAM(ofs, state, error.transpose(), x0p.transpose());
    //   }
    // }
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
    result = optimizer.optimize();
    x0p = result.at<State>(kr);
    PRX_VALUES_TO_STREAM(ofs, state, x0p);
  }

  prx::three_js_group_t* vis_group = new prx::three_js_group_t({}, { obstacle_list });
  vis_group->output_html("output.html");

  return 0;
}