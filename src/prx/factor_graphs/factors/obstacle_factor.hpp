#pragma once
#include <array>
#include <numeric>
#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "prx/external/PQP/MatVecEigen.hpp"
#include "prx/external/PQP/PQP_Eigen.hpp"
#include "prx/utilities/geometry/geometry.hpp"
#include "prx/utilities/geometry/movable_object.hpp"
#include "prx/factor_graphs/lie_groups/se3.hpp"
#include "prx/factor_graphs/utilities/symbols_factory.hpp"

namespace prx
{
namespace fg
{
struct collision_info_t
{
  enum CollisionErrorType
  {
    STEP = 0,
    LINEAR,
    QUADRATIC
  };
  using SharedPtr = std::shared_ptr<collision_info_t>;
  // using PQPModel = typename PQP_Model;
  // using PQPModel = typename PQP_Model_Eigen;
  collision_info_t(collision_info_t& other)
    : pose(other.pose), model(other.model), geom_type(other.geom_type), params(other.params)
  {
  }
  collision_info_t(collision_info_t&& other)
    : pose(other.pose), model(other.model), geom_type(other.geom_type), params(other.params)
  {
  }
  collision_info_t(const geometry_type_t geom_type_in, const std::vector<double> params_in,
                   const Eigen::Matrix3d& rotation_in, const Eigen::Vector3d& translation_in)
    : pose(Eigen::Quaterniond(rotation_in), translation_in), geom_type(geom_type_in), params(params_in)
  {
    model = geometry_t::create_collision_geometry<PQP_Model>(geom_type, params);
    // model = geometry_t::create_collision_geometry<PQP_Model_Eigen>(geom_type, params);
  }
  collision_info_t(const geometry_type_t geom_type, const std::vector<double>& params)
    : collision_info_t(geom_type, params, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()) {};

  collision_info_t(const prx::param_loader& param_loader)
    : geom_type(geometry_t::geometry_type(param_loader["type"].as<std::string>()))
    , params(param_loader["parameters"].as<std::vector<double>>())
    , pose(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero())
  {
    model = geometry_t::create_collision_geometry<PQP_Model>(geom_type, params);
  }

  static prx::param_loader default_parameters()
  {
    prx::param_loader params{};
    params["type"].set("SPHERE");
    params["parameters"].set(std::vector<double>({ 0.5 }));
    return params;
  }

  static std::vector<SharedPtr> generate_infos(std::vector<std::shared_ptr<prx::movable_object_t>>& obstacle_list)
  {
    std::vector<SharedPtr> infos{};

    for (auto obstacle : obstacle_list)
    {
      const prx::movable_object_t::Geometries geometries{ obstacle->get_geometries() };
      const prx::movable_object_t::Configurations configurations{ obstacle->get_configurations() };

      const std::size_t total_geoms{ geometries.size() };

      for (int i = 0; i < total_geoms; ++i)
      {
        const std::shared_ptr<prx::geometry_t> g{ geometries[i].second };
        const std::shared_ptr<prx::transform_t> tf{ configurations[i].second };

        const prx::geometry_type_t g_type{ g->get_geometry_type() };
        const std::vector<double> g_params{ g->get_geometry_params() };

        const Eigen::Matrix3d rot{ tf->rotation() };
        const Eigen::Vector3d t{ tf->translation() };

        infos.emplace_back(new collision_info_t(g_type, g_params, rot, t));
      }
    }
    return infos;
  }

  prx::fg::se3_t pose;

  const geometry_type_t geom_type;
  const std::vector<double> params;
  // Eigen::Vector3d translation;
  // Eigen::Matrix3d rotation;
  std::shared_ptr<PQP_Model> model;
  // std::shared_ptr<PQP_Model_Eigen> model;
};

// using StateToConfiguration = void (*)(const std::size_t, const ml4kp_bridge::SpacePoint&);
template <typename State, typename ConfigurationFromState,
          collision_info_t::CollisionErrorType ERROR_TYPE = collision_info_t::CollisionErrorType::STEP>
class obstacle_factor_t : public gtsam::NoiseModelFactor1<State>
{
  using Base = gtsam::NoiseModelFactor1<State>;
  using Derived = obstacle_factor_t<State, ConfigurationFromState, ERROR_TYPE>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
  using CollisionInfoPtr = std::shared_ptr<collision_info_t>;

  using Key = gtsam::Key;
  using FactorGraph = gtsam::NonlinearFactorGraph;
  using ObjectPtr = std::shared_ptr<prx::movable_object_t>;
  using SF = prx::fg::symbol_factory_t;

  using Rotation = Eigen::Matrix3d;
  using Translation = Eigen::Vector3d;
  using Params = std::vector<double>;
  static constexpr Eigen::Index StateDim{ gtsam::traits<State>::dimension };

public:
  using CollideResult = PQP_CollideResult;
  using DistanceResult = PQP_DistanceResult;
  using ToleranceResult = PQP_ToleranceResult;

  obstacle_factor_t(obstacle_factor_t&& other)
    : Base(other)
    , _obstacle_info(other._obstacle_info)
    , _robot_info(other._robot_info)
    , _max_error_dist(other._max_error_dist)
    , _activation_distance(other._activation_distance)
  {
  }

  obstacle_factor_t(CollisionInfoPtr obstacle, CollisionInfoPtr robot, const gtsam::Key& robot_pose_key,
                    double activation_distance = 1, double max_error_dist = 0.1, const NoiseModel& cost_model = nullptr)
    : Base(cost_model, robot_pose_key)
    , _obstacle_info(obstacle)
    , _robot_info(robot)
    , _activation_distance(activation_distance)
    , _max_error_dist(max_error_dist)
  {
    // PRX_DBG_VARS(_obstacle_info->pose);
  }

  obstacle_factor_t(const geometry_type_t obstacle_gt, const Params& obstacle_params, const Rotation& obstacle_rotation,
                    const Translation& obstacle_translation,                     // no-lint
                    const geometry_type_t robot_gt, const Params& robot_params,  // no-lint
                    const gtsam::Key& robot_pose_key, double activation_distance = 1, double max_error_dist = 0.1,
                    const NoiseModel& cost_model = nullptr)
    : Base(cost_model, robot_pose_key)
    , _obstacle_info(
          std::make_shared<collision_info_t>(obstacle_gt, obstacle_params, obstacle_rotation, obstacle_translation))
    , _robot_info(std::make_shared<collision_info_t>(robot_gt, robot_params))
    , _activation_distance(activation_distance)
    , _max_error_dist(max_error_dist)
  {
    // PRX_DBG_VARS(_obstacle_info->pose);
  }

  static bool in_collision(const State& x0, CollisionInfoPtr obstacle_info, CollisionInfoPtr robot_info,
                           ConfigurationFromState& config_from_state, CollideResult& result, Eigen::Vector3d& pt)
  {
    Eigen::Matrix3d robot_rot{ robot_info->pose.quaternion().matrix() };
    Eigen::Matrix3d obstacle_rot{ obstacle_info->pose.quaternion().matrix() };
    config_from_state(robot_rot, robot_info->pose.position(), x0);
    PQP_REAL Mr[3][3], Tr[3];
    PQP_REAL Mo[3][3], To[3];
    ::copy(Mr, robot_rot);
    ::copy(Mo, obstacle_rot);
    ::copy(Tr, robot_info->pose.position());
    ::copy(To, obstacle_info->pose.position());

    PQP_Collide(&result,                             // no-lint
                Mr, Tr, robot_info->model.get(),     // no-lint
                Mo, To, obstacle_info->model.get(),  // no-lint
                PQP_FIRST_CONTACT);

    // if (result.Colliding())
    // {
    // Eigen::Vector3d pt;
    // Eigen::Vector3d pose{ robot_info->pose.position() };
    pt[0] = result.T[0];
    pt[1] = result.T[1];
    pt[2] = result.T[2];
    // PRX_DBG_VARS(pose, pt.transpose());
    // }
    return result.Colliding();
  }

  inline bool in_collision(const State& x0) const
  {
    Eigen::Vector3d pt;
    return in_collision(x0, pt);
  }
  inline bool in_collision(const State& x0, Eigen::Vector3d& pt) const
  {
    return in_collision(x0, _obstacle_info, _robot_info, _config_from_state, _collision_result, pt);
  }
  inline bool inside_obstacle(const Eigen::Vector3d& pt) const
  {
    static double dummy_dist{ 0.0 };
    return inside_obstacle(pt, dummy_dist);
  }

  inline bool inside_obstacle(const Eigen::Vector3d& pt, double& distance) const
  {
    return inside_obstacle(pt, _obstacle_info, distance);
  }

  inline static bool inside_obstacle(const State& x, CollisionInfoPtr obstacle_info,
                                     ConfigurationFromState& config_from_state, double& distance)
  {
    Eigen::Matrix3d rot;
    Eigen::Vector3d pt;
    config_from_state(rot, pt, x);
    return inside_obstacle(pt, obstacle_info, distance);
  }

  inline static bool inside_obstacle(const Eigen::Vector3d& pt, CollisionInfoPtr obstacle_info, double& distance)
  {
    const Eigen::Vector3d pt_r{ obstacle_info->pose.inverse() * pt };
    bool is_inside{ false };

    double aux{ 0.0 };
    const std::vector<double>& params{ obstacle_info->params };
    switch (obstacle_info->geom_type)
    {
      case prx::geometry_type_t::BOX:
        is_inside = std::abs(pt_r[0]) < (params[0] * 0.5) and std::abs(pt_r[1]) < (params[1] * 0.5) and
                    std::abs(pt_r[2]) < (params[2] * 0.5);
        distance = std::min((params[0] * 0.5) - std::abs(pt_r[0]), (params[1] * 0.5) - std::abs(pt_r[1]));
        distance = std::min(distance, (params[2] * 0.5) - std::abs(pt_r[2]));
        break;
      case prx::geometry_type_t::SPHERE:
        aux = pt_r.norm();
        is_inside = aux < params[0];
        distance = params[0] - aux;
        break;
      case prx::geometry_type_t::CYLINDER:
        aux = pt_r.head(2).norm();
        is_inside = aux < params[0] and std::abs(pt_r[2]) < params[1];
        distance = std::min(params[0] - aux, params[1] - std::abs(pt_r[2]));
        break;
      default:
        prx_throw("Shape not supported");
    };
    return is_inside;
  }

  void recover_collision(CollisionInfoPtr robot_info, CollisionInfoPtr obstacle_info, Eigen::Vector3d& p1,
                         Eigen::Vector3d& p2, CollideResult& result) const
  {
    if (result.Colliding())
    {
      const int tri_1{ result.Id1(0) };
      const int tri_2{ result.Id2(0) };
      ::copy(p1, robot_info->model->tris[tri_1].p1);
      ::copy(p2, obstacle_info->model->tris[tri_2].p1);

      p1 = (robot_info->pose) * p1;
      p2 = (obstacle_info->pose) * p2;
      // PRX_DBG_VARS(p1.transpose(), p2.transpose());
    }
  }

  static double distances(const State& x0, Eigen::Vector3d& p1, Eigen::Vector3d& p2, CollisionInfoPtr obstacle_info,
                          CollisionInfoPtr robot_info, ConfigurationFromState& config_from_state,
                          DistanceResult& result)
  {
    Eigen::Matrix3d robot_rot{ robot_info->pose.quaternion().matrix() };
    Eigen::Matrix3d obstacle_rot{ obstacle_info->pose.quaternion().matrix() };
    PQP_REAL Mr[3][3], Tr[3];
    PQP_REAL Mo[3][3], To[3];
    config_from_state(robot_rot, robot_info->pose.position(), x0);
    // PRX_DBG_VARS(robot_rot);
    robot_info->pose.quaternion() = robot_rot;

    ::copy(Mr, robot_rot);
    ::copy(Mo, obstacle_rot);
    ::copy(Tr, robot_info->pose.position());
    ::copy(To, obstacle_info->pose.position());

    PQP_Distance(&result,                             // no-lint
                 Mr, Tr, robot_info->model.get(),     // no-lint
                 Mo, To, obstacle_info->model.get(),  // no-lint
                 0.0, 0.0);

    // P1: is the point in the robot that is closest to a collision IN ROBOT FRAME
    p1[0] = result.P1()[0];
    p1[1] = result.P1()[1];
    p1[2] = result.P1()[2];

    // P2 is the vector to the 'center' of the obstacle where the closest collision happens
    p2[0] = result.P2()[0];
    p2[1] = result.P2()[1];
    p2[2] = result.P2()[2];
    // PRX_DBG_VARS(x0);
    // PRX_DBG_VARS(robot_info->pose);
    // PRX_DBG_VARS(p1.transpose(), p2.transpose());

    p1 = (robot_info->pose) * p1;
    p2 = (obstacle_info->pose) * p2;

    auto px0 = (robot_info->pose) * Eigen::Vector3d(1, 0, 0);
    auto py0 = (robot_info->pose) * Eigen::Vector3d(0, 1, 0);

    // PRX_DBG_VARS(px0.transpose());
    // PRX_DBG_VARS(py0.transpose());
    // PRX_DBG_VARS(p1.transpose(), p2.transpose());
    return result.Distance();
  }

  inline double distances(const State& x0) const
  {
    static Eigen::Vector3d d0, d1;
    return distances(x0, d0, d1, _obstacle_info, _robot_info, _config_from_state, _distance_result);
  }
  inline double distances(const State& x0, Eigen::Vector3d& closest_point, Eigen::Vector3d& p2) const
  {
    return distances(x0, closest_point, p2, _obstacle_info, _robot_info, _config_from_state, _distance_result);
  }

  static bool close_enough(const State& x0, const double tolerance, CollisionInfoPtr obstacle_info,
                           CollisionInfoPtr robot_info, ConfigurationFromState& config_from_state,
                           ToleranceResult& result)
  {
    Eigen::Matrix3d robot_rot{ robot_info->pose.quaternion().matrix() };
    Eigen::Matrix3d obstacle_rot{ obstacle_info->pose.quaternion().matrix() };
    PQP_REAL Mr[3][3], Tr[3];
    PQP_REAL Mo[3][3], To[3];
    config_from_state(robot_rot, robot_info->pose.position(), x0);

    ::copy(Mr, robot_rot);
    ::copy(Mo, obstacle_rot);
    ::copy(Tr, robot_info->pose.position());
    ::copy(To, obstacle_info->pose.position());

    PQP_Tolerance(&result,                             // no-lint
                  Mr, Tr, robot_info->model.get(),     // no-lint
                  Mo, To, obstacle_info->model.get(),  // no-lint
                  tolerance);
    return result.CloserThanTolerance();
  }

  inline bool close_enough(const State& x0, const double tolerance) const
  {
    return close_enough(x0, tolerance, _obstacle_info, _robot_info, _config_from_state, _tolerance_result);
  }

  virtual std::size_t dim() const override
  {
    return StateDim;
  }

  virtual bool active(const gtsam::Values& values) const override
  {
    const State state{ values.at<State>(this->template key<1>()) };
    const bool is_close{ close_enough(state, _activation_distance) };

    return is_close;
  }

  virtual bool sendable() const override
  {
    return false;
  }

  virtual Eigen::VectorXd evaluateError(const State& x0,
                                        boost::optional<Eigen::MatrixXd&> H0 = boost::none) const override
  {
    Eigen::VectorXd error{ Eigen::Vector<double, 1>::Zero() };

    Eigen::Vector3d p1, p2, p_coll;
    const double distance{ distances(x0, p1, p2) };

    const double activated_dist{ _activation_distance - distance };
    error[0] = std::max(activated_dist, 0.0);

    if (H0)
    {
      const bool collision{ in_collision(x0, p_coll) };
      p1 = collision ? p_coll : p1;
      // if (collision)
      // {
      //   const double x{ x0[0] };
      //   const double y{ x0[1] };
      //   PRX_DBG_VARS(collision, x, y, p1.transpose(), p2.transpose());
      // }
      _config_from_state(collision, x0, p1, p2, *H0);
      // else
      // {
      //   _config_from_state(x0, p2 - p1, *H0);
      // }
    }

    return error;
  }

  void to_stream(std::ostream& os, const gtsam::Values& values) const
  {
    if (active(values))
    {
      const State state{ values.at<State>(this->template key<1>()) };
      const char sp{ prx::constants::separating_value };
      Eigen::MatrixXd H;
      const State error{ evaluateError(state, H) };
      const int collision{ in_collision(state) ? 1 : 0 };

      os << symbol_factory_t::formatter(this->template key<1>()) << " " << state.transpose() << sp;
      os << collision << sp << H.diagonal().transpose() << sp;
      os << "\n";
    }
  }

  static gtsam::NonlinearFactorGraph collision_factors(const Key& xkey, ObjectPtr obstacle,
                                                       const CollisionInfoPtr robot, const double obstacle_tolerance,
                                                       NoiseModel obstacle_noise = nullptr)
  {
    gtsam::NonlinearFactorGraph graph;

    const prx::movable_object_t::Geometries geometries{ obstacle->get_geometries() };
    const prx::movable_object_t::Configurations configurations{ obstacle->get_configurations() };

    const std::size_t total_geoms{ geometries.size() };

    for (int i = 0; i < total_geoms; ++i)
    {
      const std::shared_ptr<prx::geometry_t> g{ geometries[i].second };
      const std::shared_ptr<prx::transform_t> tf{ configurations[i].second };

      const prx::geometry_type_t g_type{ g->get_geometry_type() };
      const std::vector<double> g_params{ g->get_geometry_params() };

      const Eigen::Matrix3d rot{ tf->rotation() };
      const Eigen::Vector3d t{ tf->translation() };

      graph.emplace_shared<Derived>(g_type, g_params, rot, t,                  // no-lint
                                    robot->geom_type, robot->params, xkey,     // no-lint
                                    obstacle_tolerance, 0.1, obstacle_noise);  // 0.1 is not used?
    }
    return std::move(graph);
  }

  const CollisionInfoPtr obstacle_info() const
  {
    return _obstacle_info;
  }

private:
  const CollisionInfoPtr _obstacle_info;
  mutable CollisionInfoPtr _robot_info;

  // const Eigen::Vector3d obstacle_translation;
  // const Eigen::Matrix3d obstacle_rotation;
  // std::shared_ptr<PQP_Model_Eigen> model;

  // mutable PQP_CollideResult_Eigen _collision_result;
  mutable CollideResult _collision_result;
  mutable DistanceResult _distance_result;
  mutable ToleranceResult _tolerance_result;

  mutable ConfigurationFromState _config_from_state;
  // const Pose _obstacle_pose;

  const double _activation_distance;
  const double _max_error_dist;

  Eigen::Transform<double, 3, Eigen::TransformTraits::Isometry> T_robot;
};
}  // namespace fg
}  // namespace prx