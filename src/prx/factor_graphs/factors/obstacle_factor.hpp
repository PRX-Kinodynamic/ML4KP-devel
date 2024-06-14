#pragma once
#include <array>
#include <numeric>
#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "prx/external/PQP/PQP_Eigen.hpp"
#include "prx/utilities/geometry/geometry.hpp"
#include "prx/factor_graphs/lie_groups/se3.hpp"
#include "prx/factor_graphs/utilities/symbols_factory.hpp"

namespace prx
{
namespace fg
{
struct collision_info_t
{
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
    : collision_info_t(geom_type, params, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()){};

  prx::fg::se3_t pose;

  const geometry_type_t geom_type;
  const std::vector<double> params;
  // Eigen::Vector3d translation;
  // Eigen::Matrix3d rotation;
  std::shared_ptr<PQP_Model> model;
  // std::shared_ptr<PQP_Model_Eigen> model;
};

// using StateToConfiguration = void (*)(const std::size_t, const ml4kp_bridge::SpacePoint&);
template <typename State, typename ConfigurationFromState>
class obstacle_factor_t : public gtsam::NoiseModelFactor1<State>
{
  using Base = gtsam::NoiseModelFactor1<State>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
  using CollisionInfoPtr = std::shared_ptr<collision_info_t>;

  using Rotation = Eigen::Matrix3d;
  using Translation = Eigen::Vector3d;
  using Params = std::vector<double>;
  static constexpr Eigen::Index StateDim{ gtsam::traits<State>::dimension };

public:
  obstacle_factor_t(obstacle_factor_t&& other)
    : Base(other)
    , _obstacle_info(other._obstacle_info)
    , _robot_info(other._robot_info)
    , _max_error_dist(other._max_error_dist)
  {
    PRX_DBG_VARS(_obstacle_info->pose);
    PRX_DBG_VARS(_robot_info->pose);
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
  bool in_collision(const State& x0) const
  {
    Eigen::Matrix3d robot_rot{ _robot_info->pose.quaternion().matrix() };
    Eigen::Matrix3d obstacle_rot{ _obstacle_info->pose.quaternion().matrix() };
    _config_from_state(robot_rot, _robot_info->pose.position(), x0);
    PQP_REAL Mr[3][3], Tr[3];
    PQP_REAL Mo[3][3], To[3];
    copy(Mr, robot_rot);
    copy(Mo, obstacle_rot);
    copy(Tr, _robot_info->pose.position());
    copy(To, _obstacle_info->pose.position());

    // PQP_Collide(&_collision_result,                                                          // no-lint
    //             robot_rot, _robot_info->pose.position(), _robot_info->model.get(),             // no-lint
    //             obstacle_rot, _obstacle_info->pose.position(), _obstacle_info->model.get(),  // no-lint
    //             PQP_FIRST_CONTACT);
    PQP_Collide(&_collision_result,                   // no-lint
                Mr, Tr, _robot_info->model.get(),     // no-lint
                Mo, To, _obstacle_info->model.get(),  // no-lint
                PQP_FIRST_CONTACT);

    // printf("%.4f %.4f %d %d\n",                                         // no-lint
    //        x0[0], x0[1],                                                // no-lint
    //        _collision_result.Colliding(), _collision_result.NumPairs()  // no-lint
    // );
    return _collision_result.Colliding();
  }

  double distances(const State& x0, Eigen::Vector3d& closest_point, Eigen::Vector3d& p2) const
  {
    Eigen::Matrix3d robot_rot{ _robot_info->pose.quaternion().matrix() };
    Eigen::Matrix3d obstacle_rot{ _obstacle_info->pose.quaternion().matrix() };
    PQP_REAL Mr[3][3], Tr[3];
    PQP_REAL Mo[3][3], To[3];
    _config_from_state(robot_rot, _robot_info->pose.position(), x0);

    copy(Mr, robot_rot);
    copy(Mo, obstacle_rot);
    copy(Tr, _robot_info->pose.position());
    copy(To, _obstacle_info->pose.position());

    // PQP_Distance(&_distance_result,                                                           // no-lint
    //              robot_rot, _robot_info.pose.position(), _robot_info.model.get(),             // no-lint
    //              obstacle_rot, _obstacle_info->pose.position(), _obstacle_info->model.get(),  // no-lint
    //              0.0, 0.0);
    PQP_DistanceResult _distance_result_pqp;
    PQP_Distance(&_distance_result_pqp,                // no-lint
                 Mr, Tr, _robot_info->model.get(),     // no-lint
                 Mo, To, _obstacle_info->model.get(),  // no-lint
                 0.0, 0.0);

    // Vprintg(_distance_result_pqp.P1());
    // Vprintg(_distance_result_pqp.P2());
    // PRX_DBG_VARS(_distance_result.P1().transpose(), _distance_result.P2().transpose());
    // closest_point = _robot_info.pose.position() + _distance_result.P1();
    closest_point[0] = _distance_result_pqp.P1()[0];
    closest_point[1] = _distance_result_pqp.P1()[1];
    closest_point[2] = _distance_result_pqp.P1()[2];

    p2[0] = _distance_result_pqp.P2()[0];
    p2[1] = _distance_result_pqp.P2()[1];
    p2[2] = _distance_result_pqp.P2()[2];
    // printf("%.4f %.4f %d %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n",                                       // no-lint
    //        x0[0], x0[1], in_collision(x0), _distance_result.Distance(),                               // no-lint
    //        _distance_result_pqp.P1()[0], _distance_result_pqp.P1()[1], _distance_result_pqp.P1()[2],  // no-lint
    //        _distance_result_pqp.P2()[0], _distance_result_pqp.P2()[1], _distance_result_pqp.P2()[2]   // no-lint
    // );                                                                                                // no-lint
    // return _distance_result_pqp.Distance();
    return _distance_result.Distance();
  }

  bool close_enough(const State& x0, const double tolerance) const
  {
    Eigen::Matrix3d robot_rot{ _robot_info->pose.quaternion().matrix() };
    Eigen::Matrix3d obstacle_rot{ _obstacle_info->pose.quaternion().matrix() };
    PQP_REAL Mr[3][3], Tr[3];
    PQP_REAL Mo[3][3], To[3];
    _config_from_state(robot_rot, _robot_info->pose.position(), x0);

    copy(Mr, robot_rot);
    copy(Mo, obstacle_rot);
    copy(Tr, _robot_info->pose.position());
    copy(To, _obstacle_info->pose.position());

    PQP_Tolerance(&_tolerance_result,                   // no-lint
                  Mr, Tr, _robot_info->model.get(),     // no-lint
                  Mo, To, _obstacle_info->model.get(),  // no-lint
                  tolerance);
    return _tolerance_result.CloserThanTolerance();
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

  virtual Eigen::VectorXd evaluateError(const State& x0,
                                        boost::optional<Eigen::MatrixXd&> H0 = boost::none) const override
  {
    Eigen::VectorXd error{ Eigen::VectorXd::Zero(StateDim) };
    Eigen::Vector3d closest_point, p2;
    // if (close_enough(x0, _activation_distance))
    // {
    error = Eigen::VectorXd::Ones(StateDim);
    // }
    // error = Eigen::VectorXd::Ones(StateDim);
    if (H0)
    {
      const double dist{ std::max(distances(x0, closest_point, p2), _max_error_dist) };
      if (in_collision(x0))
      {
        closest_point = -p2;
      }
      // distances(x0, closest_point);
      _config_from_state(*H0, closest_point);
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

private:
  const CollisionInfoPtr _obstacle_info;
  mutable CollisionInfoPtr _robot_info;

  // const Eigen::Vector3d obstacle_translation;
  // const Eigen::Matrix3d obstacle_rotation;
  // std::shared_ptr<PQP_Model_Eigen> model;

  mutable PQP_CollideResult _collision_result;
  // mutable PQP_CollideResult_Eigen _collision_result;
  mutable PQP_DistanceResult_Eigen _distance_result;
  mutable PQP_ToleranceResult _tolerance_result;

  mutable ConfigurationFromState _config_from_state;
  // const Pose _obstacle_pose;

  const double _activation_distance;
  const double _max_error_dist;
};
}  // namespace fg
}  // namespace prx