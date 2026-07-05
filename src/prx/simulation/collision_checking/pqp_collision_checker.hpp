#pragma once

#include <memory>
#include <unordered_map>

#include "prx/utilities/general/debug_utils.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/utilities/general/prx_assert.hpp"
#include "prx/utilities/defs.hpp"
#include "prx/utilities/geometry/geometry.hpp"
#include "prx/utilities/geometry/movable_object.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
namespace prx
{
namespace collision_checking
{

namespace pqp
{

// Representation of a rigid body: SE3 + pqp_model
struct rigid_body_t
{
  // Ideally would be an SE3. Eigen::Transform gets complicated to use .data() for going back and forth with PQP
  // Its easier to have it in separate objects
  Eigen::Vector3d position;
  Eigen::Matrix3d rotation;
  std::shared_ptr<PQP_Model> model;
};

/** @brief Updates the poses for the geometry. */
template <typename MovableObjectPlant,
          std::enable_if_t<std::is_base_of_v<MovableObjectPlant, prx::movable_object_t>, bool> = true>
std::vector<std::shared_ptr<rigid_body_t>>
create_obstacles(const std::vector<std::shared_ptr<MovableObjectPlant>> obstacles)
{
  std::vector<std::shared_ptr<rigid_body_t>> obstacles_pqp_infos;
  for (auto&& object : obstacles)
  {
    const std::vector<std::string> keys{ object->keys() };
    for (auto k : keys)
    {
      auto rigid_body = std::make_shared<rigid_body_t>();
      rigid_body->model = object->geometry(k)->collision_geometry();
      rigid_body->rotation = object->configuration(k).linear();
      rigid_body->position = object->configuration(k).translation();
      obstacles_pqp_infos.push_back(rigid_body);
    }
  }
  return obstacles_pqp_infos;
}

inline std::vector<std::shared_ptr<rigid_body_t>> create_obstacles(const prx::param_loader obstacles)
{
  obstacle_loader_t loader(obstacles);
  return create_obstacles(loader.get_obstacles());
}

// struct plant_bodies_t
// {
//   std::vector<rigid_body_t> geometries;
//   // std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> configurations;
//   // std::vector<std::shared_ptr<PQP_Model>> pqp_models;
// };

static std::vector<std::shared_ptr<PQP_Model>>
create_pqp_models(const std::vector<std::shared_ptr<prx::geometry_t>>& geoms)
{
  std::vector<std::shared_ptr<PQP_Model>> models;
  for (int i = 0; i < geoms.size(); i++)
  {
    geoms[i]->generate_collision_geometry();
    models.push_back(geoms[i]->collision_geometry());
  }
  return models;
}

template <typename MovableObjectPlant,
          std::enable_if_t<std::is_base_of_v<MovableObjectPlant, prx::movable_object_t>, bool> = true>
static std::vector<std::shared_ptr<PQP_Model>> create_pqp_models(const std::shared_ptr<MovableObjectPlant>& plant)
{
  std::vector<std::shared_ptr<PQP_Model>> models;
  prx::movable_object_t::Geometries geometries{ plant->get_geometries() };
  for (auto pair : geometries)
  {
    auto geom = pair.second.lock();
    geom->generate_collision_geometry();
    models.push_back(geom->collision_geometry());
  }
  return models;
}

struct query_t
{
  PQP_CollideResult collision_result;
  PQP_DistanceResult distance_result;
  std::vector<std::shared_ptr<PQP_Model>> pqp_models;
  std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> plant_configurations;

  // Additional results of the queries
  // DistanceQuery: norm(P1-P2) = min_distance. P1 is a point in the system, P2 is a point in the obstacle
  Eigen::Vector3d P1;
  Eigen::Vector3d P2;
};

// Check pqp_info against all obstacles. Can be used in threads if inputs are thread-safe

// Using const vector of shared_ptrs... this [potentially] allows multiple threads querying using the same obstacles
inline bool collision(query_t& query, const std::vector<std::shared_ptr<rigid_body_t>> obstacles_bodies)
{
  prx_assert(query.pqp_models.size() == query.plant_configurations.size(),
             "[pqp:collision] mismatch number of models and configurations");

  for (int i = 0; i < query.pqp_models.size(); ++i)
  {
    Eigen::Matrix3d& rotation{ query.plant_configurations[i].first };
    Eigen::Vector3d& translation{ query.plant_configurations[i].second };
    std::shared_ptr<PQP_Model> model{ query.pqp_models[i] };
    for (auto&& obstacle : obstacles_bodies)
    {
      // query.collision_result.FreePairsList();

      PQP_Collide(&query.collision_result,  // no-lint
                  *reinterpret_cast<PQP_REAL(*)[3][3]>(rotation.data()),
                  *reinterpret_cast<PQP_REAL(*)[3]>(translation.data()),            // no-lint
                  model.get(),                                                      // no-lint
                  *reinterpret_cast<PQP_REAL(*)[3][3]>(obstacle->rotation.data()),  // no-lint
                  *reinterpret_cast<PQP_REAL(*)[3]>(obstacle->position.data()),     // no-lint
                  obstacle->model.get(),                                            // no-lint
                  PQP_FIRST_CONTACT);

      if (query.collision_result.Colliding())
      {
        return true;
      }
    }
  }
  return false;
}

inline double minimum_distance(query_t& query, const std::vector<std::shared_ptr<rigid_body_t>> obstacles_bodies)
{
  prx_assert(query.pqp_models.size() == query.plant_configurations.size(),
             "[pqp:collision] mismatch number of models and configurations");
  double min_dist{ std::numeric_limits<double>::max() };
  for (int i = 0; i < query.pqp_models.size(); ++i)
  {
    Eigen::Matrix3d& rotation{ query.plant_configurations[i].first };
    Eigen::Vector3d& translation{ query.plant_configurations[i].second };
    std::shared_ptr<PQP_Model> model{ query.pqp_models[i] };
    for (auto&& obstacle : obstacles_bodies)
    {
      // query.collision_result.FreePairsList();

      PQP_Distance(&query.distance_result,                                           // no-lint
                   *reinterpret_cast<PQP_REAL(*)[3][3]>(rotation.data()),            // no-lint
                   *reinterpret_cast<PQP_REAL(*)[3]>(translation.data()),            // no-lint
                   model.get(),                                                      // no-lint
                   *reinterpret_cast<PQP_REAL(*)[3][3]>(obstacle->rotation.data()),  // no-lint
                   *reinterpret_cast<PQP_REAL(*)[3]>(obstacle->position.data()),     // no-lint
                   obstacle->model.get(),                                            // no-lint
                   0.,                                                               // rel_err
                   0.);                                                              // abs_err

      if (min_dist > query.distance_result.distance)
      {
        min_dist = query.distance_result.distance;
        query.P1 = Eigen::Map<const Eigen::Vector3d>(query.distance_result.P1());
        query.P2 = Eigen::Map<const Eigen::Vector3d>(query.distance_result.P2());
        // min_dist = std::min(min_dist, query.distance_result.distance);
      }
      // if (query.collision_result.Colliding())
      // {
      //   return true;
      // }
    }
  }
  return min_dist;
}

template <typename DynamicalSystem>
class system_checker_t
{
public:
  using DynamicalSystemPtr = std::shared_ptr<DynamicalSystem>;
  using MovableObjectPtr = std::shared_ptr<prx::movable_object_t>;

  system_checker_t(DynamicalSystemPtr system_in, const prx::param_loader& param_loader)
    : system_checker_t(system_in, obstacle_loader_t(param_loader))
  {
  }

  system_checker_t(DynamicalSystemPtr system_in, const prx::obstacle_loader_t& obstacles)
    : system_checker_t(system_in, obstacles.get_obstacles())
  {
  }

  system_checker_t(DynamicalSystemPtr system_in, const std::vector<MovableObjectPtr> in_obstacles)
    : _system(system_in), _obstacles_bodies(create_obstacles(in_obstacles))
  {
    const std::vector<std::shared_ptr<prx::geometry_t>> system_geoms{ system_in->geometries() };
    _query.pqp_models = create_pqp_models(system_geoms);
  }

  virtual ~system_checker_t() {};

  // Assuming a single system (multiple robots *could* be model as a single-complex system)
  template <typename State>
  bool collision(const State& state)
  {
    _query.plant_configurations = _system->configuration(state);
    // prx_assert(_system_pqp_info.configurations.size() == _system_pqp_info.pqp_models.size(),
    //            "PQP geometries and configurations don't match.");

    return prx::collision_checking::pqp::collision(_query, _obstacles_bodies);
  }

  template <typename State>
  double minimum_distance(const State& state)
  {
    _query.plant_configurations = _system->configuration(state);
    return prx::collision_checking::pqp::minimum_distance(_query, _obstacles_bodies);
  }

  query_t query() const
  {
    return _query;
  }

  std::vector<std::shared_ptr<rigid_body_t>> obstacles() const
  {
    return _obstacles_bodies;
  }

protected:
  DynamicalSystemPtr _system;

  query_t _query;
  const std::vector<std::shared_ptr<rigid_body_t>> _obstacles_bodies;
  // PQP_CollideResult _collision_result;

  // std::vector<std::pair<std::weak_ptr<pqp_info_t>, std::weak_ptr<pqp_info_t>>> _collision_cache;
  // std::vector<std::shared_ptr<geometry_t>> _obstacles_bodies;
  // std::vector<rigid_body_t> _obstacles_bodies;

  // std::vector<rigid_body_t> _system_bodies;

  // std::unordered_map<std::string, std::shared_ptr<collision_group_t>> collision_groups;
};
}  // namespace pqp
}  // namespace collision_checking
}  // namespace prx