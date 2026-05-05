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
struct pqp_distance_t
{
  /**
   * @brief A vector of distances between each collision pair in the collision cache.
   * */
  std::vector<double> distances;
  /**
   * @brief A vector of the closest point for each collision pair in the collision cache.
   *
   * For every element of the collision pair, the closest point corresponds to the point on the
   * first element of the collision pair that is closest to the second element of the collision pair.
   * By convention, when constructing the collision cache, the second element of each pair typically
   * corresponds to a rigid body on a robot, while the first element corresponds to a rigid body that is
   * considered to be an obstacle.
   *
   * As a result, this is a vector of the closest point on each obstacle for each rigid body present on the robot.
   * */
  std::vector<Eigen::Vector3d> closest_points;
};

struct pqp_info_t
{
  /** @brief The (x,y,z) position of the center of mass. */
  // double position[3];
  Eigen::Vector3d position;
  /** @brief The rotation matrix of the center of mass. */
  // double rotation[3][3];
  Eigen::Matrix3d rotation;
  // std::weak_ptr<transform_t> transform;
  std::weak_ptr<PQP_Model> model;

  /** @brief Updates the poses for the geometry. */
  template <typename MovableObjectPlant,
            std::enable_if_t<std::is_base_of_v<MovableObjectPlant, prx::movable_object_t>, bool> = true>
  static std::vector<std::shared_ptr<pqp_info_t>>
  from_obstacles(const std::vector<std::shared_ptr<MovableObjectPlant>> obstacles)
  {
    std::vector<std::shared_ptr<pqp_info_t>> obstacles_pqp_infos;
    for (auto&& object : obstacles)
    {
      auto geoms = object->get_geometries();
      auto configs = object->get_configurations();
      for (int i = 0; i < geoms.size(); i++)
      {
        prx_assert(geoms[i].first == configs[i].first,
                   "Geometry and configuration lists don't match in " << object->get_object_name());
        auto g = geoms[i].second;
        auto config = configs[i].second;
        auto g_ptr = g.lock();
        auto info = std::make_shared<pqp_info_t>();
        info->model = g_ptr->get_collision_geometry();
        // info->transform = config;
        info->rotation = config.lock()->linear();
        info->position = config.lock()->translation();
        obstacles_pqp_infos.push_back(info);
      }
    }
    return obstacles_pqp_infos;
  }
};

struct system_pqp_info_t
{
  std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> configurations;
  std::vector<std::shared_ptr<PQP_Model>> pqp_models;

  // named constructor
  static system_pqp_info_t from_geometries(const std::vector<std::shared_ptr<prx::geometry_t>>& geoms)
  {
    system_pqp_info_t pqp_info;
    for (int i = 0; i < geoms.size(); i++)
    {
      geoms[i]->generate_collision_geometry();
      pqp_info.pqp_models.push_back(geoms[i]->collision_geometry());
    }
    return pqp_info;
  }
  // Named constructor for movable_object_t
  // std::is_base_of_v<A, C>
  template <typename MovableObjectPlant,
            std::enable_if_t<std::is_base_of_v<MovableObjectPlant, prx::movable_object_t>, bool> = true>
  static system_pqp_info_t from_geometries(const std::shared_ptr<MovableObjectPlant>& plant)
  {
    system_pqp_info_t pqp_info;
    prx::movable_object_t::Geometries geometries{ plant->get_geometries() };
    for (auto pair : geometries)
    {
      auto geom = pair.second.lock();
      geom->generate_collision_geometry();
      pqp_info.pqp_models.push_back(geom->collision_geometry());
    }
    return pqp_info;
  }
};

// Check pqp_info against all obstacles. Can be used in threads if inputs are thread-safe
inline bool collision(PQP_CollideResult& result, system_pqp_info_t& pqp_info,
                      const std::vector<std::shared_ptr<pqp_info_t>>& obstacles_pqp_infos)
{
  for (int i = 0; i < pqp_info.configurations.size(); ++i)
  {
    Eigen::Matrix3d& rotation{ pqp_info.configurations[i].first };
    Eigen::Vector3d& translation{ pqp_info.configurations[i].second };

    for (auto&& obstacle : obstacles_pqp_infos)
    {
      result.FreePairsList();

      PQP_Collide(&result,  // no-lint
                  *reinterpret_cast<PQP_REAL(*)[3][3]>(rotation.data()),
                  *reinterpret_cast<PQP_REAL(*)[3]>(translation.data()),            // no-lint
                  pqp_info.pqp_models[i].get(),                                     // no-lint
                  *reinterpret_cast<PQP_REAL(*)[3][3]>(obstacle->rotation.data()),  // no-lint
                  *reinterpret_cast<PQP_REAL(*)[3]>(obstacle->position.data()),     // no-lint
                  obstacle->model.lock().get(),                                     // no-lint
                  PQP_FIRST_CONTACT);

      if (result.Colliding())
      {
        return true;
      }
    }
  }
  return false;
}

template <typename DynamicalSystem>
class pqp_checker_t
{
public:
  using DynamicalSystemPtr = std::shared_ptr<DynamicalSystem>;
  using MovableObjectPtr = std::shared_ptr<prx::movable_object_t>;

  pqp_checker_t(DynamicalSystemPtr system_in, const prx::obstacle_loader_t& obstacles)
    : pqp_checker_t(system_in, obstacles.get_obstacles())
  {
  }

  pqp_checker_t(DynamicalSystemPtr system_in, const std::vector<MovableObjectPtr> in_obstacles) : _system(system_in)
  {
    _obstacles_pqp_infos = pqp_info_t::from_obstacles(in_obstacles);

    const std::vector<std::shared_ptr<prx::geometry_t>> system_geoms{ system_in->geometries() };

    _system_pqp_info = system_pqp_info_t::from_geometries(system_geoms);
  }

  virtual ~pqp_checker_t() {};

  // Assuming a single system (multiple robots *could* be model as a single-complex system)
  template <typename State>
  bool collision(const State& state)
  {
    _system_pqp_info.configurations = _system->configuration(state);
    prx_assert(_system_pqp_info.configurations.size() == _system_pqp_info.pqp_models.size(),
               "PQP geometries and configurations don't match.");

    return prx::collision_checking::collision(_collision_result, _system_pqp_info, _obstacles_pqp_infos);
  }

  // TODO:
  // pqp_distance_t get_distances()
  // {
  // }

protected:
  PQP_CollideResult _collision_result;

  // std::vector<std::pair<std::weak_ptr<pqp_info_t>, std::weak_ptr<pqp_info_t>>> _collision_cache;
  std::vector<std::shared_ptr<pqp_info_t>> _obstacles_pqp_infos;

  DynamicalSystemPtr _system;
  system_pqp_info_t _system_pqp_info;

  // std::unordered_map<std::string, std::shared_ptr<collision_group_t>> collision_groups;
};
}  // namespace collision_checking
}  // namespace prx