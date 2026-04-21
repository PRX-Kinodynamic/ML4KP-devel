#pragma once

#include <memory>
#include <unordered_map>

#include "PQP/PQP_Compile.h"
#include "general/debug_utils.hpp"
#include "general/param_loader.hpp"
#include "general/prx_assert.hpp"
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
  // void update_info();
};

struct system_pqp_info_t
{
  std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> configurations;
  std::vector<std::shared_ptr<PQP_Model>> pqp_models;
};

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

  pqp_checker_t(DynamicalSystemPtr system_in, const std::vector<MovableObjectPtr> in_obstacles)
  {
    for (auto&& object : in_obstacles)
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
        _obstacles_pqp_infos.push_back(info);
      }
    }

    const std::vector<std::shared_ptr<prx::geometry_t>> system_geoms{ system_in->geometries() };
    for (int i = 0; i < system_geoms.size(); i++)
    {
      // _system_pqp_info.pqp_models.push_back()
      system_geoms[i]->generate_collision_geometry();
      _system_pqp_info.pqp_models.push_back(system_geoms[i]->collision_geometry());
    }
  }

  virtual ~pqp_checker_t() {};

  // Assuming a single system (multiple robots *could* be model as a single-complex system)
  template <typename State>
  bool collision(const State& state)
  {
    _system_pqp_info.configurations = _system->configuration(state);
    prx_assert(_system_pqp_info.configurations.size() == _system_pqp_info.pqp_models.size(),
               "PQP geometries and configurations don't match.");

    for (int i = 0; i < _system_pqp_info.configurations.size(); ++i)
    {
      Eigen::Matrix3d& rotation{ _system_pqp_info.configurations[i].first };
      Eigen::Vector3d& translation{ _system_pqp_info.configurations[i].second };

      // auto system_model = _system_pqp_info.pqp_models[i].lock().get();
      // PRX_DBG_VARS(i, system_model)

      for (auto&& obstacle : _obstacles_pqp_infos)
      {
        _collision_result.FreePairsList();

        // PQP_REAL t[3]{ *translation.data() };
        ;
        // reinterpret_cast<PQP_REAL*[3][3]>(rotation.data());
        PQP_Collide(&_collision_result,  // no-lint
                    *reinterpret_cast<PQP_REAL(*)[3][3]>(rotation.data()),
                    *reinterpret_cast<PQP_REAL(*)[3]>(translation.data()),            // no-lint
                    _system_pqp_info.pqp_models[i].get(),                             // no-lint
                    *reinterpret_cast<PQP_REAL(*)[3][3]>(obstacle->rotation.data()),  // no-lint
                    *reinterpret_cast<PQP_REAL(*)[3]>(obstacle->position.data()),     // no-lint
                    obstacle->model.lock().get(),                                     // no-lint
                    PQP_FIRST_CONTACT);

        if (_collision_result.Colliding())
        {
          return true;
        }
      }
    }

    return false;
  }
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