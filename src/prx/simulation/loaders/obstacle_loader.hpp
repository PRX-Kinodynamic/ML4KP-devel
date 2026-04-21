#pragma once
/**
 * @file obstacle_loader.hpp
 * @brief <b>A loader that loads obstacles from a file.</b>
 * @authors Zakary Littlefield, Aravind Sivaramakrishnan, Troy McMahon, Edgar Granados
 */

#include "prx/utilities/defs.hpp"
#include "prx/utilities/geometry/movable_object.hpp"

namespace prx
{
/**
 * @brief <b>A loader that loads obstacles from a file.</b>
 * @param obstacles_file Name of the file to load obstacles from.
 * @return A mapping from obstacle names to pointers to the bodies.
 */

using PairNameObstacles = std::pair<std::vector<std::string>, std::vector<std::shared_ptr<movable_object_t>>>;
using EnvironmentBounds = std::pair<Eigen::Vector3d, Eigen::Vector3d>;
class obstacle_loader_t
{
public:
  static PairNameObstacles load_obstacles_from_file(const std::string);
  static PairNameObstacles load_obstacles_from_file(const param_loader&);

  static EnvironmentBounds bounds_from_yaml(const std::string);
  static EnvironmentBounds bounds_from_yaml(const param_loader&);

  obstacle_loader_t() : _bounds({ 100 * Eigen::Vector3d::Ones(), 100 * Eigen::Vector3d::Ones() })
  {
  }
  obstacle_loader_t(const std::string obstacles_file) : obstacle_loader_t()
  {
    auto loaded_obst = obstacle_loader_t::load_obstacles_from_file(obstacles_file);
    names = loaded_obst.first;
    obstacles = loaded_obst.second;
    _bounds = obstacle_loader_t::bounds_from_yaml(obstacles_file);
  }

  obstacle_loader_t(const param_loader& obstacles_params) : obstacle_loader_t()
  {
    auto loaded_obst = obstacle_loader_t::load_obstacles_from_file(obstacles_params);
    names = loaded_obst.first;
    obstacles = loaded_obst.second;
    _bounds = obstacle_loader_t::bounds_from_yaml(obstacles_params);
  }

  std::vector<std::string> get_names() const
  {
    return names;
  }

  std::vector<std::shared_ptr<movable_object_t>> get_obstacles() const
  {
    return obstacles;
  }

  EnvironmentBounds bounds() const
  {
    return _bounds;
  }

  Eigen::Vector3d min_bounds() const
  {
    return _bounds.first;
  }

  Eigen::Vector3d max_bounds() const
  {
    return _bounds.second;
  }

private:
  std::vector<std::string> names;
  std::vector<std::shared_ptr<movable_object_t>> obstacles;
  EnvironmentBounds _bounds;
};

inline PairNameObstacles load_obstacles(const std::string obstacles_file)
{
  return obstacle_loader_t::load_obstacles_from_file(obstacles_file);
}

}  // namespace prx
