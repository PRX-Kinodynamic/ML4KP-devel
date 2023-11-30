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

class obstacle_loader_t
{
public:
  static std::pair<std::vector<std::string>, std::vector<std::shared_ptr<movable_object_t>>>
  load_obstacles_from_file(const std::string obstacles_file);

  obstacle_loader_t(const std::string obstacles_file)
  {
    auto loaded_obst = obstacle_loader_t::load_obstacles_from_file(obstacles_file);
    names = loaded_obst.first;
    obstacles = loaded_obst.second;
  }

  const std::vector<std::string> get_names()
  {
    return names;
  }

  const std::vector<std::shared_ptr<movable_object_t>> get_obstacles()
  {
    return obstacles;
  }

private:
  std::vector<std::string> names;
  std::vector<std::shared_ptr<movable_object_t>> obstacles;
};

using PairNameObstacles = std::pair<std::vector<std::string>, std::vector<std::shared_ptr<movable_object_t>>>;
inline PairNameObstacles load_obstacles(const std::string obstacles_file)
{
  return obstacle_loader_t::load_obstacles_from_file(obstacles_file);
}

}  // namespace prx
