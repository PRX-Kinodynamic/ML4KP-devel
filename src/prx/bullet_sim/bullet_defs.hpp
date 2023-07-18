#pragma once
#include "prx/utilities/defs.hpp"
#include "prx/simulation/system.hpp"

#define PRINT_BTVECTOR(vec)                                                                                            \
  std::cout << "[btVec "                                                                                               \
            << "] ";                                                                                                   \
  for (int i = 0; i < 3; i++)                                                                                          \
  {                                                                                                                    \
    std::cout << vec.m_floats[i] << " ";                                                                               \
  }                                                                                                                    \
  std::cout << std::endl;
#define PRINT_BTQUAT(quad)                                                                                             \
  std::cout << "[btQuad "                                                                                              \
            << "] ";                                                                                                   \
  std::cout << " " << quad.x() << " " << quad.y() << " " << quad.z() << " " << quad.w();                               \
  std::cout << std::endl;

namespace prx
{
const std::string bullet_path = std::string(BULLET_PHYSICS_PATH);
template <class T>
system_ptr_t create_system(const std::string& path, std::vector<double> start_state)
{
  system_ptr_t new_ptr;
  new_ptr.reset(new T(path, start_state));
  return new_ptr;
}

}  // namespace prx