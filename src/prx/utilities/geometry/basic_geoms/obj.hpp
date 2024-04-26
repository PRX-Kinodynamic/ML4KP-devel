#pragma once
/**
 * @file obj.hpp
 * @brief <b> A generic object.</b>
 *
 * This file contains the definition of a generic object.
 *
 * @author Aravind Sivaramakrishnan
 */
#include "prx/utilities/geometry/movable_object.hpp"

namespace prx
{
/**
 * @brief <b> A generic object.</b>
 *
 * This class is a generic object that can be used to represent any object.
 *
 * @author Aravind Sivaramakrishnan
 */
class obj_t : public movable_object_t
{
public:
  /**
   * @brief <b> Constructor.</b>
   *
   * Initializes a generic object.
   *
   * @param object_name Name of the object.
   * @param obj_fname Filename of the object.
   * @param pose The pose of the object.
   *
   */
  obj_t(const std::string& object_name, const std::string obj_fname, const transform_t& pose);
  virtual ~obj_t()
  {
  }
};
}  // namespace prx