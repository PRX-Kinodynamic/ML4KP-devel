#pragma once

#include "prx/utilities/defs.hpp"

#include <memory>

namespace prx
{
namespace simulation
{
class sensor_t;
// class system_ptr_t;
using sensor_ptr_t = std::shared_ptr<sensor_t>;

class sensor_t : public std::enable_shared_from_this<sensor_t>
{
public:
  sensor_t(const std::string& name) : _sensor_name(name)
  {
  }

  virtual ~sensor_t()
  {
  }

  const std::string get_name()
  {
    return _sensor_name;
  }

  virtual void sense() = 0;

  space_t* get_sensor_space()
  {
    return sensor_space;
  }

protected:
  sensor_t(){};

  std::string _sensor_name;

  space_t* sensor_space;

  std::vector<double*> sensor_memory;
};

}  // namespace simulation
}  // namespace prx