#pragma once

#include <memory>
#include <unordered_map>
#include <string>
#include "prx/simulation/system.hpp"
#include "prx/simulation/controller.hpp"

namespace prx
{
namespace simulation
{
class time_map_t;
typedef std::function<controller_ptr_t(prx::simulation::time_map_t&)> tm_controller_gen_fn;

class time_map_controllers_t
{
public:
  static time_map_controllers_t& get();

  bool register_system(const std::string name, tm_controller_gen_fn func);

  static tm_controller_gen_fn get_controller(const std::string& name)
  {
    auto it = time_map_controllers_t::get().controller_generators.find(name);
    if (it != time_map_controllers_t::get().controller_generators.end())
    {
      return it->second;
    }
    prx_warn("TimeMap Controller '" << name << "' not found!");
    return it->second;
  };

  static std::vector<std::string> available_systems()
  {
    std::vector<std::string> r;
    for (auto it : time_map_controllers_t::get().controller_generators)
    {
      r.push_back(it.first);
    }
    return r;
  }
  static void print_systems();

private:
  std::unordered_map<std::string, tm_controller_gen_fn> controller_generators;

  // static tm_controller_gen_fn error_function = [](system_ptr_t s) {
  //   PRX_NOT_IMPLEMENTED;
  //   return nullptr;
  // };
};
}  // namespace simulation
}  // namespace prx

#define PRX_REGISTER_TM_CONTROLLER(TM_CONTROLLER_FUNCTION, CONTROLLER_NAME)                                            \
  namespace prx                                                                                                        \
  {                                                                                                                    \
  namespace factory_registration                                                                                       \
  {                                                                                                                    \
  const bool VAR_##CONTROLLER_NAME##_REGISTRED =                                                                       \
      prx::simulation::time_map_controllers_t::get().register_system(#CONTROLLER_NAME, TM_CONTROLLER_FUNCTION);        \
  }                                                                                                                    \
  }
