#include <iostream>
#include <boost/python.hpp>
#include "prx/simulation/system.hpp"
#include "prx/simulation/plant.hpp"

using namespace boost::python;

namespace pyprx
{
namespace simulation
{
namespace plant
{

struct plant_wrap : prx::plant_t, wrapper<prx::plant_t>
{
  plant_wrap(const plant_t& _plant) : prx::plant_t(_plant){};

  plant_wrap(const std::string& path) : prx::plant_t(path){};

  void update_configuration()
  {
    this->get_override("update_configuration")();
  }

public:
  void compute_derivative()
  {
    this->get_override("compute_derivative")();
    // this -> compute_derivative();
  }
};

PRX_SETTER(plant_t, derivative_space)
PRX_GETTER(plant_t, derivative_space)

// PRX_SETTER(plant_t, derivative_memory)
// PRX_GETTER(plant_t, derivative_memory)

void bindings()
{
  class_<plant_wrap, bases<prx::system_t, prx::movable_object_t>, boost::noncopyable>("plant_t", init<std::string>())
      .def("add_system", &prx::plant_t::add_system)
      .def("propagate", &prx::plant_t::propagate)
      .def("compute_stopping_maneuver", &prx::plant_t::compute_stopping_maneuver)
      .def("compute_control", &prx::plant_t::compute_control)
      .def("update_configuration", pure_virtual(&prx::plant_t::update_configuration))
      .def("compute_derivative", pure_virtual(&plant_wrap::compute_derivative))
      .def("set_integrator", &prx::plant_t::set_integrator)
      .def("set_state_space_bounds", &prx::plant_t::set_state_space_bounds)
      // Comment to force ; to the next one
      ;

  iterable_converter().from_python<std::vector<prx::system_ptr_t> >();
  // pyprx_simulation_plants_acrobot();
}
}  // namespace plant
}  // namespace simulation
}  // namespace pyprx