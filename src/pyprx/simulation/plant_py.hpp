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

using prx::plant_t;

struct plant_wrap : plant_t, wrapper<plant_t>
{
  plant_wrap(const plant_t& _plant) : plant_t(_plant){};

  plant_wrap(const std::string& path) : plant_t(path){};

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
bool (prx::plant_t::*linearize_2)(Eigen::MatrixXd&, Eigen::MatrixXd&) = &prx::plant_t::linearize;

void bindings()
{
  class_<plant_wrap, bases<prx::system_t, prx::movable_object_t>, boost::noncopyable>("plant_t", init<std::string>())
      .def("add_system", &plant_t::add_system)
      .def("propagate", &plant_t::propagate)
      .def("compute_stopping_maneuver", &plant_t::compute_stopping_maneuver)
      .def("compute_control", &plant_t::compute_control)
      .def("update_configuration", pure_virtual(&plant_t::update_configuration))
      .def("compute_derivative", pure_virtual(&plant_wrap::compute_derivative))
      .def("set_integrator", &plant_t::set_integrator)
      .def("set_state_space_bounds", &plant_t::set_state_space_bounds)
      .def("linearize", linearize_2)
      // Comment to force ; to the next one
      ;

  iterable_converter().from_python<std::vector<prx::system_ptr_t> >();
  // pyprx_simulation_plants_acrobot();
}
}  // namespace plant
}  // namespace simulation
}  // namespace pyprx