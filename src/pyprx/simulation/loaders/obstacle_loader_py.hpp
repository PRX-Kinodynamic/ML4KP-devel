#include "prx/simulation/loaders/obstacle_loader.hpp"

void pyprx_simulation_loaders_obstacle_loader()
{
  class_<prx::obstacle_loader_t, std::shared_ptr<prx::obstacle_loader_t>>("obstacle_loader", no_init)
      .def("__init__",
           make_constructor(&init_as_ptr<prx::obstacle_loader_t, const std::string&>, default_call_policies()))
      .def("get_names", &prx::obstacle_loader_t::get_names)
      .def("get_obstacles", &prx::obstacle_loader_t::get_obstacles);

  // class_<std::pair<std::vector<std::string>,std::vector<std::shared_ptr<prx::movable_object_t>>> >("obstacles")
  //    	.def_readwrite("names",  &std::pair<std::vector<std::string>,
  //    std::vector<std::shared_ptr<prx::movable_object_t>>>::first) 	.def_readwrite("objects",
  //    &std::pair<std::vector<std::string>, std::vector<std::shared_ptr<prx::movable_object_t>>>::second)
  //    	;

  // def("load_obstacles", &prx::load_obstacles);

  // iterable_converter()
  //     .from_python<std::vector<std::shared_ptr<prx::movable_object_t>>>()
  //     ;
}
