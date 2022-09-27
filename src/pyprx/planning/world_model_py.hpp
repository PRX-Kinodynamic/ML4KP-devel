#include "prx/planning/world_model.hpp"
#include "prx/simulation/system_group.hpp"
#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>
#include <iostream>

using namespace boost::python;
// clang-format off

void pyprx_planning_world_model_py()
{
	
	// class_<std::vector<std::shared_ptr<prx::movable_object_t>>>("vector_movable_object")
 //   		.def(vector_indexing_suite<std::vector<std::shared_ptr<prx::movable_object_t>>>())
	// 	;


	class_<std::pair<std::shared_ptr<prx::system_group_t>, std::shared_ptr<prx::collision_group_t>> >("context")
    	.def_readwrite("system_group",  &std::pair<std::shared_ptr<prx::system_group_t>, std::shared_ptr<prx::collision_group_t>>::first)
    	.def_readwrite("collision_group", &std::pair<std::shared_ptr<prx::system_group_t>, std::shared_ptr<prx::collision_group_t>>::second)
    	;

   	class_<prx::world_model_t>("world_model", init<std::vector<prx::system_ptr_t>, std::vector<std::shared_ptr<prx::movable_object_t>>>())
   	    .def("create_context", &prx::world_model_t::create_context)
   	    .def("get_context",    &prx::world_model_t::get_context)
   	    ;
   	
}