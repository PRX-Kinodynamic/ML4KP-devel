#include "prx/simulation/loaders/obstacle_loader.hpp"


// std::pair<std::vector<std::string>,std::vector<std::shared_ptr<prx::movable_object_t>>> load_obstacles_1_py(std::string obstacles_file)
// {
// 	return prx::load_obstacles(obstacles_file);
// }


void pyprx_simulation_loaders_obstacle_loader()
{
	

	class_<std::pair<std::vector<std::string>,std::vector<std::shared_ptr<prx::movable_object_t>>> >("obstacles")
    	.def_readwrite("names",  &std::pair<std::vector<std::string>, std::vector<std::shared_ptr<prx::movable_object_t>>>::first)
    	.def_readwrite("objects", &std::pair<std::vector<std::string>, std::vector<std::shared_ptr<prx::movable_object_t>>>::second)
    	;

	def("load_obstacles", &prx::load_obstacles);

    // iterable_converter()
    //     .from_python<std::vector<std::shared_ptr<prx::movable_object_t>>>()
   	//     ;
}