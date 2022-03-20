// #include <conio.h>
#include <iostream>

#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/rrt.hpp"
#include "prx/gazebo/gz_sim.hpp"


using namespace prx;

int main(int argc, char* argv[])
{
	// gazebo::setupServer({});
	auto sim = std::make_shared<gz_sim_t>();

  // Load a world
  // gazebo::physics::WorldPtr world = gazebo::loadWorld(worlds_path + "pendulum.world");
	sim -> set_world(worlds_path + "pendulum.world");
  // This is your custom main loop. In this example the main loop is just a
  // for loop with 2 iterations.

  // Close everything.
	sim -> initialize_simulation();
	bool keep_running = true;

	// auto models = sim -> world -> Models();
	// for (auto m : models)
	// {
	// 	std::cout << "Model: " << m -> GetName() << std::endl;
	// }

    std::cout<<"press q to exit! "<<std::endl;
    while(keep_running) 
    {
        if (std::cin.get() == 'q')
        {
            //! desired user input 'q' received
            keep_running = false;
        }
        if (std::cin.get() == 'r')
        {
        	// gazebo::runWorld(world, 100);
        }
    }

    std::cout << "DONE!" << std::endl;

}