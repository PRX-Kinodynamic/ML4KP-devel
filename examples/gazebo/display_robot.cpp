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
    // printf("GAZEBO_MODEL_PATH: %s\n", std::getenv("GAZEBO_MODEL_PATH") );
    // printf("GAZEBO_MODEL_PATH_SET: %s\n", GAZEBO_MODEL_PATH_SET?"True":"False" );
    // printf("GAZEBO_RESOURCE_PATH_SET: %s\n", GAZEBO_RESOURCE_PATH_SET?"True":"False" );
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
        auto c = std::cin.get();
        if (c == 'q')
        {
            //! desired user input 'q' received
            keep_running = false;
        }
        else if (c == 's')
        {
            std::cout << "Stepping..." << std::endl;
        	sim -> step_simulation(1000);
            std::cout << "done stepping!" << std::endl;
        }
        else if (c == 'r')
        {
            sim -> reset_simulation();
        }
        else if (c == 'm')
        {
            gazebo::physics::ModelState m = sim -> get_model_state("pendulum_gz");
            std::cout << "state: " << m.GetJointStateCount() << std::endl;
        }
    }

    std::cout << "DONE!" << std::endl;

}