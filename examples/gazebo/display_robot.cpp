// #include <conio.h>
#include <iostream>

#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/rrt.hpp"
#include "prx/gazebo/gz_sim.hpp"


using namespace prx;

int main(int argc, char* argv[])
{
	auto sim = std::make_shared<gz_sim_t>();

    sim -> set_world(worlds_path + "pendulum.world");

    std::string plant_name = "pendulum_gz";//params["/plant/name"].as<>();
    std::string plant_path = "pendulum_gz";//params["/plant/path"].as<>();

    auto plant = prx::system_factory_t::create_system_as<plant_gz_wrapper_t>(plant_name, plant_path);
    std::cout << "plant created as: " << typeid(plant).name() << plant << std::endl;
    prx_assert(plant != nullptr, "Plant is nullptr!");

    sim -> add_group({plant});
    sim -> create_context({plant_name}, {});
    sim -> initialize_simulation();
    
    auto ss = plant -> get_state_space();
    std::cout << plant -> get_pathname() << " space " << ss << std::endl;
    auto state = ss -> make_point();

	bool keep_running = true;

    std::cout<<"press q to exit! "<<std::endl;
    gazebo::physics::ModelPtr m = sim -> get_model_ptr("pendulum_gz");
    while(keep_running) 
    {
        auto c = std::cin.get();
        if (c == 'q')
        {
            keep_running = false;
        }
        else if (c == 's')
        {
            std::cout << "Stepping..." << std::endl;
            for (int i = 0; i < 100; ++i)
            {
                sim -> step_simulation(10);
            }
            std::cout << "done stepping!" << std::endl;
        }
        else if (c == 'r')
        {
            sim -> reset_simulation();
        }
        else if (c == 'm')
        {
            auto m_ptr = sim -> get_model_ptr("pendulum_gz");
        }
        else if (c == 'e')
        {   
            ss -> sample(state);
            std::cout << "sample state: " << state << std::endl;
            ss -> copy_from_point(state);
        }
    }

    std::cout << "DONE!" << std::endl;

}