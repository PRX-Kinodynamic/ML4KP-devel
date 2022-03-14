#include "prx/utilities/defs.hpp"
// #include "prx/bullet_sim/plants/husky.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/rrt.hpp"
#include "prx/bullet_sim/plants/plants.hpp"
#include "prx/bullet_sim/collision_checking/collision_checker.hpp"
#include "prx/bullet_sim/bullet_simulator.hpp"
#include "Utils/b3Clock.h"

using namespace prx;

int main(int argc, char* argv[])
{
	try
	{
    	auto params = param_loader("examples/bullet/display_robot.yaml", argc, argv);

		simulation_step = params["simulation_step"].as<double>();
		
		// auto plant = std::dynamic_pointer_cast<bullet_omnirobot_t>(create_system<bullet_omnirobot_t>("racecar"));
		std::string plant_name = params["/plant/name"].as<>();
		auto system = system_factory_t::create_system(plant_name, plant_name);
		auto plant = std::dynamic_pointer_cast<bullet_plant_t>(system);
		prx_assert(plant != nullptr, "Plant is not a bullet_plant!");

		std::shared_ptr<bullet_simulator_t> sim = std::make_shared<bullet_simulator_t>();
		// auto sim = bsim.get_ptr();
    	// bsim.add_urdf("/Users/Gary/pracsys/bullet3/data/plane.urdf");
    	sim -> add_urdf(bullet_path + "/data/plane.urdf");
    	
		sim -> add_group({plant});
		sim -> initialize_simulation();

		int rotateCamera = 0;
		btScalar fixedTimeStep = simulation_step;

		sim->setRealTimeSimulation(false);

		while (sim->canSubmitCommand())
		{
			sim -> step_simulation(simulation_step);
		// 	b3KeyboardEventsData keyEvents;
		// 	sim->getKeyboardEvents(&keyEvents);
		// 	if (keyEvents.m_numKeyboardEvents)
		// 	{
		// 		for (int i = 0; i < keyEvents.m_numKeyboardEvents; i++)
		// 		{
		// 			b3KeyboardEvent& e = keyEvents.m_keyboardEvents[i];

		// 			if (e.m_keyCode == 'r' && e.m_keyState & eButtonTriggered)
		// 			{
		// 				rotateCamera = 1 - rotateCamera;
		// 			}

		// 		}
		// 	}
		// 	sim->stepSimulation();

		// 	if (rotateCamera)
		// 	{
		// 		static double yaw = 0;
		// 		double distance = 1;
		// 		yaw += 0.1;
		// 		btVector3 basePos;
		// 		btQuaternion baseOrn;
		// 		// sim->getBasePositionAndOrientation(minitaurUid, basePos, baseOrn);
		// 		sim->resetDebugVisualizerCamera(distance, -20, yaw, basePos);
		// 	}
		// 	b3Clock::usleep(1000. * 1000. * fixedTimeStep);
		}

		std::cout << "Vis done!" << std::endl;

	}
	catch(const prx_assert_t& e)
	{
		std::cout<<e.get_message()<<std::endl;
	}
	std::cout<<"End of program"<<std::endl;
}

