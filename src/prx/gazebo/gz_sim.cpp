#include "prx/gazebo/gz_sim.hpp"



namespace prx
{

	void gz_sim_t::step_simulation(propagate_step step)
	{
		gazebo::runWorld(world, 1);
	}

	void gz_sim_t::reset_simulation()
	{
		world -> Reset();
	}

	void gz_sim_t::initialize_simulation()
	{
		gazebo::setupServer( server_params );

		if (world_file != "")
		{
			world = gazebo::loadWorld(world_file);
		}
	}

	system_ptr_t gz_sim_t::get_plant(std::string name)
	{
		auto models = world -> Models();
		for (auto m : models)
		{
			if (name == m -> GetName())
			{

			}
			// std::cout << "Model: " << m -> GetName() << std::endl;
		}
	}


}