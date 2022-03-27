#include "prx/gazebo/gz_sim.hpp"



namespace prx
{

	void gz_sim_t::step_simulation(propagate_step step)
	{
		// Do we need to check the type of step in GZ?
		gazebo::runWorld(world, 1);
		// world -> Run(1);
	}

	void gz_sim_t::step_simulation(int steps)
	{
		for (int i = 0; i < steps; ++i)
		{
			step_simulation(propagate_step::MIDDLE_STEP);
		}
	}

	void gz_sim_t::reset_simulation()
	{
		world -> Reset();
	}

	void gz_sim_t::initialize_simulation()
	{
// PRX_DEBUG_PRINT
		// server_params.push_back(world_file);
		gazebo::setupServer( server_params );
// PRX_DEBUG_PRINT
		if (world_file != "")
		{
// PRX_DEBUG_PRINT
			world = gazebo::loadWorld(world_file);
			auto models = world -> Models();
			for (auto m : models)
			{
				m -> LoadPlugins();
			}
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
		return nullptr;
	}

	gazebo::physics::ModelState gz_sim_t::get_model_state(std::string plant_name)
	{
		gazebo::physics::WorldState ws(world);
		// if (plant_name == m -> GetName())
		// {
		return ws.GetModelState(plant_name);
		// }
	}


}