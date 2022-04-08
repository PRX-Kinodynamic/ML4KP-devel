#include "prx/gazebo/gz_sim.hpp"



namespace prx
{
	// ToDo: does it makes sense to handle multiple propagate_step?
	// What was the reason for having this in bullet? 
	void gz_sim_t::step_simulation(propagate_step step)
	{
		gazebo::runWorld(world, 1);
	}

	void gz_sim_t::step_simulation(int steps)
	{
		gazebo::physics::WorldState ws(world);
		for(auto s_pair : systems)
		{	
			auto sys_name = s_pair.first;
			auto sys = s_pair.second;
			auto pw = std::dynamic_pointer_cast<plant_gz_wrapper_t>(sys) -> get_plant_gz();
			pw -> copy_to_model_ptr();
			pw -> update();
		}
		for (int i = 0; i < steps; ++i)
		{
			step_simulation(propagate_step::MIDDLE_STEP);
		}

		for(auto s_pair : systems)
		{	
			auto sys_name = s_pair.first;
			auto sys = s_pair.second;
			auto pw = std::dynamic_pointer_cast<plant_gz_wrapper_t>(sys) -> get_plant_gz();
			pw -> copy_from_model_ptr();
		}

	}

	void gz_sim_t::add_group(const std::vector<system_ptr_t>& all_systems)
	{
		for(auto s : all_systems)
		{
			auto pw = std::dynamic_pointer_cast<plant_gz_wrapper_t>(s);
			// ToDo: it would be nicer to have a map-like assert for containers...
			prx_assert(pw != nullptr, 
				"Plant " << s -> get_pathname()  << " is not of type 'plant_gz_t'");
		}
		std::cout << "add group!!" << std::endl;
		this -> simulator_t::add_group(all_systems);
		std::cout << "group added!!" << std::endl;

	}

	void gz_sim_t::reset_simulation()
	{
		for(auto s_pair : systems)
		{	
			auto sys = s_pair.second;
			auto pw = std::dynamic_pointer_cast<plant_gz_wrapper_t>(sys) -> get_plant_gz();
			pw -> reset_system();
		}
		world -> Reset();

	}

	void gz_sim_t::initialize_simulation()
	{
		gazebo::setupServer( server_params );
		if (world_file != "")
		{
			world = gazebo::loadWorld(world_file);
			auto models = world -> Models();
			for (auto m : models)
			{
				m -> LoadPlugins();
			}
		}
		for(auto s_pair : systems)
		{	
			auto sys_name = s_pair.first;
			auto sys = s_pair.second;
			auto pw = std::dynamic_pointer_cast<plant_gz_wrapper_t>(sys);
			pw -> get_plant_gz() -> set_model_ptr(get_model_ptr(sys_name));
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
		}
		return nullptr;
	}

	gazebo::physics::ModelPtr gz_sim_t::get_model_ptr(const std::string& plant_name)
	{
		return world -> ModelByName(plant_name);
	}

	gazebo::physics::ModelState gz_sim_t::get_model_state(std::string plant_name)
	{
		gazebo::physics::WorldState ws(world);
		return ws.GetModelState(plant_name);
	}

	// TODO: Handle obstacles
	void gz_sim_t::create_context(const std::vector<std::string>& system_names, 
			const std::vector<std::string>& obstacle_names, 
			const std::string& context_name)
	{
		prx_assert(all_context_names.count(context_name) == 0, 
			"Duplicated context name: " << context_name << ". Context names must be unique.")
		system_groups -> link_simulator(this);

		all_context_names.insert(context_name);
		std::vector<system_ptr_t> context_systems;
		// std::vector<std::shared_ptr<movable_object_t>> context_obstacles;
		for(auto&& s : system_names)
		{
			std::cout << "system: " << s << std::endl;
			context_systems.push_back(this -> systems[s]);
		}
		// TODO: Obstacles?
		// for(auto&& o : obstacle_names)
		// {
		// 	context_obstacles.push_back(obstacles[o]);
		// }

		system_groups->add_system_group(context_name,context_systems);
		// collision_groups->add_collision_group(context_name,context_systems,context_obstacles);
		
	}

}