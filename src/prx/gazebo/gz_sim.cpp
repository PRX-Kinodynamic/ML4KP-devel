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
		gazebo::physics::WorldState ws(world);
		for(auto s_pair : systems)
		{	
			auto sys_name = s_pair.first;
			auto sys = s_pair.second;
			// std::dynamic_pointer_cast<plant_gz_t>(s)
			auto pw = std::dynamic_pointer_cast<plant_gz_wrapper_t>(sys) -> get_plant_gz();
			// pw -> get_plant_gz() -> set_model_ptr(get_model_ptr(sys_name));
			// auto m_ptr = world -> ModelByName(sys_name);
			pw -> copy_to_model_ptr();
			pw -> update();
			// ws.GetModelState(plant_name);
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
			// auto gz_sys = std::dynamic_pointer_cast<plant_gz_t<system_t>>(sys);
			// auto m_ptr = world -> ModelByName(sys_name);
			pw -> copy_from_model_ptr();
		}

	}

	void gz_sim_t::add_group(const std::vector<system_ptr_t>& all_systems)
	// void gz_sim_t::add_group(const std::vector<std::shared_ptr<plant_gz_t<system_t>>>& all_systems)
	{
		// std::vector<system_ptr_t>
		for(auto s : all_systems)
		{
			// std::cout << typeid(s).name() << " " << s->get_pathname() << std::endl;
			// auto st = s.get();
			// std::cout << typeid(st).name() << std::endl;
			// auto pgz = dynamic_cast<pendulum_gz_t*>(st);
			// std::cout << typeid(pgz).name() << "\tplant: " << pgz << std::endl;
			// auto pgzp = dynamic_cast<plant_gz_t<pendulum_t>*>(st);
			// std::cout << typeid(pgzp).name() << "\tplant: " << pgzp << std::endl;
			// auto pgzt = dynamic_cast<plant_gz_t<plant_t>*>(st);
			// std::cout << typeid(pgzt).name() << "\tplant: " << pgzt << std::endl;
			// auto pend = dynamic_cast<pendulum_t*>(st);
			// std::cout << typeid(pend).name() << "\tpend: " << pend << std::endl;
			// auto syst = dynamic_cast<system_t*>(pend);
			// std::cout << typeid(syst).name() << "\tsyst: " << syst << std::endl;

			// std::cout << "is plant? " << get_plant_gz() << std::endl;
			// prx_assert(pgzt != nullptr, 
			// 	"Plant " << s -> get_pathname()  << " is not of type 'plant_gz_t'");
			auto pw = std::dynamic_pointer_cast<plant_gz_wrapper_t>(s);
			prx_assert(pw != nullptr, 
				"Plant " << s -> get_pathname()  << " is not of type 'plant_gz_t'");
			// pw -> get_plant_gz() -> set_model_ptr();

		}
		std::cout << "add group!!" << std::endl;
		this -> simulator_t::add_group(all_systems);
		std::cout << "group added!!" << std::endl;

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
			// std::cout << "Model: " << m -> GetName() << std::endl;
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
		// if (plant_name == m -> GetName())
		// {
		return ws.GetModelState(plant_name);
		// }
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
		// for(auto&& o : obstacle_names)
		// {
		// 	context_obstacles.push_back(obstacles[o]);
		// }

		system_groups->add_system_group(context_name,context_systems);
			// collision_groups->add_collision_group(context_name,context_systems,context_obstacles);
		
	}

}