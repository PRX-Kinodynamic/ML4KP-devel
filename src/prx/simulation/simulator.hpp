#pragma once
#include <memory>

#include "prx/utilities/defs.hpp"
#include "prx/simulation/system.hpp"
#include "prx/simulation/system_group.hpp"
// #include "prx/simulation/simulator.hpp"
#include "prx/simulation/system_group_manager.hpp"
#include "prx/simulation/collision_checking/collision_checker.hpp"

namespace prx
{

	typedef std::pair<std::shared_ptr<system_group_t>,std::shared_ptr<collision_group_t>> simulation_context;
	
	// typedef std::pair<std::shared_ptr<system_group_t>,
	// std::shared_ptr<collision_group_t>> world_model_context;

	class simulator_t 
		: public std::enable_shared_from_this<simulator_t>
	{
	public:
		// typedef std::pair<std::shared_ptr<system_group_t>,std::shared_ptr<collision_group_t>>.first system_group;
		// typedef std::pair<std::shared_ptr<system_group_t>,std::shared_ptr<collision_group_t>>.second collision_group;
		// simulator_t(plant_type plants_type);
		simulator_t(plant_type _sim_type) 
			: sim_type(_sim_type)
		{
			system_groups = std::make_shared<system_group_manager_t>();
		}

		~simulator_t() = default;

		// void set_group(const std::vector<system_ptr_t>& sys_group);
		virtual void add_group(const std::vector<system_ptr_t>& all_systems) 
		{
			for(auto s : all_systems)
			{
				systems[s->get_pathname()] = s;
			}
		}

		inline simulation_context get_context(const std::string& context_name)
		{
			return std::make_pair(system_groups->get_system_group(context_name),collision_groups->get_collision_group(context_name));
		}

		inline std::shared_ptr<system_group_t> get_context_system_group(const std::string& context_name)
		{
			return get_context(context_name).first;
		}

		inline std::shared_ptr<collision_group_t> get_context_collision_group(const std::string& context_name)
		{
			return get_context(context_name).second;
		}

		std::shared_ptr<simulator_t> shared_ptr()
		{
        	return this -> shared_from_this();
    	}
    	
		virtual void step_simulation(propagate_step step) = 0;

		virtual void reset_simulation() = 0;

		const plant_type sim_type;

	protected:

		std::shared_ptr<system_group_manager_t> system_groups;
		// QUESTION: Should we rename collision_groups -> collision_checker or something similar?
		// 			 the current name is confusing because there is another class 'collision_group'
		std::shared_ptr<collision_checker_t> collision_groups;

		std::unordered_map<std::string,system_ptr_t> systems;

		// std::vector<system_ptr_t> group;
	};
}
