#pragma once

#include "prx/simulation/system.hpp"

#include "prx/utilities/geometry/movable_object.hpp"

#include <bitset>


#include "prx/simulation/integrators/integrator.hpp"
#include "prx/simulation/integrators/euler.hpp"
#include "prx/simulation/integrators/dopri5.hpp"
#include "prx/simulation/integrators/runge_kutta4.hpp"

namespace prx
{
	class system_factory_t;
	
	class plant_t : public system_t, public movable_object_t
	{
	public:
		plant_t(const plant_t& _plant) = default;
		plant_t(const std::string& path);
		virtual ~plant_t();

		virtual void add_system(system_ptr_t& ) override final;

		virtual void propagate(const double simulation_step) override;

		virtual void compute_stopping_maneuver(space_point_t, std::vector<double>&, std::vector<double>&) override;

		virtual void compute_control() override;

		virtual void update_configuration()=0;

		const std::vector<std::pair<unsigned,unsigned>>& get_collision_list()
		{
			return collision_list;
		}

		void set_integrator(integrator_t::integrators integrator);

        virtual void set_state_space_bounds(const std::vector<double>& lower,const std::vector<double>& upper) override;

        virtual inline space_t* get_derivative_space()
        {
        	return derivative_space;
        }
        
		space_t* derivative_space;

		static int registred_plants;
		
		virtual void compute_derivative()=0;

	protected:

		plant_t(const system_ptr_t other)
			: system_t(other),
			  movable_object_t(other -> get_pathname())
		{
			auto _plant = std::dynamic_pointer_cast<plant_t>(other);
			prx_assert(_plant != nullptr, "Problem casting to a plant_t");
			derivative_space = _plant -> derivative_space;
			// derivative_memory = other -> derivative_memory;
			integrator = _plant -> integrator;
			derivative_state = _plant -> derivative_state;

			for (int i = 0; i < derivative_space -> get_dimension(); ++i)
			{
				derivative_memory.push_back(_plant -> derivative_memory[i]);
			}
		}

		std::vector<double*> derivative_memory;



		//bodies that we want to check collisions for
		std::vector<std::pair<unsigned,unsigned>> collision_list;

		std::shared_ptr<integrator_t> integrator;

	private:

		space_point_t derivative_state;
		friend system_factory_t;

	};

}
