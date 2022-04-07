#include "prx/gazebo/plants/plant_gz.hpp"

namespace prx
{

	class plant_gz_wrapper_t : public plant_t//, public plant_gz_t
	{
		public:
		template<class T>
		plant_gz_wrapper_t(std::shared_ptr<T> _plant) : plant_t(_plant) //plant_t(*_plant.get())
		{
			prx_assert( std::dynamic_pointer_cast<plant_gz_t>(_plant), 
				"Unable to wrap class " << typeid(_plant).name() << ", can't cast to plant_gz_t.");
			prx_assert( std::dynamic_pointer_cast<plant_t>(_plant), 
				"Unable to wrap class " << typeid(_plant).name() << ", can't cast to plant_t.");
			plant = std::dynamic_pointer_cast<plant_t>(_plant);
			plant_gz = std::dynamic_pointer_cast<plant_gz_t>(_plant);
			// auto this_plant = dynamic_cast<plant_t*>(this);
			// this_plant = std::move(_plant.get());
			// *dynamic_cast<plant_t*>(this) = *dynamic_cast<plant_t*>(_plant.get());
			// pathname = _plant -> get_pathname();
			// pathname = _plant -> get_pathname();
			// collision_list = plant -> get_collision_list();
		}

		~plant_gz_wrapper_t() = default;

		std::shared_ptr<plant_t> get_plant()
		{
			return plant;
		}

		std::shared_ptr<plant_gz_t> get_plant_gz()
		{
			return plant_gz;
		}

		virtual void update_configuration() override
		{
			prx_throw("Shouldn't call update_configuration from the plant_gz_wrapper_t, use plant_gz_wrapper_t::get_plant().");
		}

		virtual void compute_derivative() override
		{
			
		}

		private:
			std::shared_ptr<plant_t> plant;
			std::shared_ptr<plant_gz_t> plant_gz;
	};
}