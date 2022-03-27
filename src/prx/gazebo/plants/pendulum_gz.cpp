#include "prx/gazebo/plants/pendulum_gz.hpp"

namespace prx
{
	pendulum_gz_t::pendulum_gz_t(const std::string& path)
		: pendulum_t(path), plant_gz_t()
	{}

	pendulum_gz_t::~pendulum_gz_t()
	{}

	gazebo::physics::ModelState pendulum_gz_t::get_model_state() const
	{
		gazebo::physics::ModelState model_state;
		return model_state;
	}

	void pendulum_gz_t::set_model_state(const gazebo::physics::ModelState&)
	{

		
	}
}