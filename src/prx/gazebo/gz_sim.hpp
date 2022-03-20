#include <optional>

#include "prx/utilities/defs.hpp"
#include "prx/simulation/simulator.hpp"
#include "prx/utilities/general/constants.hpp"
#include "prx/simulation/playback/trajectory.hpp"

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
// #include "gazebo/math/Vector3.hh"
// #include "gazebo/msgs/msgs.hh"
// #include "gazebo/physics/physics.hh"
// #include "gazebo/transport/transport.hh"


namespace prx
{
	class gz_sim_t : public simulator_t
	{

	public: 
		gz_sim_t(const std::vector< std::string > _params = {}) 
			: simulator_t(plant_type::GZ), server_params(_params)
		{
		}
		gz_sim_t(int _argc, char **_argv) 
			: simulator_t(plant_type::GZ)
		{
			for (int i = 0; i < _argc; ++i)
			{
				server_params.push_back(_argv[i]);
			}
		}

		~gz_sim_t()
		{
  			// gazebo::shutdown();
		}


		virtual void step_simulation(propagate_step step) override;

		virtual void reset_simulation() override;

		void initialize_simulation();

		void set_world(std::string _path)
		{
			world_file = _path;
			// world = gazebo::loadWorld(_path);
		}

		system_ptr_t get_plant(std::string name);

		gazebo::physics::WorldPtr world;

	private:
		std::string world_file;
		std::vector< std::string > server_params;
	};
}