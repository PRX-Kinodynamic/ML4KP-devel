#include <memory>
#include <optional>
#include <typeinfo>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/RenderingIface.hh>

#include "prx/utilities/defs.hpp"
#include "prx/simulation/simulator.hpp"
#include "prx/utilities/general/constants.hpp"
#include "prx/simulation/playback/trajectory.hpp"

#include "prx/gazebo/constants.hpp"
#include "prx/gazebo/plants/plants.hpp"

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
  			gazebo::shutdown();
		}

		// void add_group(const std::vector<std::shared_ptr<plant_gz_t<system_t>>>& all_systems);
		virtual void add_group(const std::vector<system_ptr_t>& all_systems) override;

		virtual void step_simulation(propagate_step step) override;

		void step_simulation(int steps);

		virtual void reset_simulation() override;

		void initialize_simulation();

        gazebo::physics::ModelState get_model_state(std::string plant_name);

		gazebo::physics::ModelPtr get_model_ptr(const std::string& plant_name);

		void set_world(std::string _path)
		{
			world_file = _path;
		}

		void pause(bool pause)
		{
			gazebo::physics::pause_world(world, pause);
		}

		void create_context(const std::vector<std::string>& system_names, 
			const std::vector<std::string>& obstacle_names, 
			const std::string& context_name = "default_context");

		system_ptr_t get_plant(std::string name);

		gazebo::physics::WorldPtr world;

	private:
		std::string world_file;
		std::vector< std::string > server_params;

		std::unique_ptr<gazebo::Server> server;

		std::shared_ptr<gazebo::physics::WorldState> world_state;

		ignition::fuel_tools::ServerConfig config;

	};
}