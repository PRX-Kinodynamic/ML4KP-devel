#ifndef BULLET_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/rrt.hpp"
#include "prx/bullet_sim/plants/plants.hpp"
#include "prx/bullet_sim/collision_checking/collision_checker.hpp"
#include "prx/bullet_sim/bullet_simulator.hpp"
#include "prx/utilities/learned_modules/learned_controller.hpp"

using namespace prx;

int main(int argc, char* argv[])
{
    try
    {
        std::string params_file;
        if (argc <= 1)
        {
            prx_throw("This executable needs a parameter file!");
        }
        else 
        {
            params_file = std::string(argv[1]);
        }
        param_loader params(params_file);

        simulation_step = params["simulation_step"].as<double>();
        int random_seed = params["random_seed"].as<int>();
        init_random(random_seed);

        auto system = system_factory_t::create_system("segway","segway");
        auto plant = std::dynamic_pointer_cast<bullet_plant_t>(system);
        prx_assert(plant != nullptr, "Plant is not a bullet_plant!");

        std::shared_ptr<bullet_simulator_t> sim = std::make_shared<bullet_simulator_t>();
        sim -> add_urdf(bullet_path + "/data/plane.urdf");
        std::string obstacles_file = params["environment"].as<std::string>();
        auto retval = load_obstacles(obstacles_file,sim);
        sim -> add_group({plant});
        sim -> initialize_simulation();

        auto context = sim -> get_context("bullet_context");
		
		auto ss = context.first->get_state_space();
    	auto cs = context.first -> get_control_space();
        auto sg = context.first;
        auto cg = context.second;

        bullet_collision_group_t bcg(sim);
		auto bcg_ptr = std::make_shared<bullet_collision_group_t>(bcg);
		sim->set_collision_group(bcg_ptr);

        rrt_specification_t rrt_spec(context.first,context.second);
        btVector3 basePosition, baseRotation;
        btQuaternion baseOrientation;
        rrt_spec.valid_state = [&](const space_point_t& state)
        {
            basePosition[0] = state -> at(0);
            basePosition[1] = state -> at(1);
            baseRotation[2] = state -> at(2);
            bullet_simulator_t::get_quaternion_from_euler(baseOrientation, baseRotation);
            sim->resetBasePositionAndOrientation(sim->robot_ids[0],basePosition,baseOrientation);
            sim->stepSimulation();
            bool valid = !bcg_ptr->in_collision();
            return valid;
        };

        std::vector<std::vector<bool>> occupancy_map;
        space_point_t point = ss -> make_point();
        auto bounds = ss -> get_bounds();
        auto bounds_x = bounds[0];
        auto bounds_y = bounds[1];
        double map_resolution = 0.05;
        int map_size_x = (bounds_x.second - bounds_x.first)/map_resolution;

        for (double x_curr = bounds_x.first; x_curr <= bounds_x.second; x_curr += map_resolution)
        {
            std::vector<bool> row;
            for (double y_curr = bounds_y.first; y_curr <= bounds_y.second; y_curr += map_resolution)
            {
                point -> at(0) = x_curr;
                point -> at(1) = y_curr;
                row.push_back(rrt_spec.valid_state(point));
            }
            occupancy_map.push_back(row);
            output_progress_bar(occupancy_map.size()*1.0/map_size_x);
        }

        // Write the occupancy map to a file
        std::ofstream occupancy_map_file;
        occupancy_map_file.open(output_path+"occupancy_map.txt");
        for (auto row : occupancy_map)
        {
            for (auto cell : row)
            {
                occupancy_map_file << cell << ",";
            }
            occupancy_map_file << std::endl;
        }
        occupancy_map_file.close();
    }
    catch(const prx_assert_t& e)
	{
		std::cout<<e.get_message()<<std::endl;
	}
	std::cout<<"End of program"<<std::endl;
}
#else
int main(int argc, char* argv[])
{
}
#endif