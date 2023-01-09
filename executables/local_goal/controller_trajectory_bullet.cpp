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

        rrt_query_t rrt_query(ss,cs);
        rrt_query.start_state = ss -> make_point();
        rrt_query.goal_state  = ss -> make_point();
        rrt_query.goal_region_radius = params["goal_radius"].as<double>();
        rrt_query.goal_check = [&](space_point_t point)
        {
            return ss -> euclidean_2d(point, rrt_query.goal_state, 0, 3) < rrt_query.goal_region_radius;
        };

        std::vector<double> sv = params["start_state"].as<std::vector<double>>();
        std::vector<double> gv = params["goal_state"].as<std::vector<double>>();


        ss -> copy_point_from_vector(rrt_query.start_state, sv);
        rrt_query.start_state -> at(6) = 0;
        ss -> copy_from_point(rrt_query.start_state);
        sim -> reset_simulation();
        ss -> copy_to_point(rrt_query.start_state);
        ss -> copy_point_from_vector(rrt_query.goal_state, gv);

        learned_controller_t controller(params);
        controller.fulfill_query(rrt_query, sg);

        // Write trajectory to a file
        std::string output_id = params["output_id"].as<std::string>();
        std::ofstream traj_file;
        traj_file.open(output_path+"trajectory.txt");
        traj_file << rrt_query.solution_traj.print(4) << std::endl;
        traj_file.close();
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