#ifndef TORCH_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/bullet_sim/bullet_simulator.hpp"
#include "prx/bullet_sim/plants/plants.hpp"
#include "prx/planning/planners/rrt.hpp"
#include "prx/bullet_sim/loaders/obstacle_loader.hpp"
#include "prx/utilities/learned_modules/learned_controller.hpp"

#include <fstream>

using namespace prx;

int main(int argc, char* argv[])
{
    try
    {
        std::string params_file;
        if (argc <= 1)
        {
            prx_throw("The planner evaluation executable needs a parameter file!");
        }
        else 
        {
            params_file = std::string(argv[1]);
        }
        
        param_loader params(params_file);
        params.print();
        simulation_step = params["simulation_step"].as<double>();
        int random_seed = params["random_seed"].as<int>();
        init_random(random_seed);

        learned_controller_t controller(params);

        std::string plant_name = params["/plant/name"].as<std::string>();
        std::string plant_path = params["/plant/path"].as<std::string>();
        auto plant = system_factory_t::create_system(plant_name,plant_path);

        auto bsim = std::make_shared<bullet_simulator_t>();
		bsim -> add_urdf(bullet_path + "/data/plane.urdf");
        std::string obstacles_file = params["environment"].as<std::string>();
        // auto retval = load_obstacles(obstacles_file,bsim);
		bsim -> add_group({plant});
		bsim -> initialize_simulation();

        auto context = bsim -> get_context("bullet_context");
		
		auto ss = context.first->get_state_space();
    	auto cs = context.first -> get_control_space();
		auto sg = context.first;
        auto cg = context.second;

        bullet_collision_group_t bcg(bsim);
		auto bcg_ptr = std::make_shared<bullet_collision_group_t>(bcg);
		bsim->set_collision_group(bcg_ptr);

        const int num_trajectories = params["num_trajectories"].as<int>();
        const double max_duration  = params["max_duration"].as<double>();
        const double control_duration = params["control_duration"].as<double>();

        btVector3 basePosition, baseRotation;
        btQuaternion baseOrientation;

        rrt_specification_t rrt_spec(context.first,context.second);
        rrt_spec.valid_state = [&](const space_point_t& state)
        {
            PRX_DEBUG_PRINT
            basePosition[0] = state -> at(0);
            basePosition[1] = state -> at(1);
            baseRotation[2] = state -> at(2);
            bullet_simulator_t::get_quaternion_from_euler(baseOrientation, baseRotation);
            bsim->resetBasePositionAndOrientation(bsim->robot_ids[0],basePosition,baseOrientation);
            bsim->stepSimulation();
            bool valid = !cg->in_collision();
            return valid;
        };

        rrt_query_t rrt_query(ss,cs);
        rrt_query.start_state = ss -> make_point();
        rrt_query.goal_state  = ss -> make_point();
        rrt_query.goal_region_radius = params["goal_radius"].as<double>();
        rrt_query.goal_check = [&,ss](space_point_t point)
        {
            return ss -> euclidean_2d(point, rrt_query.goal_state, 0, 3) < rrt_query.goal_region_radius;
        };

        std::string output_dir = params["output_dir"].as<std::string>();

        space_point_t current = ss -> make_point();
        unsigned dim = ss -> get_dimension();

        for (int i = 0; i < num_trajectories; i++)
        {
            ss->sample(rrt_query.start_state);
            std::cout << "Sampled start: " << ss->print_point(rrt_query.start_state) << std::endl;
            // This is additional for the bullet-simulated plant.
            rrt_query.start_state->at(dim-1) = 0;
            ss->copy_from_point(rrt_query.start_state);
            bsim->reset_simulation();
            ss->sample(rrt_query.goal_state);
            ss->copy_to_point(rrt_query.start_state);
            // End of Bullet stuff

            std::cout << ss->print_point(rrt_query.start_state,2) << " " 
            << ss->print_point(rrt_query.goal_state,2) << std::endl;

            controller.fulfill_query(rrt_query,sg,max_duration);

            std::ofstream ofs;
            ofs.open(output_path+output_dir+ "/trajectory_" + std::to_string(i) + ".txt");
            ofs << rrt_query.solution_traj.print(2) << std::endl;
            ofs.close();

            auto retval = load_obstacles(obstacles_file,bsim);
            
            ofs.open(output_path+output_dir+"/trajectory_annotated_"+std::to_string(i)+".txt");

            unsigned last_state = -1;
            for (unsigned i = 0; i < rrt_query.solution_traj.size(); i += control_duration/simulation_step)
            {
        
                ss->copy_point(current,rrt_query.solution_traj[i]);
                if (last_state != current->at(dim-1))
                {
                    last_state = current->at(dim-1);
                    ofs << ss->print_point(rrt_query.solution_traj[i],4) << "," << rrt_spec.valid_state(current) << std::endl;
                }
            }
            ofs.close();

            output_progress_bar(i*1.0/num_trajectories);
        }

    }
    catch(const prx_assert_t& e) 
    {
        std::cerr << e.what() << std::endl;
        return -1;
    }
}

#else
int main() {}
#endif