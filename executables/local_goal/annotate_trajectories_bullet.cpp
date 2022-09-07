#if !defined(TORCH_NOT_BUILT) && !defined(BULLET_NOT_BUILT)
#include "prx/utilities/defs.hpp"
#include "prx/bullet_sim/bullet_simulator.hpp"
#include "prx/bullet_sim/plants/plants.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/bullet_sim/loaders/obstacle_loader.hpp"
#include "prx/utilities/learned_modules/learned_controller.hpp"

#include "prx/utilities/learned_modules/reachability_estimator.hpp"
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

        std::string plant_name = params["/plant/name"].as<std::string>();
        std::string plant_path = params["/plant/path"].as<std::string>();
        auto plant = system_factory_t::create_system(plant_name,plant_path);

        std::vector<double> lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
        std::vector<double> upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
        plant -> set_state_space_bounds(lower_bounds,upper_bounds);

        auto bsim = std::make_shared<bullet_simulator_t>();
		bsim -> add_urdf(models_path + "misc/plane.urdf");
        std::string obstacles_file = params["environment"].as<std::string>();
		bsim -> add_group({plant});
        auto retval = load_obstacles(obstacles_file,bsim);
		bsim -> initialize_simulation();

        bullet_collision_group_t bcg(bsim);
		auto bcg_ptr = std::make_shared<bullet_collision_group_t>(bcg);
		bsim->set_collision_group(bcg_ptr);

        auto context = bsim -> get_context("bullet_context");
		
		auto ss = context.first->get_state_space();
    	auto cs = context.first -> get_control_space();
		auto sg = context.first;   

        const int num_trajectories = params["num_trajectories"].as<int>();

        dirt_t dirt("dirt");

        dirt_specification_t dirt_spec(context.first,context.second);
        dirt_spec.min_control_steps = params["/learned_controller/control_duration"].as<double>()/simulation_step;
        dirt_spec.max_control_steps = params["/learned_controller/control_duration"].as<double>()/simulation_step;
        dirt_spec.blossom_number = 1;
        dirt_spec.use_pruning = true;

        btVector3 basePosition, baseRotation;
        btQuaternion baseOrientation;
        dirt_spec.valid_state = [&](const space_point_t& state)
        {
            basePosition[0] = state -> at(0);
            basePosition[1] = state -> at(1);
            baseRotation[0] = baseRotation[1] = basePosition[2] = 0;
            baseRotation[2] = state -> at(2);
            bullet_simulator_t::get_quaternion_from_euler(baseOrientation, baseRotation);
            bsim->resetBasePositionAndOrientation(bsim->robot_ids[0],basePosition,baseOrientation);
            bsim->stepSimulation();
            bool valid = !bcg_ptr->in_collision();
            return valid;
        };

        dirt_spec.valid_check = [&](trajectory_t& traj)
        {
            for(auto&& s : traj)
            {
                if (!dirt_spec.valid_state(s))
                {
                    return false;
                }
            }
            return true;
        };

        dirt_query_t dirt_query(ss,cs);
        dirt_query.start_state = ss -> make_point();
        dirt_query.goal_state  = ss -> make_point();
        dirt_query.goal_region_radius = params["goal_radius"].as<double>();

        dirt_spec.distance_function = [&](space_point_t a, space_point_t b)
        {
            std::vector <double> diff = {a->at(0)-b->at(0),a->at(1)-b->at(1),
            norm_angle_pi(a->at(2)-b->at(2))};

            double accum = 0.;
            for (auto v: diff) {
                accum += v*v;
            }
            return sqrt(accum);
        };

        dirt_spec.h = [&](space_point_t a, space_point_t b)
        {
            return dirt_spec.distance_function(a,b) / 3.0;
        };

        dirt_query.goal_check = [&,dirt_spec,ss](space_point_t s)
        {
            return dirt_spec.distance_function(s,dirt_query.goal_state) < dirt_query.goal_region_radius; 
            // return ss -> euclidean_2d(point, dirt_query.goal_state, 0, 3) < dirt_query.goal_region_radius;
        };

        std::string output_dir = params["output_dir"].as<std::string>();

        learned_controller_t controller(params);
        double horizon = params["max_steps"].as<double>();

        std::vector<double> start_vec, goal_vec, local_vec, last_state_vec;

        for (int idx = 0; idx < num_trajectories; idx++)
        {
            PRX_DEBUG_PRINT
            dirt_query.clear_outputs();

            // bsim->reset_simulation();
            // auto retval = load_obstacles(obstacles_file,bsim);

            bool sample = true;
            while(sample)
            {
                ss -> sample(dirt_query.start_state);
                ss -> sample(dirt_query.goal_state);
                sample = !dirt_spec.valid_state(dirt_query.start_state) || !dirt_spec.valid_state(dirt_query.goal_state);
            }

            ss->copy_from_point(dirt_query.start_state);
            space_point_t init_state = ss->clone_point(dirt_query.start_state);
            bsim->reset_simulation();
            ss->copy_to_point(dirt_query.start_state);

            controller.fulfill_query(dirt_query, sg, horizon);

            retval = load_obstacles(obstacles_file,bsim);

            std::vector <std::vector <double>> input_states, local_goals, last_states, global_goals;
            if (dirt_query.solution_traj.size() != 0 && dirt_spec.valid_check(dirt_query.solution_traj))
            {
                std::cout << "Valid\n";
                for (unsigned i = 0; i < dirt_query.solution_traj.size(); i++)
                {
                    ss -> copy_vector_from_point(start_vec, dirt_query.solution_traj[i]);
                    ss -> copy_vector_from_point(local_vec, dirt_query.goal_state);
                    ss -> copy_vector_from_point(last_state_vec, dirt_query.solution_traj.back());
                    ss -> copy_vector_from_point(goal_vec, dirt_query.goal_state);

                    input_states.push_back(start_vec);
                    local_goals.push_back(local_vec);
                    last_states.push_back(last_state_vec);
                    global_goals.push_back(goal_vec);
                }
            }
            else
            {
                PRX_DEBUG_PRINT
                dirt_query.clear_outputs();
                std::cout << "Planning for: " << ss->print_point(dirt_query.start_state,4) << "," << ss->print_point(dirt_query.goal_state,4) << std::endl;

                ss->copy_from_point(dirt_query.start_state);
                bsim->reset_simulation_with_obstacles(obstacles_file);
                
                dirt.reset();
                dirt.link_and_setup_spec(&dirt_spec);
                dirt.preprocess();
                dirt.link_and_setup_query(&dirt_query);

                condition_check_t checker("time",60);

                dirt.resolve_query(&checker);
                dirt.fulfill_query();

                if (dirt_query.solution_traj.size() == 0) continue;
                trajectory_t planned_trajectory(ss);
                planned_trajectory.clear();
                for (auto s:dirt_query.solution_traj)
                {
                    planned_trajectory.copy_onto_back(s);
                }

                unsigned state_id = 0;

                while (state_id < planned_trajectory.size()-1)
                {
                    int max_state_id = -1;
                    
                    space_point_t state = ss -> clone_point(planned_trajectory[state_id]);
                    space_point_t max_state;
                    
                    std::vector<double> test_state, test_goal;
                    ss -> copy_vector_from_point(test_state, state);
                    
                    for (unsigned i = planned_trajectory.size()-1; i > state_id; i -= dirt_spec.max_control_steps)
                    {
                        PRX_DEBUG_PRINT
                        dirt_query.clear_outputs();
                        ss -> copy_point(dirt_query.goal_state,planned_trajectory[i]);
                        ss -> copy_point(dirt_query.start_state,state);

                        ss->copy_from_point(dirt_query.start_state);
                        bsim->reset_simulation_with_obstacles(obstacles_file);

                        controller.fulfill_query(dirt_query, sg, horizon);

                        if (dirt_query.solution_traj.size() == 0 || !dirt_spec.valid_check(dirt_query.solution_traj)) continue;
                        for(auto s: dirt_query.solution_traj)
                        {
                            ss -> copy_vector_from_point(start_vec, s);
                            ss -> copy_vector_from_point(local_vec, planned_trajectory[i]);
                            ss -> copy_vector_from_point(last_state_vec, dirt_query.solution_traj.back());
                            ss -> copy_vector_from_point(goal_vec, planned_trajectory.back());

                            input_states.push_back(start_vec);
                            local_goals.push_back(local_vec);
                            last_states.push_back(last_state_vec);
                            global_goals.push_back(goal_vec);
                        }
                        max_state_id = i;
                        break;
                    }
                    if (max_state_id == -1)
                    {
                        std::cout << "No max state found" << std::endl;
                        break;
                    }
                    else
                    {
                        unsigned id = max_state_id;
                        std::cout << "Max state id: " << max_state_id << std::endl;
                        max_state = ss -> clone_point(planned_trajectory[id]);
                    }
                    std::cout << max_state_id;
                    state_id = max_state_id;
                }
            }
            std::ofstream fout;
            fout.open(output_path+output_dir+std::to_string(random_seed)+"annotated_trajectories_"+std::to_string(idx)+".txt");
            for(int i=0; i<input_states.size();i++){
                for(int j=0; j<input_states[i].size(); j++){
                    fout << input_states[i][j] << " ";
                }
                fout << "# ";
                for(int j=0; j<input_states[i].size(); j++){
                    fout << local_goals[i][j] << " ";
                }
                fout << "# ";
                for(int j=0; j<input_states[i].size(); j++){
                    fout << last_states[i][j] << " ";
                }
                fout << "# ";
                for(int j=0; j<input_states[i].size(); j++){
                    fout << global_goals[i][j] << " ";
                }
                fout << "\n";
            }
            fout.close();

            output_progress_bar(idx*1.0/num_trajectories);
        }
    }
    catch(const prx_assert_t& e) 
    {
        std::cerr << e.what() << std::endl;
        return -1;
    }
}
#else
int main() 
{
    // prx_throw("Torch and Bullet are not built. Cannot run this program.");
    return -1;
}
#endif