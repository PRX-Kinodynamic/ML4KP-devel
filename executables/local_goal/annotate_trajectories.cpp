#ifndef TORCH_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/utilities/learned_modules/reachability_estimator.hpp"
#include "prx/utilities/learned_modules/learned_controller.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/visualization/three_js_group.hpp"

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

        auto obstacles = load_obstacles(params["environment"].as<std::string>());
        auto obstacle_list = obstacles.second;
        auto obstacle_names = obstacles.first;

        std::string plant_name = params["/plant/name"].as<std::string>();
        std::string plant_path = params["/plant/path"].as<std::string>();
        auto plant = system_factory_t::create_system(plant_name,plant_path);

        std::vector<double> lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
        std::vector<double> upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
        plant -> set_state_space_bounds(lower_bounds,upper_bounds);

        world_model_t world_model({plant},{obstacle_list});
        world_model.create_context("planning_context",{plant_name},{obstacle_names});
        auto context = world_model.get_context("planning_context");

        auto ss = context.first->get_state_space();
        auto cs = context.first->get_control_space();
        auto sg = context.first;

        const int num_trajectories = params["num_trajectories"].as<int>();

        dirt_t dirt("dirt");

        dirt_specification_t dirt_spec(context.first,context.second);
        dirt_spec.min_control_steps = params["/learned_controller/control_duration"].as<double>()/simulation_step;
        dirt_spec.max_control_steps = params["/learned_controller/control_duration"].as<double>()/simulation_step;
        dirt_spec.blossom_number = 5;
        dirt_spec.use_pruning = false;

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

        dirt_query.goal_check = [&,dirt_spec,ss](space_point_t s)
        {
            return dirt_spec.distance_function(s,dirt_query.goal_state) < dirt_query.goal_region_radius; 
            // return ss -> euclidean_2d(point, dirt_query.goal_state, 0, 3) < dirt_query.goal_region_radius;
        };

        std::string output_dir = params["output_dir"].as<std::string>();

        reachability_estimator_t estimator(params);
        learned_controller_t controller(params);
        double horizon = params["max_steps"].as<double>();

        std::vector<double> start_vec, goal_vec, local_vec, last_state_vec;

        for (int idx = 0; idx < num_trajectories; idx++)
        {
            dirt_query.clear_outputs();

            bool sample = true;
            while(sample)
            {
                ss -> sample(dirt_query.start_state);
                dirt_query.start_state->at(3) = dirt_query.start_state->at(4) = 0;
                ss -> sample(dirt_query.goal_state);
                sample = !dirt_spec.valid_state(dirt_query.start_state) || !dirt_spec.valid_state(dirt_query.goal_state);
            }

            controller.fulfill_query(dirt_query, sg, horizon);

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
                dirt_query.clear_outputs();
                std::cout << ss->print_point(dirt_query.start_state,4) << "," << ss->print_point(dirt_query.goal_state,4) << std::endl;

                dirt.reset();
                dirt.link_and_setup_spec(&dirt_spec);
                dirt.preprocess();
                dirt.link_and_setup_query(&dirt_query);

                condition_check_t checker("time",30);

                dirt.resolve_query(&checker);
                dirt.fulfill_query();

                if (dirt_query.solution_traj.size() == 0) continue;
                trajectory_t lc_trajectory(ss);
                lc_trajectory.clear();
                for (auto s:dirt_query.solution_traj){
                    lc_trajectory.copy_onto_back(s);
                }

                unsigned state_id = 0;

                while (state_id < lc_trajectory.size()-1)
                {
                    int max_state_id = -1;
                    space_point_t state = ss -> clone_point(lc_trajectory[state_id]);
                    space_point_t max_state;
                    std::vector<double> test_state, test_goal;
                    ss -> copy_vector_from_point(test_state, state);
                    for (unsigned i = lc_trajectory.size()-1; i > state_id; i -= dirt_spec.max_control_steps)
                    {
                        dirt_query.clear_outputs();
                        dirt_query.goal_state = lc_trajectory[i];
                        dirt_query.start_state = state;
                        controller.fulfill_query(dirt_query, sg, horizon);
                        if (dirt_query.solution_traj.size() == 0 || !dirt_spec.valid_check(dirt_query.solution_traj)) continue;
                        for(auto s: dirt_query.solution_traj){
                            ss -> copy_vector_from_point(start_vec, s);
                            ss -> copy_vector_from_point(local_vec, lc_trajectory[i]);
                            ss -> copy_vector_from_point(last_state_vec, dirt_query.solution_traj.back());
                            ss -> copy_vector_from_point(goal_vec, lc_trajectory.back());

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
                        // return -1;
                        if (params["debug"].as<bool>())
                        {
                            std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();
                            three_js_group_t* vis_group = new three_js_group_t({plant},{obstacle_list});
                            vis_group->add_vis_infos(info_geometry_t::LINE, dirt_query.solution_traj, body_name, ss, "0x000000");
                            vis_group->add_vis_infos(info_geometry_t::LINE, lc_trajectory, body_name, ss, "0x0000ff");
                            vis_group->output_html(params["output_dir"].as<std::string>()+std::to_string(idx)+".html");
                            delete vis_group;
                        }

                        break;
                    }
                    else
                    {
                        unsigned id = max_state_id;
                        std::cout << "Max state id: " << max_state_id << std::endl;
                        max_state = ss -> clone_point(lc_trajectory[id]);
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
#endif