#ifndef TORCH_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/utilities/learned_modules/learned_controller.hpp"
#include "prx/utilities/learned_modules/landmark_roadmap.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/planning/planner_statistics.hpp"
#include "prx/visualization/three_js_group.hpp"

#include <fstream>

#ifdef __cpp_lib_filesystem
    #include <filesystem.hpp>
    namespace fs = std::filesystem;
#else
    #define _LIBCPP_NO_EXPERIMENTAL_DEPRECATION_WARNING_FILESYSTEM
    #include <experimental/filesystem>
    namespace fs = std::experimental::filesystem;
#endif

using namespace prx;

std::vector<std::vector<double>> read_comma_separated_file(const std::string& path, const std::string& delimiter = ",")
{
    std::ifstream file(path);
    std::vector<std::vector<double>> dataset;
    std::string line = "";
    while (std::getline(file, line))
    {
        std::vector<double> row;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, delimiter[0]))
        {
            row.push_back(std::stod(cell));
        }
        dataset.push_back(row);
    }
    return dataset;
}

int main(int argc, char* argv[])
{
    try
    {
        std::string params_file;
        if (argc <= 1)
        {
            // prx_throw("This executable needs a parameter file!");
            params_file = "local_goal/car_like.yaml";
            //params_file = "local_goal/annotate_treaded.yaml";
        }
        else 
        {
            params_file = std::string(argv[1]);
        }
        
        param_loader params(params_file);
        params.print();
        prx::timer_t timer; 
        simulation_step = params["simulation_step"].as<double>();
        int random_seed = params["random_seed"].as<int>();
        init_random(random_seed);
        torch::set_num_threads(1);

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

        dirt_specification_t dirt_spec(context.first,context.second);
        dirt_query_t dirt_query(ss,cs);
        dirt_query.start_state = ss -> make_point();
        dirt_query.goal_state  = ss -> make_point();
        dirt_query.get_visualization = true;

        dirt_query.goal_region_radius = params["goal_radius"].as<double>();

        learned_controller_t controller(params);

        dirt_spec.distance_function = [&](space_point_t a, space_point_t b)
        {
            return ss -> euclidean_2d(a,b,0,2);
        };

        dirt_spec.h = [&](space_point_t s, space_point_t d)
        {
            return ss -> euclidean_2d(s,d,0,2);
        };

        dirt_query.goal_check = [&,dirt_spec,ss](space_point_t s)
        {
            // return dirt_spec.distance_function(s,dirt_query.goal_state) < dirt_query.goal_region_radius; 
            return ss -> euclidean_2d(s, dirt_query.goal_state, 0, 3) < dirt_query.goal_region_radius;
        };

        dirt_t dirt("dirt");
        dirt_spec.min_control_steps = params["/plant/min_steps"].as<int>();
        dirt_spec.max_control_steps = params["/plant/max_steps"].as<int>();
        dirt_spec.blossom_number = 1;
        dirt_spec.use_pruning = false;

        std::vector<double> s = params["start_state"].as<std::vector<double>>();
        std::vector<double> g = params["goal_state"].as<std::vector<double>>();
        ss -> copy_point_from_vector(dirt_query.start_state,s);
        ss -> copy_point_from_vector(dirt_query.goal_state,g);

        std::ofstream fout;
        std::string output_dir = params["output_dir"].as<std::string>();
        std::string out_path = output_path + output_dir;
        
        landmark_roadmap_t rrr;
        
        std::string points_fname = out_path + "points.txt";
        std::cout << "Reading points from: " << points_fname << std::endl;
        std::vector<std::vector<double>> dataset = read_comma_separated_file(points_fname);
        space_point_t current = ss -> make_point();
        for (auto row: dataset)
        {
            ss -> copy_point_from_vector(current,row);
            rrr.verification_set.push_back(ss -> clone_point(current));
        }

        int verification_set_size = rrr.verification_set.size();
        std::cout << "Verification set size: " << verification_set_size << std::endl;

        timer.reset();
        rrr.set_stretch_factor(params["stretch_factor"].as<double>());
        rrr.build_roadmap(dirt_query, dirt_spec, controller);
        double time_taken = timer.measure_reset();
        std::cout << "Time taken to build roadmap: " << time_taken << std::endl;
        std::cout << rrr.is_connected() << std::endl;

        // Output graph to file.
        std::string vertex_fname = out_path + "vertices.txt";
        std::string edge_fname = out_path + "edges.txt";

        std::ofstream vertex_file(vertex_fname);
        std::ofstream edge_file(edge_fname);

        vertex_file << rrr.print_vertices(ss);
        edge_file << rrr.print_edges();

        vertex_file.close();
        edge_file.close();
        
        auto roadmap_edges = rrr.get_all_edges();
        for (auto e = roadmap_edges.first; e != roadmap_edges.second; ++e)
        {
            auto edge = *e;
            std::string traj_fname = out_path + "traj_" + std::to_string(edge.first) + "_" + std::to_string(edge.second) + ".txt";
            std::ofstream fout;
            fout.open(traj_fname);
            fout << rrr.print_edge_traj(edge.first,edge.second,dirt_query,dirt_spec,controller);
            fout.close();
        }

        // If the output directory does not exist, create it
        if (!fs::exists(out_path))
        {
            fs::create_directory(out_path);
        }

        ss -> copy_point_from_vector(dirt_query.start_state,s);
        auto s_nn = rrr.add_start(dirt_query.start_state, dirt_spec, dirt_query, controller);
        ss -> copy_point_from_vector(dirt_query.goal_state,g);
        auto g_nn = rrr.add_goal(dirt_query.goal_state, dirt_spec, dirt_query, controller);
        ss -> copy_point_from_vector(dirt_query.start_state,s);
        ss -> copy_point_from_vector(dirt_query.goal_state,g);

        std::cout << ss -> print_point(dirt_query.start_state) << std::endl;
        std::cout << ss -> print_point(dirt_query.goal_state) << std::endl;

        prx_assert(s_nn != -1 && g_nn != -1, "Could not find a start or goal node!");
        
        auto path = rrr.get_shortest_path(s_nn,g_nn);


        
        std::cout << "Path: " << std::endl;
        for (auto v: path)
        {
            std::cout << v << " ";
        }
        std::cout << std::endl;

        auto iter = path.begin()+1;

        double path_len = 0.0;
        for(iter; iter < path.end(); iter++)
        {
            path_len += rrr.get_edge_len(*(iter-1),*iter);
        }

        std::string path_fname = out_path + "path.txt";
        fout.open(path_fname);
        fout << rrr.print_path(s_nn,g_nn,dirt_spec);
        fout.close();


        std::cout << "Path len (with gaps): "<< path_len << std::endl;
        
        /*
        dirt_query_t controller_query(ss,cs);
        controller_query.start_state = ss -> make_point();
        controller_query.goal_state  = ss -> make_point();
        controller_query.goal_region_radius = params["goal_radius"].as<double>();
        controller_query.goal_check = [&,dirt_spec,ss](space_point_t s)
        {
            return dirt_spec.distance_function(s,controller_query.goal_state) < controller_query.goal_region_radius; 
        };

        space_point_t lg = ss -> make_point();
        dirt_spec.expand = [&](space_point_t& s, std::vector<plan_t*>& plans, std::vector<trajectory_t*>& trajs, int bn, bool blossom_expand)
        {
            if (blossom_expand)
            {
                std::vector<std::vector<double>> current_states;
                std::vector<std::vector<double>> local_goals;

                std::vector<double> current_state;
                ss -> copy_vector_from_point(current_state,s);
                std::vector<double> local_goal;

                auto nn = rrr.get_best_node_backward(s,controller_query, dirt_spec, controller);
                if (nn == -1)
                {
                    // std::cout << "No best node was found!" << std::endl;
                    local_goal.clear();
                    do
                    {
                        ss -> sample(lg);
                    } while (!dirt_spec.valid_state(lg));
                    ss -> copy_vector_from_point(local_goal,lg);
                    current_states.push_back(current_state);
                    local_goals.push_back(local_goal);

                    // auto controls = controller.get_controls(current_states,local_goals);

                    // trajectory_t traj(ss);
                    // plan_t plan(cs);

                    // for (int i = 0; i < bn; i++)
                    // {
                    //     traj.clear(); plan.clear();
                    //     plan.append_onto_back(controller.get_control_duration());
                    //     cs -> copy_point_from_vector(plan.back().control,controls[i]);
                    //     dirt_spec.propagate(s,plan,traj);
                    //     plans.push_back(new plan_t(plan));
                    //     trajs.push_back(new trajectory_t(traj));
                    // }
                }
                else
                {
                    ss -> copy_point(lg,rrr.get_point(nn));
                    ss -> copy_vector_from_point(local_goal,lg);
                    current_states.push_back(current_state);
                    local_goals.push_back(local_goal);
                    // plans.push_back(new  plan_t(controller_query.solution_plan));
                    // trajs.push_back(new trajectory_t(controller_query.solution_traj));
                }

                auto controls = controller.get_controls(current_states,local_goals);

                trajectory_t traj(ss);
                plan_t plan(cs);

                for (int i = 0; i < bn; i++)
                {
                    traj.clear(); plan.clear();
                    plan.append_onto_back(controller.get_control_duration());
                    cs -> copy_point_from_vector(plan.back().control,controls[i]);
                    dirt_spec.propagate(s,plan,traj);
                    plans.push_back(new plan_t(plan));
                    trajs.push_back(new trajectory_t(traj));
                }
            }
            else
            {
                default_expand(s,plans,trajs,bn,sg,dirt_spec.sample_plan,dirt_spec.propagate);
            }
        };

        
        // condition_check_t checker("solutions",1);
        // condition_check_t checker("iterations",1);
        int stats_runs = 10;
        condition_check_t checker("time", 0.5);
        for (int i = 0; i < stats_runs; i++)
        {
            dirt.link_and_setup_spec(&dirt_spec);
            dirt.preprocess();
            dirt.link_and_setup_query(&dirt_query);

            planner_statistics_t stats;
            stats.link_planner(&dirt);
            stats.link_criterion(&checker);
            simulation_time = 0.0;
            stats.repeat_data_gathering(60);
            double end_sim_time = simulation_time;

            std::string full_name = out_path + params["planner_name"].as<std::string>()+"_"+ std::to_string(i) + ".txt";
            fout.open(full_name);
            fout << stats.serialize() << std::endl;
            fout.close();

            // simulation_time = 0.0;
            // dirt.resolve_query(&checker);
            // double end_sim_time = simulation_time;
            // dirt.fulfill_query();

            // std::string full_filename = out_path + params["planner_name"].as<std::string>()+"_"+ std::to_string(i) + ".txt";
            // fout.open(full_filename);
            // fout << dirt.get_current_solution() << std::endl;
            // fout << dirt.get_current_solution_time() << std::endl;
            // fout << dirt.get_current_solution_iters() << std::endl;
            // fout << dirt.get_branching_factor() << std::endl;
            // fout << end_sim_time << std::endl;
            // // fout << 1.0 * rrr.verification_set.size() / verification_set_size << std::endl;
            // fout << time_taken << std::endl;
            // fout.close();

            std::string traj_fname = out_path + "traj.txt";
            fout.open(traj_fname);
            fout << dirt_query.solution_traj.print() << std::endl;
            fout.close();

            std::string plan_fname = out_path + "plan.txt";
            fout.open(plan_fname);
            fout << dirt_query.solution_plan.print(4) << std::endl;
            fout.close();

            // three_js_group_t* vis_group = new three_js_group_t({plant},{obstacle_list});
            // std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();
            // vis_group -> add_vis_infos(info_geometry_t::LINE, dirt_query.tree_visualization, body_name, ss);
            // vis_group -> add_detailed_vis_infos(info_geometry_t::FULL_LINE, dirt_query.solution_traj, body_name, ss);
            // vis_group -> add_animation(dirt_query.solution_traj, ss, dirt_query.start_state);
            // vis_group -> output_html(params["output_dir"].as<std::string>()+params["planner_name"].as<std::string>()+"_"+std::to_string(i)+".html");
            // delete vis_group;

            dirt_query.clear_outputs();
            dirt.reset();
        }
        */
        
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