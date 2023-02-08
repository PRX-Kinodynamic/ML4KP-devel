#ifndef TORCH_NOT_BUILT

#include "prx/utilities/defs.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/utilities/learned_modules/learned_controller.hpp"
#include "prx/utilities/learned_modules/landmark_roadmap.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/planning/planners/dirt_roadmap.hpp"
#include "prx/planning/planner_statistics.hpp"

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
        if (row.size() > 0) dataset.push_back(row);
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
            // params_file = "local_goal/car_like.yaml";
            params_file = "local_goal/annotate_treaded.yaml";
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

        dirt_roadmap_specification_t dirt_spec(context.first,context.second);
        dirt_roadmap_query_t dirt_query(ss,cs);
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

        dirt_roadmap_t dirt("dirt");
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
        std::string roadmap_dir = output_path + params["roadmap_dir"].as<std::string>();
        std::vector<std::vector<double>> vertices = read_comma_separated_file(roadmap_dir + "/vertices.txt");

        for (auto& v : vertices)
        {
            auto p = ss -> make_point();
            // Copy from vector starting at index 1
            auto pv = std::vector<double>(v.begin() + 1, v.end());
            ss -> copy_point_from_vector(p,pv);
            rrr.add_vertex(dirt_spec,p,int(v[0]));
        }

        std::vector<std::vector<double>> edges = read_comma_separated_file(roadmap_dir + "/edges.txt");

        for (auto& e : edges)
        {
            rrr.add_edge(int(e[0]),int(e[1]),e[2]);
        }

        // auto roadmap_edges = rrr.get_all_edges();
        // for (auto e = roadmap_edges.first; e != roadmap_edges.second; ++e)
        // {
        //     auto edge = *e;
        //     std::string traj_fname = out_path + "traj_" + std::to_string(edge.first) + "_" + std::to_string(edge.second) + ".txt";
        //     std::ofstream fout;
        //     fout.open(traj_fname);
        //     fout << rrr.print_edge_traj(edge.first,edge.second,dirt_query,dirt_spec,controller);
        //     fout.close();
        // }

        ss -> copy_point_from_vector(dirt_query.start_state,s);
        auto s_nn = rrr.add_start(dirt_query.start_state, dirt_spec, dirt_query, controller);
        ss -> copy_point_from_vector(dirt_query.goal_state,g);
        auto g_nn = rrr.add_goal(dirt_query.goal_state, dirt_spec, dirt_query, controller);
        ss -> copy_point_from_vector(dirt_query.start_state,s);
        ss -> copy_point_from_vector(dirt_query.goal_state,g);
        prx_assert(s_nn != -1 && g_nn != -1, "Could not find a start or goal node!");

        auto paths = rrr.get_k_shortest_paths(s_nn,g_nn,dirt_spec.blossom_number);

        dirt_query.clear_outputs();
        
        ss -> copy_point_from_vector(dirt_query.start_state,s);
        ss -> copy_point_from_vector(dirt_query.goal_state,g);

        std::cout << ss -> print_point(dirt_query.start_state) << std::endl;
        std::cout << ss -> print_point(dirt_query.goal_state) << std::endl;

        dirt_query_t controller_query(ss,cs);
        controller_query.start_state = ss -> make_point();
        controller_query.goal_state  = ss -> make_point();
        controller_query.goal_region_radius = params["goal_radius"].as<double>();
        controller_query.goal_check = [&,dirt_spec,ss](space_point_t s)
        {
            return dirt_spec.distance_function(s,controller_query.goal_state) < controller_query.goal_region_radius; 
        };
        
        space_point_t lg = ss -> make_point();
        dirt_spec.roadmap_expand = [&](space_point_t& s, std::vector<plan_t*>& plans, std::vector<trajectory_t*>& trajs, 
                    int bn, std::vector<unsigned*>& idxes)
        {
            bool perform_random_expand = true;
            if (bn < dirt_spec.blossom_number)
            {
                std::vector<std::vector<double>> current_states;
                std::vector<std::vector<double>> local_goals;

                std::vector<double> current_state;
                ss -> copy_vector_from_point(current_state,s);
                std::vector<double> local_goal;

                auto nn = rrr.get_best_node_on_kth_path_backward(s,controller_query, dirt_spec, controller, bn);
                
                if (nn == -1)
                {
                    local_goal.clear();
                    do
                    {
                        ss -> sample(lg);
                    } while (!dirt_spec.valid_state(lg));
                    ss -> copy_vector_from_point(local_goal,lg);
                    current_states.push_back(current_state);
                    local_goals.push_back(local_goal);
                }
                else
                {
                    ss -> copy_point(lg,rrr.get_point(nn));
                    ss -> copy_vector_from_point(local_goal,lg);
                    current_states.push_back(current_state);
                    local_goals.push_back(local_goal);
                }

                auto controls = controller.get_controls(current_states,local_goals);

                trajectory_t traj(ss);
                plan_t plan(cs);

                traj.clear(); plan.clear();
                plan.append_onto_back(controller.get_control_duration());
                cs -> copy_point_from_vector(plan.back().control,controls[0]);
                dirt_spec.propagate(s,plan,traj);
                plans.push_back(new plan_t(plan));
                trajs.push_back(new trajectory_t(traj));

            }
            else
            {
                default_expand(s,plans,trajs,1,sg,dirt_spec.sample_plan,dirt_spec.propagate);
            }
        };

        if (!fs::exists(out_path))
        {
            fs::create_directory(out_path);
        }


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

            std::string full_name = out_path + params["planner_name"].as<std::string>()+"_"+ std::to_string(i) + ".txt";
            fout.open(full_name);
            fout << stats.serialize() << std::endl;
            fout.close();

            dirt_query.clear_outputs();
            dirt.reset();
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