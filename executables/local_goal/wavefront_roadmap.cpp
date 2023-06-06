#ifndef TORCH_NOT_BUILT

#include "prx/utilities/defs.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/utilities/learned_modules/learned_controller.hpp"
#include "prx/utilities/learned_modules/landmark_roadmap.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/planning/planners/dirt_roadmap.hpp"
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
            params_file = "local_goal/car_like.yaml";
            // params_file = "local_goal/annotate_treaded.yaml";
        }
        else 
        {
            params_file = std::string(argv[1]);
        }
        
        param_loader params(params_file);
        // params.print();
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

        distance_function_t goal_dist = [&](space_point_t s1, space_point_t s2)
        {
            double diff = (s1 -> at(0) - s2 -> at(0)) * (s1 -> at(0) - s2 -> at(0)) + (s1 -> at(1) - s2 -> at(1)) * (s1 -> at(1) - s2 -> at(1));
            diff += norm_angle_pi(s1 -> at(2) - s2 -> at(2)) * norm_angle_pi(s1 -> at(2) - s2 -> at(2));
            return sqrt(diff);
        };

        dirt_query.goal_check = [&,dirt_spec,ss,goal_dist](space_point_t s)
        {
            // return dirt_spec.distance_function(s,dirt_query.goal_state) < dirt_query.goal_region_radius; 
            // return ss -> euclidean_2d(s, dirt_query.goal_state, 0, 3) < dirt_query.goal_region_radius;
            return goal_dist(s,dirt_query.goal_state) < dirt_query.goal_region_radius;
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
        if (!fs::exists(out_path))
        {
            fs::create_directory(out_path);
        }
        
        landmark_roadmap_t rrr;
        std::string roadmap_dir = output_path + params["roadmap_dir"].as<std::string>();
        std::vector<std::vector<double>> vertices = read_comma_separated_file(roadmap_dir + "/vertices.txt");

        graph_nearest_neighbors_t* metric = new graph_nearest_neighbors_t(goal_dist);
        
        for (auto& v : vertices)
        {
            landmark_node_t* node = new landmark_node_t();
            node->set_index(int(v[0]));
            node->point = ss -> make_point();
            // Copy from vector starting at index 1
            auto pv = std::vector<double>(v.begin() + 1, v.end());
            ss -> copy_point_from_vector(node->point,pv);
            rrr.add_vertex(dirt_spec,node->point,int(v[0]));

            metric->add_node(rrr.get_vertex(int(v[0])));
        }

        std::vector<std::vector<double>> edges = read_comma_separated_file(roadmap_dir + "/edges.txt");

        for (auto& e : edges)
        {
            rrr.add_edge(int(e[0]),int(e[1]),e[2]);
        }

        dirt_query_t controller_query(ss,cs);
        controller_query.start_state = ss -> make_point();
        controller_query.goal_state  = ss -> make_point();
        controller_query.goal_region_radius = params["goal_radius"].as<double>();
        controller_query.goal_check = [&,goal_dist,dirt_spec,ss](space_point_t s)
        {
            return goal_dist(s,controller_query.goal_state) < controller_query.goal_region_radius;
        };

        timer.reset();
        ss -> copy_point_from_vector(controller_query.start_state,s);
        auto s_nn = rrr.add_start(controller_query.start_state, dirt_spec, controller_query, controller);
        metric->add_node(rrr.get_vertex(s_nn));
        ss -> copy_point_from_vector(controller_query.goal_state,g);
        auto g_nn = rrr.add_goal(controller_query.goal_state, dirt_spec, controller_query, controller);
        metric->add_node(rrr.get_vertex(g_nn));
        std::cout << s_nn << " " << g_nn << std::endl;

        rrr.compute_wavefront(g_nn);
        std::cout << "Load time: " << timer.measure_reset() << std::endl;
        // dirt_spec.start_node_reachable_goal = rrr.get_vertex(s_nn)->get_successor();
        dirt_spec.start_node_reachable_goal = s_nn;
        std::cout << "Start node reachable goal: " << dirt_spec.start_node_reachable_goal << std::endl;

        space_point_t lg = ss -> make_point();
        std::vector<landmark_node_t*> roadmap_nodes;

        bool debug = params["debug"].as<bool>();

        dirt_spec.roadmap_h = [&](space_point_t s)
        {
            roadmap_nodes.clear();
            auto prox_nodes = metric -> radius_and_closest_query(s, 0.5);

            std::transform(prox_nodes.begin(),prox_nodes.end(),std::back_inserter(roadmap_nodes),[]
                    (proximity_node_t* prox_node)
            {
                return static_cast<landmark_node_t*>(prox_node);
            });

            double h = PRX_INFINITY;
            for (auto& node : roadmap_nodes)
            {
                double cost = node->get_node_cost_to_go();
                if (cost < h) h = cost;
            }
            return h;
        };
        
        dirt_spec.node_expand = [&](dirt_roadmap_node_t* tree_node, std::vector<plan_t*>& plans, std::vector<trajectory_t*>& trajs)
        {
            std::vector<std::vector<double>> current_states;
            std::vector<std::vector<double>> local_goals;

            std::vector<double> current_state;
            ss -> copy_vector_from_point(current_state,tree_node->point);
            std::vector<double> local_goal;

            if (tree_node->expand_num == 0)
            {
                roadmap_nodes.clear();
                auto prox_nodes = metric -> radius_query(tree_node->point, 0.5);

                std::transform(prox_nodes.begin(),prox_nodes.end(),std::back_inserter(roadmap_nodes),[]
                        (proximity_node_t* prox_node)
                {
                    return static_cast<landmark_node_t*>(prox_node);
                });

                int best_index = tree_node -> reachable_goal;
                if (roadmap_nodes.size() != 0)
                {
                    auto nn = roadmap_nodes[0];
                    if (debug) std::cout << "Inside " << nn -> get_index() << " : " << ss -> print_point(nn->point,2) << std::endl;
                    bool is_node_on_path = rrr.is_node_on_path(tree_node->reachable_goal, nn -> get_index());
                    if (debug) std::cout << "Is node on path: " << is_node_on_path << std::endl;

                    if (is_node_on_path)
                    {
                        best_index = rrr.get_next_local_goal(tree_node->point, controller_query, dirt_spec, controller, tree_node->reachable_goal);
                        if (debug) std::cout << "Local goal index[a]: " << best_index << std::endl;
                        if (best_index == nn -> get_index()) best_index = -1;
                        if (tree_node -> reachable_goal == nn -> get_index()) tree_node -> reachable_goal = -1;
                    }
                    else
                    {
                        best_index = rrr.get_next_local_goal(tree_node->point, controller_query, dirt_spec, controller, nn->get_successor());
                        if (debug) std::cout << "Local goal index[b]: " << best_index << std::endl;
                    }
                }

                if (best_index == -1) best_index = tree_node -> reachable_goal;
                if (best_index != -1)
                {
                    if (debug) std::cout << "Local goal: " << ss -> print_point(rrr.get_point(best_index), 4) << std::endl;
                    ss -> copy_point(lg, rrr.get_point(best_index));
                    tree_node -> reachable_goal = best_index;
                }
                else
                {
                    ss -> sample(lg);
                }

                local_goal.clear();
                ss -> copy_vector_from_point(local_goal,lg);
                current_states.push_back(current_state);
                local_goals.push_back(local_goal);

                auto controls = controller.get_controls(current_states,local_goals);

                trajectory_t traj(ss);
                plan_t plan(cs);

                traj.clear(); plan.clear();
                plan.append_onto_back(controller.get_control_duration());
                cs -> copy_point_from_vector(plan.back().control,controls[0]);
                dirt_spec.propagate(tree_node->point,plan,traj);
                plans.push_back(new plan_t(plan));
                trajs.push_back(new trajectory_t(traj));
            }
            else 
            {
                default_expand(tree_node->point,plans,trajs,1,sg,dirt_spec.sample_plan,dirt_spec.propagate);
            }
        };

        int stats_runs = 10;
        condition_check_t checker("time", 0.5);
        // int stats_runs = 1;
        // condition_check_t checker("iterations", 1000);
        // condition_check_t checker("time", 1);
        for (int i = 0; i < stats_runs; i++)
        {
            init_random(random_seed + i);
            dirt.link_and_setup_spec(&dirt_spec);
            dirt.preprocess();
            dirt.link_and_setup_query(&dirt_query);

            planner_statistics_t stats;
            stats.link_planner(&dirt);
            stats.link_criterion(&checker);
            simulation_time = 0.0;
            stats.repeat_data_gathering(60);
            // stats.repeat_data_gathering(20);
            dirt.print_statistics();
            simulation_time = 0.0;

            std::string full_name = out_path + params["planner_name"].as<std::string>()+"_"+ std::to_string(i) + ".txt";
            fout.open(full_name);
            fout << stats.serialize() << std::endl;
            fout.close();

            dirt.fulfill_query();
            three_js_group_t* vis_group = new three_js_group_t({plant},{obstacle_list});
            std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();
            vis_group -> add_vis_infos(info_geometry_t::LINE, dirt_query.tree_visualization, body_name, ss);
            vis_group -> add_detailed_vis_infos(info_geometry_t::FULL_LINE, dirt_query.solution_traj, body_name, ss);
            vis_group -> add_animation(dirt_query.solution_traj, ss, dirt_query.start_state);
            vis_group -> output_html(params["output_dir"].as<std::string>()+params["planner_name"].as<std::string>()+"_"+std::to_string(i)+".html");
            delete vis_group;

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