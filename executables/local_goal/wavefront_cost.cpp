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
        // dirt_spec.use_pruning = true;

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

        ss -> copy_point_from_vector(controller_query.start_state,s);
        auto s_nn = rrr.add_start(controller_query.start_state, dirt_spec, controller_query, controller);
        metric->add_node(rrr.get_vertex(s_nn));
        ss -> copy_point_from_vector(controller_query.goal_state,g);
        auto g_nn = rrr.add_goal(controller_query.goal_state, dirt_spec, controller_query, controller);
        metric->add_node(rrr.get_vertex(g_nn));
        std::cout << s_nn << " " << g_nn << std::endl;

        rrr.compute_wavefront(g_nn);
        auto roadmap_vertices = rrr.get_vertices();

        double obs_cost_weight = 0.0;
        fout.open(out_path+"wavefront_costs.txt");

        for (auto v = roadmap_vertices.first; v != roadmap_vertices.second; v++)
        {
            auto node = v -> second;
            auto obs_dist = dirt_spec.obstacle_distance_function(node -> point);
            double min_obs_dist = PRX_INFINITY;
            for (auto dist : obs_dist.distances)
            {
                if (dist < min_obs_dist)
                {
                    min_obs_dist = dist;
                }
            }
            // std::cout << ss -> print_point(node -> point,4) << "," << node -> get_node_cost_to_go() << "," << min_obs_dist << std::endl;
            fout << ss -> print_point(node -> point,4) << "," << node -> get_node_cost_to_go() - obs_cost_weight*min_obs_dist << std::endl;
        }
        fout.close();
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