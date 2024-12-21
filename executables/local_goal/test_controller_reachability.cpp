#ifndef TORCH_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/utilities/learned_modules/learned_controller.hpp"
#include "prx/utilities/learned_modules/planning/strict_reachable_roadmap.hpp"
#include "prx/utilities/learned_modules/planning/reachable_roadmap.hpp"
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
            params_file = "local_goal/controller_test.yaml";
            // params_file = "local_goal/annotate_treaded.yaml";
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

        std::cout << "Params Loaded" << std::endl;

        auto obstacles = load_obstacles(params["environment"].as<std::string>());
        auto obstacle_list = obstacles.second;
        auto obstacle_names = obstacles.first;
        
        std::cout << "Obstacles Loaded" << std::endl;

        std::string plant_name = params["/plant/name"].as<std::string>();
        std::string plant_path = params["/plant/path"].as<std::string>();
        auto plant = system_factory_t::create_system(plant_name,plant_path);

        std::cout << "System Loaded" << std::endl;

        std::vector<double> lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
        std::vector<double> upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
        plant -> set_state_space_bounds(lower_bounds,upper_bounds);

        world_model_t world_model({plant},{obstacle_list});
        world_model.create_context("planning_context",{plant_name},{obstacle_names});
        auto context = world_model.get_context("planning_context");

        std::cout << "World Constructed" << std::endl;

        auto ss = context.first->get_state_space();
        auto cs = context.first->get_control_space();
        auto sg = context.first;

        dirt_specification_t dirt_spec(context.first,context.second);
        dirt_spec.sample_state = [ss](space_point_t& s)
        {
        ss -> sample(s); s -> at(3) = s -> at(4) = 0.0;
        };
        dirt_query_t dirt_query(ss,cs);
        dirt_query.start_state = ss -> make_point();
        dirt_query.goal_state  = ss -> make_point();
        dirt_query.get_visualization = true;
        
        std::cout << "DIRT spec Initialized" << std::endl;

        dirt_query.goal_region_radius = params["goal_radius"].as<double>();

        learned_controller_t controller(params);

        std::cout << "Controller Loaded" << std::endl;

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

        std::cout << "DIRT Initialized" << std::endl;

        std::vector<double> s = params["start_state"].as<std::vector<double>>();
        std::vector<double> g = params["goal_state"].as<std::vector<double>>();
        ss -> copy_point_from_vector(dirt_query.start_state,s);
        ss -> copy_point_from_vector(dirt_query.goal_state,g);

        std::ofstream fout;
        std::string output_dir = params["output_dir"].as<std::string>();
        std::string out_path = output_path + output_dir;
        
        

        timer.reset();
        double time_taken = timer.measure_reset();

        // // Output graph to file.
        // std::string vertex_fname = out_path + "vertices.txt";
        // std::string edge_fname = out_path + "edges.txt";

        // std::ofstream vertex_file(vertex_fname);
        // std::ofstream edge_file(edge_fname);

        // vertex_file << rrr.print_vertices(ss);
        // edge_file << rrr.print_edges();

        // vertex_file.close();
        // edge_file.close();
        
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

        // // If the output directory does not exist, create it
        // if (!fs::exists(out_path))
        // {
        //     fs::create_directory(out_path);
        // }

        // rrr.print_components();


        double num_fail_traj = 0;
        double tot_num_traj = 0;

        for(int i = 0; i<1; i++){
            
            std::cout<<"Start trial: "<<i<<std::endl; 

            std::vector<double> pt_vec;
            space_point_t pt;
            pt = dirt_spec.state_space -> make_point();
            ///sample start and goal
            do
            {
                dirt_spec.sample_state(pt);    ///for some reason, this isnt working.
            } while (!dirt_spec.valid_state(pt));

            pt_vec.clear();
            
            dirt_spec.state_space -> copy_vector_from_point(pt_vec,pt);

            pt_vec = { -0.341736,7.833043,1.252700,-0.084688,0.00000};
            
            for( double val: pt_vec){
                std::cout << val << ", ";
            }
            std::cout << std::endl;

            ss -> copy_point_from_vector(dirt_query.start_state,pt_vec);

            std::vector<double> goal_vec(pt_vec.size());
            pt = dirt_spec.state_space -> make_point();
            ss -> copy_point_from_vector(dirt_query.goal_state,goal_vec);

            /////////////////////////////////////////////////////////////////////////////////////////

            //TODO: test controller.
            dirt_query.clear_outputs();
            controller.fulfill_query(dirt_query, dirt_spec);
            


            if (dirt_spec.valid_check(dirt_query.solution_traj) && dirt_query.solution_traj.size() > 1){
                
                num_fail_traj++;
            }

            std::cout << "trajectory"<< std::endl;

            for (auto s : dirt_query.solution_traj){
                std::cout << (ss->print_point(s)) << std::endl;
            }

            tot_num_traj++;

        }
        std::printf("controller efficacy: %f/%f",num_fail_traj,tot_num_traj);

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