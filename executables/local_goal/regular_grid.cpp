#ifndef TORCH_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/utilities/learned_modules/learned_controller.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/planning/planners/dirt.hpp"

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
        params.print();
        prx::timer_t timer; 
        simulation_step = params["simulation_step"].as<double>();
        int random_seed = params["random_seed"].as<int>();
        init_random(random_seed);

        auto obstacles = load_obstacles(params["environment"].as<std::string>());
        param_loader obstacles_file(params["environment"].as<std::string>());
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

        learned_controller_t controller(params);

        dirt_query.goal_check = [&,dirt_spec,ss](space_point_t s)
        {
            return dirt_spec.distance_function(s,dirt_query.goal_state) < dirt_query.goal_region_radius; 
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

        space_point_t current = ss -> make_point();
        std::vector<double> xs = linspace( 0. ,30., 20);
        std::vector<double> ys = linspace( 0. ,18., 12);
        std::vector<double> ts = linspace(-PRX_PI, PRX_PI, 8);

        trajectory_t traj(ss); plan_t plan(cs);
        std::vector<space_point_t> verification_points;

        std::vector<double> ps = linspace(-.2,.2,3);
        for (auto x : xs)
        {
            for (auto y : ys)
            {
                for (auto t : ts)
                {
                    current -> at(0) = x;
                    current -> at(1) = y;
                    current -> at(2) = t;

                    if (dirt_spec.valid_state(current))
                    {
                        verification_points.push_back(ss -> clone_point(current));
                    }
                }
            }
        }

        std::ofstream fout;
        std::string output_dir = params["output_dir"].as<std::string>();
        std::string out_path = output_path + output_dir;
        std::string out_file = out_path + "points.txt";
        fout.open(out_file);

        for (auto pt : verification_points)
        {
            fout << ss -> print_point(pt,4) << std::endl;
        }

        fout.close();

        /*
        std::string landmarks_file = out_path + "landmarks.txt";
        fout.open(landmarks_file);
        
        // auto landmarks = read_comma_separated_file(output_path + "landmarks.txt");
        // std::cout << landmarks.size() << std::endl;

        PRX_DEBUG_PRINT
        auto landmarks = obstacles_file["/environment/landmarks"].as<std::vector<std::vector<double>>>();

        for (int i = 0; i < landmarks.size(); i++)
        {
            for (int j = 0; j < landmarks.size(); j++)
            {
                if (i != j)
                {
                    dirt_query.clear_outputs();
                    ss -> copy_point_from_vector(dirt_query.start_state, landmarks[i]);
                    ss -> copy_point_from_vector(dirt_query.goal_state, landmarks[j]);
                    controller.fulfill_query(dirt_query,dirt_spec);
                    ss -> copy_point(current, dirt_query.solution_traj.back());
                    if (!dirt_query.goal_check(current) || !dirt_spec.valid_check(dirt_query.solution_traj))
                    {
                        fout << ss -> print_point(dirt_query.start_state,4) << "," << ss -> print_point(dirt_query.goal_state,4) << std::endl;
                    }
                }
            }
            output_progress_bar(1.0*i/landmarks.size());
        }

        fout.close();
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