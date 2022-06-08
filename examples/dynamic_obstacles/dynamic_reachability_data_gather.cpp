#ifndef TORCH_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/rrt.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
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
            params_file = "examples/dynamic_obstacles/reach_data_gather.yaml";
            // prx_throw("The planner evaluation executable needs a parameter file!");
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

        auto obstacles = load_obstacles(params["environment"].as<std::string>());
        auto obstacle_list = obstacles.second;
        auto obstacle_names = obstacles.first;

        std::string plant_name = params["/plant/name"].as<std::string>();
        std::string plant_path = params["/plant/path"].as<std::string>();
        auto plant = system_factory_t::create_system(plant_name,plant_path);

        std::vector<double> lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
        std::vector<double> upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
        plant -> set_state_space_bounds(lower_bounds,upper_bounds);

        std::shared_ptr<world_model_t> sim(new world_model_t({plant},{obstacle_list}));
        sim -> create_context("dirt_context",{plant_name},{obstacle_names});
        auto context = sim -> get_context("dirt_context");

        auto ss = context.first->get_state_space();
        auto cs = context.first->get_control_space();
        auto sg = context.first;

        const int num_trajectories = params["num_trajectories"].as<int>();
        const double max_duration  = params["max_duration"].as<double>();
        const double control_duration = params["/learned_controller/control_duration"].as<double>();

        PRX_DEBUG_PRINT
        rrt_specification_t rrt_spec(context.first,context.second);

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

        for (int i = 0; i < num_trajectories; i++)
        {
            rrt_query.clear_outputs();
            ss->sample(rrt_query.start_state);
            ss->sample(rrt_query.goal_state);

            std::cout << ss->print_point(rrt_query.start_state,2) << " " 
            << ss->print_point(rrt_query.goal_state,2) << std::endl;

            controller.fulfill_query(rrt_query,sg,max_duration);

            std::ofstream ofs;
            ofs.open(out_path+output_dir+"/trajectory_"+std::to_string(i)+".txt");

            for (unsigned i = 0; i < rrt_query.solution_traj.size(); i += control_duration/simulation_step)
            {
                ss->copy_point(current,rrt_query.solution_traj[i]);
                ofs << ss->print_point(rrt_query.solution_traj[i],4) << "," << rrt_spec.valid_state(current) << std::endl;
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