#ifndef TORCH_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/utilities/learned_modules/learned_controller.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/planning/planner_statistics.hpp"

#ifdef __cpp_lib_filesystem
    #include <boost/filesystem.hpp>
    namespace fs = boost::filesystem;
#else
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
            params_file = "examples/test_treaded_rlg_0.yaml";
            // prx_throw("This executable needs a parameter file!");
        }
        else 
        {
            params_file = std::string(argv[1]);
        }
        
        std::cout << "Using params file: " << params_file << std::endl;
        param_loader params(params_file);
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

        learned_controller_t controller(params);
        double duration = params["/learned_controller/control_duration"].as<double>();
        bool use_random_local_goal = params["random_local_goal"].as<bool>();

        dirt_t dirt("dirt");

        dirt_specification_t dirt_spec(context.first,context.second);
        dirt_spec.min_control_steps = params["/plant/min_steps"].as<double>()/simulation_step;
        dirt_spec.max_control_steps = params["/plant/max_steps"].as<double>()/simulation_step;
        dirt_spec.blossom_number = 1;
        dirt_spec.use_pruning = false;
        space_point_t sample_point = ss -> make_point();

        dirt_query_t dirt_query(ss,cs);
        dirt_query.start_state = ss -> make_point();
        ss -> copy_point_from_vector(dirt_query.start_state,params["start_state"].as<std::vector<double>>());
        dirt_query.goal_state = ss -> make_point();
        ss -> copy_point_from_vector(dirt_query.goal_state,params["goal_state"].as<std::vector<double>>());
        
        dirt_query.get_visualization = true;
        dirt_query.goal_region_radius = params["goal_radius"].as<double>();

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
            // return ss -> euclidean_2d(s, dirt_query.goal_state, 0, 3) < dirt_query.goal_region_radius;
            double diff = (s -> at(0) - dirt_query.goal_state -> at(0)) * (s -> at(0) - dirt_query.goal_state -> at(0)) + (s -> at(1) - dirt_query.goal_state -> at(1)) * (s -> at(1) - dirt_query.goal_state -> at(1));
            diff += norm_angle_pi(s -> at(2) - dirt_query.goal_state -> at(2)) * norm_angle_pi(s -> at(2) - dirt_query.goal_state -> at(2));
            return std::sqrt(diff) < dirt_query.goal_region_radius;
        };

        dirt_spec.expand = [&](space_point_t& s, std::vector<plan_t*>& plans, std::vector<trajectory_t*>& trajs, int bn, bool blossom_expand)
        {
            if (blossom_expand && use_random_local_goal)
            {
                std::vector<std::vector<double>> current_states;
                std::vector<std::vector<double>> local_goals;

                std::vector<double> current_state;
                ss -> copy_vector_from_point(current_state,s);
                std::vector<double> local_goal;

                for (int i = 0; i < bn; i++)
                {
                    local_goal.clear();
                    ss -> sample(sample_point);
                    ss -> copy_vector_from_point(local_goal,sample_point);
                    current_states.push_back(current_state);
                    local_goals.push_back(local_goal);
                }

                auto controls = controller.get_controls(current_states,local_goals);

                trajectory_t traj(ss);
                plan_t plan(cs);

                for (int i = 0; i < bn; i++)
                {
                    traj.clear(); plan.clear();
                    plan.append_onto_back(duration);
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

        int stats_runs = 10;
        condition_check_t checker("time", 0.5);
        // condition_check_t checker("time",30);
        // condition_check_t checker("solutions",1);
        std::ofstream fout;

        std::string output_dir = params["output_dir"].as<std::string>();
        std::string out_path = output_path + output_dir;
        if (!fs::exists(out_path))
        {
            fs::create_directory(out_path);
        }

        for( int i = 0; i < stats_runs; ++i )
        {
            dirt.link_and_setup_spec(&dirt_spec);
            dirt.preprocess();
            dirt.link_and_setup_query(&dirt_query);

            planner_statistics_t stats;
            stats.link_planner(&dirt);
            stats.link_criterion(&checker);
            simulation_time = 0.0;
            // stats.repeat_data_gathering(60);
            stats.repeat_data_gathering(20);
            simulation_time = 0.0;

            std::string full_name = out_path + params["planner_name"].as<std::string>()+"_"+ std::to_string(i) + ".txt";
            fout.open(full_name);
            fout << stats.serialize() << std::endl;
            fout.close();

            // std::string full_fname = out_path + params["planner_name"].as<std::string>()+"_"+ std::to_string(i) + ".txt";
            // fout.open(full_fname);
            // fout << dirt.get_current_solution() << std::endl;
            // fout << dirt.get_current_solution_time() << std::endl;
            // fout << dirt.get_current_solution_iters() << std::endl;
            // fout << dirt.get_branching_factor() << std::endl;
            // fout << end_sim_time << std::endl;
            // fout.close();

            // TODO: Add visualization code here.
            if (true)
            {
                dirt.fulfill_query();
                std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();
                three_js_group_t* vis_group = new three_js_group_t({plant},{obstacle_list});
                vis_group->add_vis_infos(info_geometry_t::FULL_LINE, dirt_query.tree_visualization, body_name, ss, "0x000000");
                vis_group->output_html(params["output_dir"].as<std::string>()+params["planner_name"].as<std::string>()+"_"+std::to_string(i)+".html");
                delete vis_group;
            }

            dirt.reset();
            dirt_query.clear_outputs();
            checker.reset();
        }

        // std::cout << dirt_query.tree_visualization.size() << std::endl;

        // three_js_group_t* vis_group = new three_js_group_t({plant},{obstacle_list});
        // std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();
        // vis_group -> add_vis_infos(info_geometry_t::LINE, dirt_query.tree_visualization, body_name, ss);
        // vis_group -> add_detailed_vis_infos(info_geometry_t::FULL_LINE, dirt_query.solution_traj, body_name, ss);
        // vis_group -> add_animation(dirt_query.solution_traj, ss, dirt_query.start_state);
        // vis_group -> output_html("output.html");

        // delete vis_group;
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