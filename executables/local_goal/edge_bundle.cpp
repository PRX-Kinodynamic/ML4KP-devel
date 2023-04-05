#ifndef TORCH_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/utilities/learned_modules/learned_controller.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/planning/planners/edge_bundle.hpp"
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
            params_file = "local_goal/car_like.yaml";
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

        edge_bundle_planner_t edge_bundle("edge_bundle");
        edge_bundle_planner_specification_t spec(context.first,context.second);
        spec.min_control_steps = params["/plant/min_steps"].as<int>();
        spec.max_control_steps = params["/plant/max_steps"].as<int>();

        edge_bundle_planner_query_t query(ss,cs);
        query.start_state = ss -> make_point();
        ss -> copy_point_from_vector(query.start_state, params["start_state"].as<std::vector<double>>());
        query.goal_state = ss -> make_point();
        ss -> copy_point_from_vector(query.goal_state, params["goal_state"].as<std::vector<double>>());
        query.get_visualization = true;

        spec.sample_state = [&](space_point_t& s)
        {
            ss -> sample(s);
            s -> at(0) = uniform_random(-10.,10.);
            s -> at(1) = uniform_random(-6.,6.);
        };

        // query.goal_check = [&](space_point_t s)
        // {
        //     double diff = (s->at(0) - query.goal_state->at(0))*(s->at(0) - query.goal_state->at(0)) + (s->at(1) - query.goal_state->at(1))*(s->at(1) - query.goal_state->at(1));
        //     diff += norm_angle_pi(s->at(2) - query.goal_state->at(2))*norm_angle_pi(s->at(2) - query.goal_state->at(2));
        //     return std::sqrt(diff) < query.goal_region_radius;
        // };

        condition_check_t checker("iterations",1000);

        edge_bundle.link_and_setup_spec(&spec);
        edge_bundle.preprocess();
        edge_bundle.link_and_setup_query(&query);
        edge_bundle.resolve_query(&checker);
        edge_bundle.fulfill_query();

        std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();
        three_js_group_t* vis_group = new three_js_group_t({plant},{obstacle_list});
        vis_group->add_vis_infos(info_geometry_t::LINE, query.tree_visualization, body_name, ss, "0x000000");
        vis_group -> add_detailed_vis_infos(info_geometry_t::FULL_LINE, query.solution_traj, body_name, ss);
        vis_group -> add_animation(query.solution_traj, ss, query.start_state);
        vis_group->output_html("edge_bundle.html");
        delete vis_group;
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