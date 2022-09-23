#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/sst.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
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

int main(int argc, char* argv[])
{
    param_loader params;
    if (argc > 1)
    {
        params = param_loader(argv[1]);
    }
    else
    {
        params = param_loader("examples/benchmark/run_sst.yaml");
    }

    simulation_step = params["simulation_step"].as<double>();
    init_random(params["random_seed"].as<int>());

    auto obstacles = load_obstacles(params["environment"].as<>());
    std::vector<std::shared_ptr<movable_object_t>> obstacle_list = obstacles.second;
    std::vector<std::string> obstacle_names = obstacles.first;

    std::string plant_name = params["/plant/name"].as<>();
    std::string plant_path = params["/plant/path"].as<>();
    auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
    prx_assert(plant != nullptr, "Plant is nullptr!");

    world_model_t world_model({plant},{obstacle_list});
    world_model.create_context("sst_context",{plant_name},{obstacle_names});
    auto context = world_model.get_context("sst_context");

    sst_t sst(params["planner"].as<>());
    sst_specification_t sst_spec(context.first,context.second);

    sst_spec.distance_function = [&](const space_point_t& s1, const space_point_t& s2)
    {
        return space_t::euclidean_2d(s1, s2, 0, 5);
    };

    sst_spec.delta_near        = params["delta_near"].as<double>();
    sst_spec.delta_drain       = params["delta_drain"].as<double>();

    sst_spec.min_control_steps = params["/plant/min_steps"].as<int>();
    sst_spec.max_control_steps = params["/plant/max_steps"].as<int>();

    sst_query_t sst_query(context.first->get_state_space(),context.first->get_control_space());
    sst_query.start_state = context.first->get_state_space()->make_point();
    sst_query.goal_state  = context.first->get_state_space()->make_point();

    auto lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
    auto upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
    context.first -> get_state_space() -> set_bounds(lower_bounds, upper_bounds);

    context.first -> get_state_space() -> copy_point_from_vector(sst_query.start_state, params["/plant/start_state"].as<std::vector<double>>());
    context.first -> get_state_space() -> copy_point_from_vector(sst_query.goal_state, params["/plant/goal_state"].as<std::vector<double>>());
    
    sst_query.goal_region_radius = params["goal_region_radius"].as<double>();

    sst_query.goal_check = [&](const space_point_t& s)
    {
        return space_t::euclidean_2d(s, sst_query.goal_state, 0, s->size()) < sst_query.goal_region_radius;
    };

    if (plant_name == "Acrobot")
    {
        sst_spec.distance_function = [&](const space_point_t& s1, const space_point_t& s2)
        {
            double cost = 0;
            double s1a0 = s1->at(0) + PRX_PI;
            double s1a1 = s1->at(1) + PRX_PI;
            double s2a0 = s2->at(0) + PRX_PI;
            double s2a1 = s2->at(1) + PRX_PI;

            double s1a2 = s1->at(2);
            double s1a3 = s1->at(3);
            double s2a2 = s2->at(2);
            double s2a3 = s2->at(3);

            double a0 = std::min(std::abs(s1a0 - s2a0), 2 * PRX_PI - std::abs(s1a0 - s2a0));
            double a1 = std::min(std::abs(s1a1 - s2a1), 2 * PRX_PI - std::abs(s1a1 - s2a1));
            double a2 = s1a2 - s2a2;
            double a3 = s1a3 - s2a3;

            cost += a0 * a0 + a1 * a1 + a2 * a2 + a3 * a3;
            return std::sqrt(cost);
        };

        sst_query.goal_check = [&](const space_point_t& s)
        {
            return sst_spec.distance_function(s, sst_query.goal_state) < sst_query.goal_region_radius;
        };

    }

    sst_query.get_visualization = params["visualize"].as<bool>();

    const int stats_runs = 10;
    condition_check_t checker("time", 1.0);
    const int num_calls = params["planning_time"].as<int>();

    if (!fs::exists(output_path + params["output_dir"].as<>()))
        fs::create_directory(output_path + params["output_dir"].as<>());


    for (int i = 0; i < stats_runs; i++)
    {
        sst.link_and_setup_spec(&sst_spec);
        sst.preprocess();
        sst.link_and_setup_query(&sst_query);

        planner_statistics_t stats;
        stats.link_planner(&sst);
        stats.link_criterion(&checker);
        stats.repeat_data_gathering(num_calls, false);

        std::string fname = output_path + params["output_dir"].as<>()+"/" + std::to_string(i) + ".txt";
        std::ofstream out(fname);
        out << stats.serialize();
        out.close();

        // sst.fulfill_query();
        sst.reset();
        sst_query.clear_outputs();
        
        output_progress_bar (1.0 * i/stats_runs);
    }

    // three_js_group_t* vis_group = new three_js_group_t({plant},{obstacle_list});
    // std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();
    // auto ss = context.first -> get_state_space();
    // vis_group -> add_vis_infos(info_geometry_t::LINE, sst_query.tree_visualization, body_name, ss);
    // vis_group -> add_detailed_vis_infos(info_geometry_t::FULL_LINE, sst_query.solution_traj, body_name, ss);
    // vis_group -> add_animation(sst_query.solution_traj, ss, sst_query.start_state);
    // vis_group -> output_html("output.html");
    // delete vis_group;
}
