#ifndef TORCH_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/utilities/learned_modules/learned_controller.hpp"
// #include "prx/utilities/learned_modules/reachable_region_roadmap.hpp"
#include "prx/utilities/learned_modules/ground_truth_roadmap.hpp"
#include "prx/utilities/learned_modules/access_roadmap.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/planning/planner_statistics.hpp"
#include "prx/visualization/three_js_group.hpp"

#include <fstream>

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

        // reachable_region_roadmap_t rrr(params);
        // ground_truth_roadmap_t rrr;
        access_roadmap_t rrr(params);
        double roadmap_time_taken = 0.0;
        timer.reset();

        rrr.build_roadmap(dirt_query, dirt_spec, controller);
        std::cout << "Finished constructing the graph." << std::endl;

        roadmap_time_taken += timer.measure();
        std::cout << "Time taken for roadmap construction: " << roadmap_time_taken << std::endl;

        dirt_t dirt("dirt");
        dirt_spec.min_control_steps = params["/plant/min_steps"].as<int>();
        dirt_spec.max_control_steps = params["/plant/max_steps"].as<int>();
        dirt_spec.blossom_number = 1;
        dirt_spec.use_pruning = false;

        std::vector<double> s = params["/plant/start_state"].as<std::vector<double>>();
        std::vector<double> g = params["/plant/goal_state"].as<std::vector<double>>();
        ss -> copy_point_from_vector(dirt_query.start_state,s);
        ss -> copy_point_from_vector(dirt_query.goal_state,g);

        ss -> copy_point_from_vector(dirt_query.start_state,s);
        auto s_nn = rrr.add_start(dirt_query.start_state, dirt_spec, dirt_query, controller);
        ss -> copy_point_from_vector(dirt_query.goal_state,g);
        auto g_nn = rrr.add_goal(dirt_query.goal_state, dirt_spec, dirt_query, controller);
        ss -> copy_point_from_vector(dirt_query.start_state,s);
        ss -> copy_point_from_vector(dirt_query.goal_state,g);

        prx_assert(s_nn != -1 && g_nn != -1, "Could not find a start or goal node!");
        
        std::cout << "Nearest accessible node to start: " << s_nn << std::endl;

        auto path = rrr.get_shortest_path(s_nn,g_nn);
        for (auto v : path)
        {
            std::cout << v << " ";
        }
        std::cout << std::endl;

        dirt_query_t controller_query(ss,cs);
        controller_query.start_state = ss -> make_point();
        controller_query.goal_state  = ss -> make_point();
        controller_query.goal_region_radius = params["goal_radius"].as<double>();
        controller_query.goal_check = [&,dirt_spec,ss](space_point_t s)
        {
            return dirt_spec.distance_function(s,dirt_query.goal_state) < controller_query.goal_region_radius; 
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

                // auto nn = rrr.get_best_node(s,controller_query, dirt_spec, controller);
                auto nn = rrr.get_best_node(s,dirt_spec);
                // int nn = -1;
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

        std::ofstream fout;

        condition_check_t checker("time",1);
        for (int i = 0; i < 10; i++)
        {
            dirt.link_and_setup_spec(&dirt_spec);
            dirt.preprocess();
            dirt.link_and_setup_query(&dirt_query);

            planner_statistics_t stats;
            stats.link_planner(&dirt);
            stats.link_criterion(&checker);
            stats.repeat_data_gathering(10);

            std::string full_filename = output_path+params["output_dir"].as<std::string>()+std::to_string(i)+".txt";
			fout.open(full_filename);
			fout<<stats.serialize() << std::endl;
			fout.close();

            // dirt.fulfill_query();

            // three_js_group_t* vis_group = new three_js_group_t({plant},{obstacle_list});
            // std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();
            // vis_group -> add_vis_infos(info_geometry_t::LINE, dirt_query.tree_visualization, body_name, ss);
            // vis_group -> add_detailed_vis_infos(info_geometry_t::FULL_LINE, dirt_query.solution_traj, body_name, ss);
            // vis_group -> add_animation(dirt_query.solution_traj, ss, dirt_query.start_state);
            // vis_group -> output_html("output.html");
            // delete vis_group;

            dirt_query.clear_outputs();
            dirt.reset();
        }

        

        // Output graph to file.
        std::string vertex_fname = output_path + "vertices.txt";
        std::string edge_fname = output_path + "/edges.txt";

        std::ofstream vertex_file(vertex_fname);
        std::ofstream edge_file(edge_fname);

        vertex_file << rrr.print_vertices(ss) << std::endl;
        edge_file << rrr.print_edges() << std::endl;

        vertex_file.close();
        edge_file.close();
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