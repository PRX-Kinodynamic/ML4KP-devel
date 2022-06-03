#ifndef TORCH_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/utilities/learned_modules/learned_expand.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/planning/planner_statistics.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/visualization/three_js_group.hpp"

using namespace prx;

int main(int argc, char* argv[])
{
    try
    {
        std::string params_file;
        if (argc <= 1)
        {
            prx_throw("The planner evaluation executable needs a parameter file!");
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
        double horizon = params["max_steps"].as<double>();

        auto obstacles = load_obstacles(params["environment"].as<std::string>());
        auto obstacle_list = obstacles.second;
        auto obstacle_names = obstacles.first;

        std::string plant_name = params["/plant/name"].as<std::string>();
        std::string plant_path = params["/plant/path"].as<std::string>();
        auto plant = system_factory_t::create_system(plant_name,plant_path);
        prx_assert(plant != nullptr, "Plant is nullptr!");

        std::vector<double> lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
        std::vector<double> upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
        plant -> set_state_space_bounds(lower_bounds,upper_bounds);

        world_model_t world_model({plant},{obstacle_list});
        world_model.create_context("planning_context",{plant_name},{obstacle_names});
        auto context = world_model.get_context("planning_context");

        auto ss = context.first->get_state_space();
        auto cs = context.first->get_control_space();
        auto sg = context.first;

        dirt_t dirt("dirt");
        dirt_specification_t dirt_spec(context.first,context.second);
        dirt_spec.min_control_steps = params["/plant/min_steps"].as<int>();
        dirt_spec.max_control_steps = params["/plant/max_steps"].as<int>();
        dirt_spec.blossom_number = params["blossom_number"].as<int>();
        dirt_spec.use_pruning = false;

        // Define distance function, heuristi function here

        dirt_query_t dirt_query(ss,cs);
        dirt_query.start_state = context.first->get_state_space()->make_point();
		dirt_query.goal_state = context.first->get_state_space()->make_point();
		std::vector <double> start_state_vec = params["/plant/start_state"].as<std::vector<double>>();
        std::vector <double> goal_state_vec = params["/plant/goal_state"].as<std::vector<double>>();

        //define graph 
        std::vector <double> n1 {0, 7.5, PRX_PI/2};
        std::vector <double> n2 {0, -7.5, PRX_PI/2};
        std::vector <double> n3 {goal_state_vec[0], goal_state_vec[1], goal_state_vec[2]};
        std::vector <std::vector <double>> nodes {n1, n2, n3};

        std::vector <std::pair <int,int>> edge_list 
            {std::make_pair(0,1), std::make_pair(1,0)};

        std::vector <double> weight_list {25.0, 25};

        for(int i=0; i<nodes.size()-1; i++){
            std::vector <double> node = nodes[i];
            node.push_back(0); node.push_back(0);
            dirt_query.clear_outputs();
            context.first->get_state_space()->copy_point_from_vector(dirt_query.start_state,node);
            context.first->get_state_space()->copy_point_from_vector(dirt_query.goal_state,goal_state_vec);
            controller.fulfill_query(dirt_query,sg,horizon);

            if (dirt_spec.valid_check(dirt_query.solution_traj))
            {
                edge_list.push_back(std::make_pair(i, nodes.size()-1));
                weight_list.push_back(dirt_query.solution_traj.size()/100.0);

                dirt_query.solution_traj.print();
                // ofs.close();
            }
        }

        //all-pairs shortest path lengths
        int V = 3;
        double INF = 100000;
        std::vector <std::vector <double>> dist (V, std::vector <double> {INF});

        for(int i=0; i<edge_list.size(); i++){
            std::pair <int, int> edge = edge_list[i];
            dist[edge.first][edge.second] = weight_list[i];
        }
        for(int i=0; i<V; i++){
            dist[i][i] = 0;
        }

        //Run Floyd-Warshall Algorithm to get all pairs shortest pathas
        for (int k = 0; k < V; k++) {
            // Pick all vertices as source one by one
            for (int i = 0; i < V; i++) {
                // Pick all vertices as destination for the
                // above picked source
                for (int j = 0; j < V; j++) {
                    // If vertex k is on the shortest path from
                    // i to j, then update the value of
                    // dist[i][j]
                    if (dist[i][j] > (dist[i][k] + dist[k][j])
                        && (dist[k][j] != INF
                            && dist[i][k] != INF))
                        dist[i][j] = dist[i][k] + dist[k][j];
                }
            }
        }

        // Define goal check function here
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

        dirt_query.goal_check = [&,ss](space_point_t point)
        {
            return dirt_spec.distance_function(point, dirt_query.goal_state) 
                < dirt_query.goal_region_radius;
        };
        
        auto get_local_goal = [&](std::vector <double> current_state_vec)
        {
            std::vector <std::pair<double, int>> f_values;
            for(int i=0; i<V; i++){
                //compute g_value
                std::vector <double> node = nodes[i];
                double g_value = 0;
                for(int i=0; i<2; i++){
                    double diff = current_state_vec[i]-node[i];
                    g_value += diff*diff;
                }
                g_value = sqrt(g_value)*0.7;

                double h_value = dist[i].back();

                double f_value = g_value + h_value;
                f_values.push_back(std::make_pair(f_value, i));
            }
            std::sort(f_values.begin(), f_values.end());

            std::vector <double> selected_node = nodes[f_values[0].second];

            std::vector <double> lg_prediction {selected_node[0], selected_node[1],
                selected_node[2], 0, 0};
            return lg_prediction;
        };

        double control_duration = 1.0;
        dirt_spec.expand = [&](space_point_t& s, std::vector<plan_t*>& plans, std::vector<trajectory_t*>& trajs, int bn, bool blossom_expand)
        {
            plans.clear();
            trajs.clear();
            std::vector <double> current_state_vec;
            if (blossom_expand)
            {
                sg->get_state_space()->copy_vector_from_point(current_state_vec,s);

                // Get local goal prediction.
                std::vector<double> lg_prediction = get_local_goal(current_state_vec);
                
                // Get controller prediction
                std::vector<double> controller_prediction = controller.get_control(current_state_vec, lg_prediction);
                
                trajectory_t traj(sg->get_state_space());
                plan_t plan(sg->get_control_space());

                plan.append_onto_back(control_duration);
                sg->get_control_space()->copy_point_from_vector(plan.back().control,controller_prediction);

                sg->propagate(s,plan,traj);

                plans.push_back(new plan_t(plan));
                trajs.push_back(new trajectory_t(traj));
            }

            else
            {
                sample_plan_t sample_plan;
                propagate_t propagate;
                default_expand(s,plans,trajs,bn,sg,sample_plan,propagate);
            }
        };

        int stats_runs = params["stats_runs"].as<int>();
        condition_check_t checker(params["checker_type"].as<std::string>(),params["checker_value"].as<double>());
        double stats_iters = params["stats_iters"].as<double>();
        std::ofstream fout;

        for( int i = 0; i < stats_runs; ++i )
        {
            dirt.link_and_setup_spec(&dirt_spec);
            dirt.preprocess();
            dirt.link_and_setup_query(&dirt_query);

            planner_statistics_t stats;
            stats.link_planner(&dirt);
            stats.link_criterion(&checker);
            stats.repeat_data_gathering(stats_iters);

            std::string full_filename = output_path+params["output_dir"].as<std::string>()+params["planner_name"].as<std::string>()+"_"+std::to_string(i)+".txt";
			fout.open(full_filename);
			fout<<stats.serialize() << std::endl;
			fout.close();

            // TODO: Add visualization code here.
            if (true)
            {
                dirt.fulfill_query();
                std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();
                three_js_group_t* vis_group = new three_js_group_t({plant},{obstacle_list});
                vis_group->add_vis_infos(info_geometry_t::LINE, dirt_query.tree_visualization, body_name, ss, "0x000000");
                vis_group->output_html(params["output_dir"].as<std::string>()+params["planner_name"].as<std::string>()+"_"+std::to_string(i)+".html");
                delete vis_group;
            }

            dirt.reset();
            dirt_query.clear_outputs();
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