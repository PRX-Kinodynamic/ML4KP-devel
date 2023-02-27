#ifndef TORCH_NOT_BUILT

#include "prx/utilities/defs.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/utilities/learned_modules/learned_controller.hpp"
#include "prx/utilities/learned_modules/landmark_roadmap.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/planning/planners/dirt_roadmap.hpp"
#include "prx/planning/planner_statistics.hpp"
#include "prx/visualization/three_js_group.hpp"

#include "prx/mujoco/mj_simulator.hpp"

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
            params_file = "examples/mujoco/mushr_rm_trajectory.yaml";
            // params_file = "local_goal/car_like.yaml";
            // params_file = "local_goal/annotate_treaded.yaml";
        }
        else 
        {
            params_file = std::string(argv[1]);
        }
        
        param_loader params(params_file);
        //std::cout << "--- loaded params file ---" << std::endl;
        params.print();
        prx::timer_t timer; 
        int random_seed = params["random_seed"].as<int>();
        init_random(random_seed);
        //torch::set_num_threads(1);

        bool visualize_tree = params["visualize_tree"].as<bool>();
        bool record_statistics = params["record_statistics"].as<bool>();

        //std::cout << "--- init seed ---" << std::endl;

        std::string plant_name = params["/plant/name"].as<std::string>();
        std::string plant_path = params["/plant/path"].as<std::string>();
        auto plant = system_factory_t::create_system(plant_name,plant_path);
        //std::cout << "--- loaded plant ---" << std::endl;

        std::shared_ptr<mujoco_simulator_t> sim = std::make_shared<mujoco_simulator_t>(params["environment"].as<std::string>());
        sim->init_simulator();

        //std::cout << "--- made mujoco simulator ---" << std::endl;

        auto context = sim -> get_context("mujoco");
        auto ss = context.first -> get_state_space();
        auto cs = context.first -> get_control_space();
        auto sg = context.first;

        for (double i = 0; i < 1.0/simulation_step; i += 1)
        {
            sim -> step_simulation(propagate_step::FIRST_STEP);
        }
        //std::cout << "--- made context ---" << std::endl;

        dirt_roadmap_t dirt("dirt");
        dirt_roadmap_specification_t dirt_spec(context.first,context.second);
        dirt_roadmap_query_t dirt_query(ss,cs);
        dirt_query.start_state = ss -> make_point();
        dirt_query.goal_state  = ss -> make_point();
        dirt_query.get_visualization = true;
        //std::cout << "--- made dirt query ---" << std::endl;

        dirt_query.goal_region_radius = params["goal_radius"].as<double>();
        //std::cout << "--- loaded goal radius ---" << std::endl;

        learned_controller_t controller(params);
        //std::cout << "--- made controller ---" << std::endl;
        
        dirt_spec.sample_state = [ss](space_point_t& s)
        {
            s->at(0) = uniform_random(-9., 9.);
            s->at(1) = uniform_random(-9., 9.);
            double roll = 0, pitch = 0, yaw = uniform_random(-PRX_PI, PRX_PI);
            Eigen::Quaterniond quat = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
                                    * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                                    * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
            s->at(2) = 0.0;
            s->at(3) = quat.w();
            s->at(4) = quat.x();
            s->at(5) = quat.y();
            s->at(6) = quat.z();
        };
    
        dirt_spec.distance_function = [&](space_point_t a, space_point_t b)
        {
            
            double diff = (a->at(0) - b->at(0)) * (a->at(0) - b->at(0)) + (a->at(1) - b->at(1)) * (a->at(1) - b->at(1));
            // Get the Euler angles between the two quaternions
            quaternion_t quat1 = Eigen::Quaterniond(a->at(3), a->at(4), a->at(5), a->at(6));
            quaternion_t quat2 = Eigen::Quaterniond(b->at(3), b->at(4), b->at(5), b->at(6));
            double angular_diff = quat1.angularDistance(quat2);
            diff += angular_diff * angular_diff;
            return sqrt(diff);
            //return ss -> euclidean_2d(a,b,0,2);
        };


        dirt_spec.h = [&](space_point_t s, space_point_t d)
        {
            double diff = (s->at(0) - d->at(0)) * (s->at(0) - d->at(0)) + (s->at(1) - d->at(1)) * (s->at(1) - d->at(1));

            quaternion_t quat1 = Eigen::Quaterniond(s->at(3), s->at(4), s->at(5), s->at(6));
            quaternion_t quat2 = Eigen::Quaterniond(d->at(3), d->at(4), d->at(5), d->at(6));
            double angular_diff = quat1.angularDistance(quat2);
            diff += angular_diff * angular_diff;
            return sqrt(diff);
            //return ss -> euclidean_2d(s,d,0,2);
        };

        distance_function_t goal_dist = [&](space_point_t s1, space_point_t s2)
        {
            double diff = (s1->at(0) - s2->at(0)) * (s1->at(0) - s2->at(0)) + (s1->at(1) - s2->at(1)) * (s1->at(1) - s2->at(1));

            quaternion_t quat1 = Eigen::Quaterniond(s1->at(3), s1->at(4), s1->at(5), s1->at(6));
            quaternion_t quat2 = Eigen::Quaterniond(s2->at(3), s2->at(4), s2->at(5), s2->at(6));
            double angular_diff = quat1.angularDistance(quat2);
            diff += angular_diff * angular_diff;
            return sqrt(diff);
        };

        dirt_query.goal_check = [&,dirt_spec,ss,goal_dist](space_point_t s)
        {
            // return dirt_spec.distance_function(s,dirt_query.goal_state) < dirt_query.goal_region_radius; 
            // return ss -> euclidean_2d(s, dirt_query.goal_state, 0, 3) < dirt_query.goal_region_radius;
            return goal_dist(s,dirt_query.goal_state) < dirt_query.goal_region_radius;
        };

        //std::cout << "--- lambdas defined ---" << std::endl;

        
        dirt_spec.min_control_steps = params["/plant/min_steps"].as<int>();
        dirt_spec.max_control_steps = params["/plant/max_steps"].as<int>();
        dirt_spec.blossom_number = 1;
        dirt_spec.use_pruning = false;
        //std::cout << "--- made dirt_roadmap ---" << std::endl;

        std::vector<double> start = params["start_state"].as<std::vector<double>>();

        double s_roll = start[2], s_pitch = start[3], s_yaw = start[4];
        Eigen::Quaterniond s_quat = Eigen::AngleAxisd(s_roll, Eigen::Vector3d::UnitX())
                                * Eigen::AngleAxisd(s_pitch, Eigen::Vector3d::UnitY())
                                * Eigen::AngleAxisd(s_yaw, Eigen::Vector3d::UnitZ());


        dirt_query.start_state = ss -> make_point();
        ss -> copy_to_point(dirt_query.start_state);
        dirt_query.start_state ->at(0) =  start[0];
        dirt_query.start_state ->at(1) =  start[1];
        dirt_query.start_state ->at(3) = s_quat.w();
        dirt_query.start_state ->at(4) = s_quat.x();
        dirt_query.start_state ->at(5) = s_quat.y();
        dirt_query.start_state ->at(6) = s_quat.z();


        std::vector<double> goal = params["goal_state"].as<std::vector<double>>();

        double g_roll = goal[2], g_pitch = goal[3], g_yaw = goal[4];
        Eigen::Quaterniond g_quat = Eigen::AngleAxisd(g_roll, Eigen::Vector3d::UnitX())
                                * Eigen::AngleAxisd(g_pitch, Eigen::Vector3d::UnitY())
                                * Eigen::AngleAxisd(g_yaw, Eigen::Vector3d::UnitZ());
        dirt_query.goal_state = ss -> make_point();
        ss -> copy_to_point(dirt_query.goal_state);
        dirt_query.goal_state -> at(0) = goal[0];
        dirt_query.goal_state -> at(1) = goal[1];
        dirt_query.goal_state -> at(3) = g_quat.w();
        dirt_query.goal_state -> at(4) = g_quat.x();
        dirt_query.goal_state -> at(5) = g_quat.y();
        dirt_query.goal_state -> at(6) = g_quat.z();

        //std::cout << "--- loaded start and goal ---" << std::endl;

        //ss -> copy_point_from_vector(dirt_query.start_state,s);
        //ss -> copy_point_from_vector(dirt_query.goal_state,g);

        std::ofstream fout;
        std::string output_dir = params["output_dir"].as<std::string>();
        std::string out_path = output_path + output_dir;
        //std::cout << "--- set output path ---" << std::endl;
        
        landmark_roadmap_t rrr;
        std::string roadmap_dir = input_path + params["roadmap_dir"].as<std::string>();
        std::vector<std::vector<double>> vertices = read_comma_separated_file(roadmap_dir + "/vertices.txt");
        //std::cout << "--- read roadmap vertices file---" << std::endl;

        graph_nearest_neighbors_t* metric = new graph_nearest_neighbors_t(goal_dist);
        //std::cout << "--- made nn metric ---" << std::endl;

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
        //std::cout << "--- loaded roadmap vertices ---" << std::endl;

        std::vector<std::vector<double>> edges = read_comma_separated_file(roadmap_dir + "/edges.txt");

        //std::cout << "--- read edges file ---" << std::endl;

        for (auto& e : edges)
        {
            rrr.add_edge(int(e[0]),int(e[1]),e[2]);
        }

        //std::cout << "--- loaded roadmap edges ---" << std::endl;



        dirt_query_t controller_query(ss,cs);
        controller_query.start_state = ss -> make_point();
        controller_query.goal_state  = ss -> make_point();
        controller_query.goal_region_radius = params["goal_radius"].as<double>();
        controller_query.goal_check = [&,goal_dist,dirt_spec,ss](space_point_t s)
        {
            return goal_dist(s,controller_query.goal_state) < controller_query.goal_region_radius;
        };
        //std::cout << "--- made controller query ---" << std::endl;


        ss -> copy_to_point(controller_query.start_state);
        controller_query.start_state -> at(0) = start[0];
        controller_query.start_state -> at(1) = start[1];
        controller_query.start_state -> at(3) = s_quat.w();
        controller_query.start_state -> at(4) = s_quat.x();
        controller_query.start_state -> at(5) = s_quat.y();
        controller_query.start_state -> at(6) = s_quat.z();

        auto s_nn = rrr.add_start(controller_query.start_state, dirt_spec, controller_query, controller);
        metric->add_node(rrr.get_vertex(s_nn));

        ss -> copy_to_point(controller_query.goal_state);
        controller_query.goal_state -> at(0) = goal[0];
        controller_query.goal_state -> at(1) = goal[1];
        controller_query.goal_state -> at(3) = g_quat.w();
        controller_query.goal_state -> at(4) = g_quat.x();
        controller_query.goal_state -> at(5) = g_quat.y();
        controller_query.goal_state -> at(6) = g_quat.z();

        //std::cout << "--- added controller start/goal ---" << std::endl;


        auto g_nn = rrr.add_goal(controller_query.goal_state, dirt_spec, controller_query, controller);
        metric->add_node(rrr.get_vertex(g_nn));
        //std::cout << s_nn << " " << g_nn << std::endl;

        rrr.compute_wavefront(g_nn);
        dirt_spec.start_node_reachable_goal = s_nn;
        std::cout << "Start node reachable goal: " << dirt_spec.start_node_reachable_goal << std::endl;

        space_point_t lg = ss -> make_point();
        std::vector<landmark_node_t*> roadmap_nodes;

        //std::cout << ss -> print_point(dirt_query.start_state,2) << std::endl;
        //std::cout << ss -> print_point(dirt_query.goal_state,2) << std::endl;

        dirt_spec.node_expand = [&](dirt_roadmap_node_t* tree_node, std::vector<plan_t*>& plans, std::vector<trajectory_t*>& trajs, bool& override_child_extension)
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
                    if (tree_node -> roadmap_cost_to_go >= nn -> get_node_cost())
                    {
                        tree_node -> roadmap_cost_to_go = nn -> get_node_cost();
                        bool is_node_on_path = rrr.is_node_on_path(tree_node->reachable_goal, nn -> get_index());

                        if (is_node_on_path && tree_node->greedy_expand)
                        {
                            best_index = rrr.get_next_local_goal(tree_node->point, controller_query, dirt_spec, controller, tree_node->reachable_goal);
                            if (best_index == nn -> get_index()) best_index = -1;
                            if (tree_node -> reachable_goal == nn -> get_index()) tree_node -> reachable_goal = -1;
                        }
                        else
                        {
                            best_index = rrr.get_next_local_goal(tree_node->point, controller_query, dirt_spec, controller, nn->get_successor());
                        }
                    }
                    else
                    {
                        if (!tree_node -> greedy_expand)
                        {
                            best_index = rrr.get_next_local_goal(tree_node->point, controller_query, dirt_spec, controller, nn->get_successor());
                        }
                    }
                }

                if (best_index == -1) best_index = tree_node -> reachable_goal;
                if (best_index != -1)
                {
                    override_child_extension = true;
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


        std::cout << ss -> print_point(rrr.get_point(rrr.get_vertex(401) -> get_successor()));


        if(visualize_tree && !record_statistics)
        {
            dirt.link_and_setup_spec(&dirt_spec);
            dirt.preprocess();
            dirt.link_and_setup_query(&dirt_query);
            //condition_check_t checker("time", 30.0);
            condition_check_t checker("iterations", 60);
            dirt.resolve_query(&checker);
            dirt.fulfill_query(); 

            unsigned counter = 0;
            for (auto& traj : dirt_query.tree_visualization)
            {

                std::string out_path = output_path + params["output_dir"].as<std::string>()+ params["planner_name"].as<std::string>()+"_tree/";
                if (!fs::exists(out_path))
                {
                    fs::create_directory(out_path);
                }
                fout.open(out_path+ "tree" + std::to_string(counter) + ".txt");
                fout << traj.print(2);
                fout.close();
                counter++;
            }
            

            fout.open(output_path + "solution.txt");
            fout << dirt_query.solution_traj.print(4);
            fout.close();
        }



        if(record_statistics)
        { 
            int stats_runs = 10;
            condition_check_t checker("iterations", 10);
            for (int i = 0; i < stats_runs; i++)
            {
                init_random(random_seed + i);
                dirt.link_and_setup_spec(&dirt_spec);
                dirt.preprocess();
                dirt.link_and_setup_query(&dirt_query);

                planner_statistics_t stats;
                stats.link_planner(&dirt);
                stats.link_criterion(&checker);
                // simulation_time = 0.0;
                stats.repeat_data_gathering(60);
                // stats.repeat_data_gathering(20);
                dirt.print_statistics();
                // simulation_time = 0.0;

                std::string full_name = out_path + params["planner_name"].as<std::string>()+"_"+ std::to_string(i) + ".txt";
                fout.open(full_name);
                fout << stats.serialize() << std::endl;
                fout.close();

                dirt.fulfill_query();

                if(visualize_tree){
                    unsigned counter = 0;
                    for (auto& traj : dirt_query.tree_visualization)
                    {
                        std::string out_path = output_path + params["output_dir"].as<std::string>()+ params["planner_name"].as<std::string>()+"_tree_"+std::to_string(i)+"/";
                        if (!fs::exists(out_path))
                        {
                            fs::create_directory(out_path);
                        }
                        fout.open(out_path+ "tree" + std::to_string(counter) + ".txt");
                        fout << traj.print(2);
                        fout.close();
                        counter++;
                    }
                    fout.open(output_path + "solution.txt");
                    fout << dirt_query.solution_traj.print(4);
                    fout.close();
                }

                dirt_query.clear_outputs();
                dirt.reset();
            }
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