#ifndef TORCH_NOT_BUILT

#include "prx/mujoco/mj_simulator.hpp"
#include "prx/utilities/learned_modules/learned_controller.hpp"
#include "prx/utilities/learned_modules/landmark_roadmap.hpp"
#include "prx/planning/planners/dirt_roadmap.hpp"
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

bool visualize_tree = true;
bool record_statistics = false;

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
    std::string params_file = "examples/mujoco/mushr_trajectory.yaml";
    if(argc>=2)
    {
        params_file = argv[1];
    }

    param_loader params(params_file);
    learned_controller_t controller(params);

    int random_seed = params["random_seed"].as<int>();
    init_random(random_seed);

    std::shared_ptr<mujoco_simulator_t> sim = std::make_shared<mujoco_simulator_t>("mushr_indoors.xml");
    sim->init_simulator();

    auto context = sim -> get_context("mujoco");
    auto ss = context.first -> get_state_space();
    auto cs = context.first -> get_control_space();
    auto sg = context.first;

    for (double i = 0; i < 1.0/simulation_step; i += 1)
    {
        sim -> step_simulation(propagate_step::FIRST_STEP);
    }

    dirt_roadmap_t dirt("dirt");
    dirt_roadmap_specification_t dirt_spec(context.first, context.second);
    dirt_spec.blossom_number = 1;

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
    
    dirt_spec.distance_function = [](const space_point_t& a, const space_point_t& b)
    {
        double diff = (a->at(0) - b->at(0)) * (a->at(0) - b->at(0)) + (a->at(1) - b->at(1)) * (a->at(1) - b->at(1));
        return sqrt(diff);
    };

    distance_function_t goal_dist = [](const space_point_t& a, const space_point_t& b)
    {
        double diff = (a->at(0) - b->at(0)) * (a->at(0) - b->at(0)) + (a->at(1) - b->at(1)) * (a->at(1) - b->at(1));
        // Get the Euler angles between the two quaternions
        quaternion_t quat1 = Eigen::Quaterniond(a->at(3), a->at(4), a->at(5), a->at(6));
        quaternion_t quat2 = Eigen::Quaterniond(b->at(3), b->at(4), b->at(5), b->at(6));
        double angular_diff = quat1.angularDistance(quat2);
        diff += angular_diff * angular_diff;
        return sqrt(diff);
    };

    dirt_spec.h = [&](const space_point_t& s, const space_point_t& s2)
    {
        return space_t::euclidean_2d(s, s2);
    };

    dirt_spec.min_control_steps = 0.5 * (1.0/simulation_step);
    dirt_spec.max_control_steps = 1.0 * (1.0/simulation_step);
    dirt_spec.use_pruning = false;
    std::cout << dirt_spec.min_control_steps << " " << dirt_spec.max_control_steps << std::endl;


    std::vector<double> start = params["start_state"].as<std::vector<double>>();
    
    double s_roll = start[2], s_pitch = start[3], s_yaw = start[4];
    Eigen::Quaterniond s_quat = Eigen::AngleAxisd(s_roll, Eigen::Vector3d::UnitX())
                            * Eigen::AngleAxisd(s_pitch, Eigen::Vector3d::UnitY())
                            * Eigen::AngleAxisd(s_yaw, Eigen::Vector3d::UnitZ());

    dirt_roadmap_query_t dirt_query(ss,cs);
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

    dirt_query.get_visualization = true;

    dirt_query.goal_check = [&](const space_point_t& point)
    {
        return goal_dist(point, dirt_query.goal_state) < 0.5;
    };

    space_point_t s = ss -> clone_point(dirt_query.start_state);
    space_point_t g = ss -> clone_point(dirt_query.goal_state);

    landmark_roadmap_t rrr;
    std::string rm_path = output_path + params["roadmap_dir"].as<std::string>();
    std::string out_path = output_path + params["output_dir"].as<std::string>();
    graph_nearest_neighbors_t* metric = new graph_nearest_neighbors_t(goal_dist);


    std::vector<std::vector<double>> vertices = read_comma_separated_file(rm_path + "/vertices.txt");
    for (auto& v : vertices)
    {
        auto p = ss -> make_point();
        auto pv = std::vector<double>(v.begin() + 1, v.end());
        ss -> copy_point_from_vector(p,pv);
        rrr.add_vertex(dirt_spec,p,int(v[0]));

        metric->add_node(rrr.get_vertex(int(v[0])));
    }

    std::vector<std::vector<double>> edges = read_comma_separated_file(rm_path + "/edges.txt");
    for (auto& e : edges)
    {
        rrr.add_edge(int(e[0]),int(e[1]),e[2]);
    }
    
    std::cout << "Loaded the roadmap." << std::endl;

    ss -> copy_point(dirt_query.start_state,s);
    auto s_nn = rrr.add_start(dirt_query.start_state, dirt_spec, dirt_query, controller);
    ss -> copy_point(dirt_query.goal_state,g);
    auto g_nn = rrr.add_goal(dirt_query.goal_state, dirt_spec, dirt_query, controller);

    prx_assert(s_nn != -1 && g_nn != -1, "Could not find a start or goal node!");
    rrr.compute_wavefront(g_nn);

    dirt_query.clear_outputs();
        
    ss -> copy_point(dirt_query.start_state,s);
    ss -> copy_point(dirt_query.goal_state,g);

    std::cout << ss -> print_point(dirt_query.start_state,4) << std::endl;
    std::cout << ss -> print_point(dirt_query.goal_state,4) << std::endl;

    dirt_spec.start_node_reachable_goal = s_nn;
    std::cout << "Start node reachable goal: " << dirt_spec.start_node_reachable_goal << std::endl;

    dirt_query_t controller_query(ss,cs);
    controller_query.start_state = ss -> make_point();
    controller_query.goal_state  = ss -> make_point();
    controller_query.goal_region_radius = 0.5;;
    controller_query.goal_check = [&,goal_dist,dirt_spec,ss](space_point_t s)
    {
        return goal_dist(s,controller_query.goal_state) < controller_query.goal_region_radius;
    };

    space_point_t lg = ss -> make_point();
    std::vector<landmark_node_t*> roadmap_nodes;

    dirt_spec.roadmap_h = [&](space_point_t s)
    {
        roadmap_nodes.clear();
        auto prox_nodes = metric -> radius_and_closest_query(s, 0.5);

        std::transform(prox_nodes.begin(),prox_nodes.end(),std::back_inserter(roadmap_nodes),[]
                (proximity_node_t* prox_node)
        {
            return static_cast<landmark_node_t*>(prox_node);
        });

        double h = PRX_INFINITY;
        for (auto& node : roadmap_nodes)
        {
            double cost = node->get_node_cost_to_go();
            if (cost < h) h = cost;
        }
        return h;
    };


    dirt_spec.node_expand = [&](dirt_roadmap_node_t* tree_node, std::vector<plan_t*>& plans, std::vector<trajectory_t*>& trajs)
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

            auto nn = roadmap_nodes[0];
            if (rrr.get_vertex(nn->get_index())->get_successor() != -1)
                ss -> copy_point(lg, rrr.get_point(rrr.get_vertex(nn->get_index())->get_successor()));
            else
                dirt_spec.sample_state(lg);

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

    if (!fs::exists(out_path))
        fs::create_directory(out_path);

    int stats_runs = 10;
    condition_check_t checker("time", 1.0);
    std::ofstream fout;
    for (int i = 0; i < stats_runs; i++)
    {
        init_random(random_seed + i);
        dirt.link_and_setup_spec(&dirt_spec);
        dirt.preprocess();
        dirt.link_and_setup_query(&dirt_query);

        planner_statistics_t stats;
        stats.link_planner(&dirt);
        stats.link_criterion(&checker);
        simulation_time = 0.0;
        stats.repeat_data_gathering(240);
        dirt.print_statistics();
        simulation_time = 0.0;

        std::string full_name = out_path + params["planner_name"].as<std::string>()+"_"+ std::to_string(i) + ".txt";
        fout.open(full_name);
        fout << stats.serialize() << std::endl;
        fout.close();

        // dirt.fulfill_query();
        // unsigned counter = 0;
        // for (auto& traj : dirt_query.tree_visualization)
        // {
        //     fout.open(output_path + "tree" + std::to_string(counter) + ".txt");
        //     fout << traj.print(2);
        //     fout.close();
        //     counter++;
        // }

        dirt_query.clear_outputs();
        dirt.reset();
    }
}
#else
int main() {}
#endif