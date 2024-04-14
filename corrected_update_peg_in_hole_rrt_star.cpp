#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <memory>
#include <random>

#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/rrt_star.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/simulation/environment.hpp"  // Assuming environment-related functionalities

using namespace prx;

// Function to read the tree from file
void readTreeFromFile(const std::string& filename, rrt_star_t& tree) 
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    std::string line;
    std::getline(file, line); // Skip the header line

    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue; // Skip empty lines or comments

        std::istringstream iss(line);
        int parent_idx, edge_idx, node_idx;
        double x, y, z, qw, qx, qy, qz;

        if (!(iss >> parent_idx >> edge_idx >> node_idx >> x >> y >> z >> qw >> qx >> qy >> qz)) {
            std::cerr << "Error parsing line: " << line << std::endl;
            continue;
        }

        space_point_t point = tree.get_state_space()->make_point();
        point->at(0) = x;
        point->at(1) = y;
        point->at(2) = z;
        point->at(3) = qw;
        point->at(4) = qx;
        point->at(5) = qy;
        point->at(6) = qz;
        tree.add_node(point, parent_idx, edge_idx, node_idx);
    }
}

std::vector<Eigen::VectorXd> readSuggestedStates(const std::string& filename) 
{
    std::vector<Eigen::VectorXd> states;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Cannot open states file." << std::endl;
        return states;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        Eigen::VectorXd state(7);
        for (int i = 0; i < 7; ++i) {
            if (!(iss >> state[i])) {
                std::cerr << "State parsing error." << std::endl;
                break;
            }
            if (i < 6) iss.ignore();
        }
        states.push_back(state);
    }
    return states;
}

space_point_t convertToSpacePoint(const Eigen::VectorXd& vec) 
{
    space_point_t point;
    for (int i = 0; i < vec.size(); ++i) {
        point[i] = vec[i];
    }
    return point;
}

bool attemptAddState(rrt_star_t& tree, const space_point_t& state) 
{
    rrt_star_node_t* x_nearest = static_cast<rrt_star_node_t*>(tree.get_metric()->single_query(state));

    trajectory_t traj(tree.get_state_space());
    tree.get_steer_function()(traj, x_nearest->point, state, tree.get_eta());

    if (tree.get_valid_check()(traj)) {
        std::shared_ptr<rrt_star_node_t> x_new = std::make_shared<rrt_star_node_t>();
        x_new->point = tree.get_state_space()->clone_point(traj.back());

        tree.add_to_tree(x_new, traj.back());
        const auto X_near = tree.get_near_nodes(x_new, tree.get_tree().num_vertices());
        rrt_star_node_t* x_min = x_nearest;
        double edge_cost = tree.connect_along_minimum_cost(x_min, x_new, X_near, traj);
        tree.add_edge(x_new, x_min, traj, edge_cost);
        tree.rewire_tree(X_near, x_new);
        tree.update_goal(x_new->get_index());

        return true;
    }
    return false;
}

space_point_t sampleAroundState(rrt_star_t& tree, const space_point_t& center, const double radius) 
{
    space_point_t sample = tree.get_state_space()->make_point();
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(-radius, radius);

    for (size_t i = 0; i < sample->size(); ++i) {
        double offset = distribution(generator);
        sample->at(i) = center->at(i) + offset;
    }
    return sample;
}

bool processState(rrt_star_t& tree, const Eigen::VectorXd& state, const double sampling_radius, const int num_samples, double& collision_distance) 
{
    space_point_t suggested_state = convertToSpacePoint(state);
    collision_distance = calculateCollisionDistance(tree, suggested_state);

    if (attemptAddState(tree, suggested_state)) {
        return true;
    } else {
        for (int i = 0; i < num_samples; ++i) {
            space_point_t sample = sampleAroundState(tree, suggested_state, sampling_radius);
            if (attemptAddState(tree, sample)) {
                return attemptAddState(tree, suggested_state);
            }
        }
    }
    return false;
}

double calculateCollisionDistance(const rrt_star_t& tree, const space_point_t& state) 
{
    environment_t environment;
    return environment.calculate_collision_distance(state);
}

void saveTreeToFile(const rrt_star_t& tree, const std::string& filename) 
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Cannot open file for writing." << std::endl;
        return;
    }
    tree.to_file(file);
}

int main(int argc, char* argv[])
{
    prx::param_loader params("executables/peg_in_hole.yaml", argc, argv);

    prx::simulation_step = params["simulation_step"].as<double>();
    prx::init_random(params["random_seed"].as<int>());

    prx::PairNameObstacles obstacles{ prx::load_obstacles(params["environment"].as<>()) };
    const std::vector<std::shared_ptr<prx::movable_object_t>> obstacle_list{ obstacles.second };
    const std::vector<std::string> obstacle_names{ obstacles.first };

    const std::string plant_name{ params["/plant/name"].as<>() };
    const std::string plant_path{ params["/plant/path"].as<>() };
    prx::system_ptr_t plant{ prx::system_factory_t::create_system(plant_name, plant_path) };
    prx_assert(plant != nullptr, "Plant is nullptr!");

    prx::world_model_t world_model({ plant }, { obstacle_list });
    world_model.create_context("context", { plant_name }, { obstacle_names });
    auto context = world_model.get_context("context");
    std::shared_ptr<prx::system_group_t> sys_group{ context.first };

    prx::space_t* ss{ sys_group->get_state_space() };
    prx::space_t* cs{ sys_group->get_control_space() };
    prx::space_t* ps{ sys_group->get_parameter_space() };

    prx::rrt_star_t rrt_star(params["/planner/name"].as<>());
    prx::rrt_star_specification_t rrt_star_spec(context.first, context.second);

    rrt_star_spec.distance_function = [&](const prx::space_point_t& x, const prx::space_point_t& y) 
    {
        double dist{ 0.0 };
        dist = (Vec(x).head(3) - Vec(y).head(3)).norm();
        const double q0_w{ x->at(3 + 0) };
        const double q0_x{ x->at(3 + 1) };
        const double q0_y{ x->at(3 + 2) };
        const double q0_z{ x->at(3 + 3) };
        const double q1_w{ y->at(3 + 0) };
        const double q1_x{ y->at(3 + 1) };
        const double q1_y{ y->at(3 + 2) };
        const double q1_z{ y->at(3 + 3) };
        const Eigen::Quaterniond q0(q0_w, q0_x, q0_y, q0_z);
        const Eigen::Quaterniond q1(q1_w, q1_x, q1_y, q1_z);
        dist += q0.angularDistance(q1);
        return dist;
    };

    rrt_star_spec.min_control_steps = params["plant/min_steps"].as<int>();
    rrt_star_spec.max_control_steps = params["/plant/max_steps"].as<int>();

    prx::rrt_star_query_t rrt_star_query(ss, cs);
    rrt_star_query.start_state = context.first->get_state_space()->make_point();
    rrt_star_query.goal_state = context.first->get_state_space()->make_point();

    rrt_star_spec.eta_min = params["/planner/eta_min"].as<double>();
    rrt_star_spec.eta_max = params["/planner/eta_max"].as<double>();

    const std::vector<double> ss_lower_bounds{ params["/plant/state_space/lower_bound"].as<std::vector<double>>() };
    const std::vector<double> ss_upper_bounds{ params["/plant/state_space/upper_bound"].as<std::vector<double>>() };

    const std::vector<double> cs_lower_bounds{ params["/plant/control_space/lower_bound"].as<std::vector<double>>() };
    const std::vector<double> cs_upper_bounds{ params["/plant/control_space/upper_bound"].as<std::vector<double>>() };

    const std::vector<double> ps_values{ params["/plant/parameter_space/values"].as<std::vector<double>>() };

    ss->set_bounds(ss_lower_bounds, ss_upper_bounds);
    cs->set_bounds(cs_lower_bounds, cs_upper_bounds);

    ps->copy_from(ps_values);

    ss->copy(rrt_star_query.start_state, params["/plant/start_state"].as<std::vector<double>>());
    ss->copy(rrt_star_query.goal_state, params["/plant/goal_state"].as<std::vector<double>>());

    rrt_star_query.goal_region_radius = params["/planner/goal_region_radius"].as<double>();

    rrt_star_query.goal_check = [&](prx::space_point_t pt)
    {
        const double dist_to_goal{ rrt_star_spec.distance_function(pt, rrt_star_query.goal_state) };
        return dist_to_goal < rrt_star_query.goal_region_radius;
    };
    const std::string out_dir{ params["/out/dir"].as<>() };
    const std::string file_prefix{ params["/out/file_prefix"].as<>() };

    rrt_star_query.get_visualization = params["visualize"].as<bool>();

    bool grow_tree = params["grow_tree"].as<bool>();
    bool query_tree = params["query_tree"].as<bool>();
    bool tree_to_files = params["tree_to_files"].as<bool>();

    std::string initial_tree_file = "pih_0007_tree.txt";
    rrt_star_t _tree;
    readTreeFromFile(initial_tree_file, _tree);
    auto suggestedStates = readSuggestedStates("suggested_states.txt");

    int successfulInsertions = 0;
    std::ofstream updatedFile("suggested_states_updated.txt");

    if (grow_tree) 
    {
        prx::condition_check_t checker(params["/planner/checker_type"].as<>(), params["/planner/checker_value"].as<int>());

        for (const auto& state : suggestedStates) 
        {
            double collision_distance;
            bool added = processState(_tree, state, 0.5, 100, collision_distance);
            updatedFile << state.transpose() << ", " << added << ", " << collision_distance << std::endl;
            if (added) 
            {
                successfulInsertions++;
            }    
        }
    }

    if (query_tree) 
    {
        std::string file_prefix = params["/out/file_prefix"].as<std::string>();
        std::string out_dir = params["/out/dir"].as<std::string>();
        _tree.from_files(file_prefix, out_dir);
        _tree.connect_goal();
    }
    _tree.fulfill_query();

    if (tree_to_files) 
    {
        std::string file_prefix = params["/out/file_prefix"].as<std::string>();
        std::string out_dir = params["/out/dir"].as<std::string>();
        _tree.to_files(file_prefix, out_dir);
    }

    double successPercentage = static_cast<double>(successfulInsertions) / static_cast<double>(suggestedStates.size()) * 100.0;
    std::cout << "Percentage of successful insertions: " << successPercentage << "%" << std::endl;

    saveTreeToFile(_tree, "pih_0007_updated.txt");

    three_js_group_t* vis_group = new three_js_group_t({ plant }, { obstacle_list });
    std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();

    vis_group->set_floor_plane(std::vector<double>({ 0, 0, -3 }), std::vector<double>({ 0.707, 0, 0, 0.707 }),
                               std::vector<double>({ 500, 500 }), "0xbbbbbb");
    vis_group->add_vis_infos(info_geometry_t::LINE, rrt_star_query.tree_visualization, body_name, ss);
    vis_group->add_detailed_vis_infos(info_geometry_t::FULL_LINE, rrt_star_query.solution_traj, body_name, ss);
    vis_group->add_vis_infos(info_geometry_t::SPHERE, { Vec(rrt_star_query.goal_state).head(3) }, "0xffff00",
                             rrt_star_query.goal_region_radius);
    vis_group->add_animation(rrt_star_query.solution_traj, ss, rrt_star_query.start_state);
    vis_group->output_html("updated_rrt_star_visualization.html");

    delete vis_group;

    std::cout << "End of program" << std::endl;
    return 0;
