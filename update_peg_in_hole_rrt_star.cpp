#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>  // Include for Eigen's Quaternion support
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
#include "prx/utilities/general/prx_assert.hpp"  // Replace with the actual header file name

//#include "prx/simulation/state_spaces/state_space.hpp"
//#include "prx/simulation/environment.hpp"  // Assuming environment-related functionalities

using namespace prx;


// Add print statements in the main function or other relevant places to track the program's progress and identify potential issues

// Function to read suggested states from file and convert them to space_point_t
std::vector<space_point_t> readSuggestedStates(const std::string& filename, space_t* state_space) 
{
    std::vector<space_point_t> states;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Cannot open states file." << std::endl;
        return states;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        Eigen::VectorXd eigenState(7); // Assuming each state has 7 values
        for (int i = 0; i < 7; ++i) {
            if (!(iss >> eigenState[i])) {
                std::cerr << "State parsing error." << std::endl;
                break;
            }
            if (i < 6) iss.ignore(); // Skip the comma
        }

        // Convert Eigen::VectorXd to space_point_t
        space_point_t state = state_space->make_point();
        for (int i = 0; i < eigenState.size(); ++i) {
            state->at(i) = eigenState[i];
        }
        
        states.push_back(state);
    }
    return states;
}


space_point_t convertToSpacePoint(const Eigen::VectorXd& vec, const prx::space_t* space)  
{
    space_point_t point = std::make_shared<prx::space_snapshot_t>(space, vec.size());
    for (int i = 0; i < vec.size(); ++i) 
    {
        (*point)[i] = vec[i]; 
    }
    return point;
}

Eigen::VectorXd convertToEigenVector(const space_point_t& point)
{
    Eigen::VectorXd vec(point->size());
    for (int i = 0; i < point->size(); ++i)
    {
        vec[i] = (*point)[i];
    }
    return vec;
}


// Function to attempt adding a state to the RRT* tree
bool attemptAddState(rrt_star_t& tree, const space_point_t& state) 
{
    // Assuming get_metric() is a public member function of rrt_star_t
    rrt_star_node_t* x_nearest = static_cast<rrt_star_node_t*>(tree._metric->single_query(state));
    // print "Nearest"
    std::cout << "Nearest" << std::endl;
    // print x_nearest
    std::cout << "x_nearest: " << tree.get_state_space()->print_point(x_nearest->point) << std::endl;   
    // print "Traj"
    std::cout << "Traj" << std::endl;
    prx::trajectory_t traj(tree.get_state_space());
    
    // Assuming steer() is a public member function of rrt_star_t
    // print "Steer"
    std::cout << "Steer" << std::endl;
    tree._steer_function(traj, x_nearest->point, state, tree._eta);

    if (tree._valid_check(traj)) {
        std::shared_ptr<rrt_star_node_t> x_new = std::make_shared<rrt_star_node_t>();
        // print "Create new node"
        std::cout << "Create new node" << std::endl;
        x_new->point = tree.get_state_space()->clone_point(traj.back());
        // print "Add to tree"
        std::cout << "Add to tree" << std::endl;
        tree.add_to_tree(x_new, traj.back());
        // print "Get mear nodes"
        std::cout << "Get near nodes" << std::endl;
        const auto X_near = tree.get_near_nodes(x_new, tree._tree.num_vertices());
        // print "Get near nodes"
        std::cout << "Get near nodes" << std::endl;
        //std::cout << "X_near: " << tree.get_state_space()->print_point(X_near->point) << std::endl;
        // print X_near
        // print "Get x_min"
        std::cout << "Get x_min" << std::endl;
        rrt_star_node_t* x_min = x_nearest;
        // print "Connect along minimum cost"
        std::cout << "Connect along minimum cost" << std::endl;
        double edge_cost = tree.connect_along_minimum_cost(x_min, x_new, X_near, traj);
        // print "Add edge"
        std::cout << "Add edge" << std::endl;
        tree.add_edge(x_new, x_min, traj, edge_cost);
        // print "Rewire tree"
        std::cout << "Rewire tree" << std::endl;
        tree.rewire_tree(X_near, x_new);
        // print "Update goal"
        std::cout << "Update goal" << std::endl;
        tree.update_goal(x_new->get_index());

        bool flag = true;
        // print flag
        std::cout << "Flag: " << flag << std::endl;
        return flag;
    }
    bool flag = false;
    std::cout << "Flag: " << flag << std::endl;
    return flag;
}


space_point_t sampleAroundState(rrt_star_t& tree, const space_point_t& center, const double radius) 
{
    space_point_t sample = tree.get_state_space()->make_point();

    std::default_random_engine generator(std::random_device{}());
    std::uniform_real_distribution<double> pos_distribution(-radius, radius);
    std::uniform_real_distribution<double> axis_distribution(-1.0, 1.0);
    std::uniform_real_distribution<double> angle_distribution(-.25 * M_PI, .25 * M_PI);

    // Sample position (assuming the first 3 elements of the state are position)
    for (size_t i = 0; i < 3; ++i) 
    {
        double offset = pos_distribution(generator);
        sample->at(i) = center->at(i) + offset;
    }
    
    // Multiply x, y, z by 1000 for scaling
    //sample->at(0) *= 1000;
    //sample->at(1) *= 1000;
    //sample->at(2) *= 1000;

    // Original quaternion (assuming the next 4 elements of the state are quaternion components)
    Eigen::Quaterniond q_orig(center->at(3), center->at(4), center->at(5), center->at(6));
    
    // Sample a random rotation axis for quaternion perturbation
    Eigen::Vector3d axis;
    do 
    {
        axis = Eigen::Vector3d(axis_distribution(generator), axis_distribution(generator), axis_distribution(generator));
    } while (axis.norm() < 0.001); // Ensure the axis is not too small
    axis.normalize(); // Normalize to get a valid rotation axis

    // Sample a random rotation angle
    double angle = angle_distribution(generator);

    // Construct a quaternion from the axis and angle
    Eigen::Quaterniond q_rand(Eigen::AngleAxisd(angle, axis));

    // Apply the quaternion difference
    Eigen::Quaterniond q_new = q_orig * q_rand;
    q_new.normalize(); // Normalize to ensure it's a valid rotation

    sample->at(3) = q_new.w();
    sample->at(4) = q_new.x();
    sample->at(5) = q_new.y();
    sample->at(6) = q_new.z();

    // Debug: Print the sampled state
    std::cout << "Sampled State: " << tree.get_state_space()->print_point(sample) << std::endl;

    return sample;
}


// This is wrong !!!
double calculateCollisionDistance(const rrt_star_t& tree, const space_point_t& state)
{
    // Find the nearest node in the tree
    rrt_star_node_t* nearestNode = static_cast<rrt_star_node_t*>(tree._metric->single_query(state));
    // Calculate the collision distance using the distance function
    double collisionDistance = tree._distance_function(nearestNode->point, state);
    return collisionDistance;
}

bool processState(rrt_star_t& tree, const Eigen::VectorXd& state, const double sampling_radius, const int num_samples) {
    space_point_t suggested_state = convertToSpacePoint(state, tree.get_state_space());
    bool added = false;
    while (!added) 
    {
        std::cout << "Attempting to add suggested state." << std::endl;
        added = attemptAddState(tree, suggested_state);
        if (!added) 
        {
            std::cout << "Suggested state not added, attempting to add sampled states around suggested state." << std::endl;
            for (int i = 0; i < num_samples; ++i) 
            {
                space_point_t sample = sampleAroundState(tree, suggested_state, sampling_radius);
                std::cout << "Sampled state I attempt to add: " << tree.get_state_space()->print_point(sample) << std::endl;
                if (attemptAddState(tree, sample)) 
                {
                    std::cout << "Added sampled state around suggested state." << std::endl;
                    // Check again if suggested state can now be added
                    added = attemptAddState(tree, suggested_state);
                    if (added) 
                    {
                        std::cout << "Successfully added suggested state after adding a sampled state." << std::endl;
                        break;
                    }
                }
            }
        }
    }

    std::cout << "Suggested state successfully added to the tree." << std::endl;
    return true;
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
        // const Eigen::Matrix3d R0{ q0.toRotationMatrix() };
        // const Eigen::Matrix3d R1{ q1.toRotationMatrix() };
        // dist += std::acos(((R1.transpose() * R0).trace() - 1) / 2.0);
        return dist;
    };

    rrt_star_spec.min_control_steps = params["plant/min_steps"].as<int>();
    rrt_star_spec.max_control_steps = params["/plant/max_steps"].as<int>();

    prx::rrt_star_query_t rrt_star_query(ss, cs);
    rrt_star_query.start_state = context.first->get_state_space()->make_point();
    rrt_star_query.goal_state = context.first->get_state_space()->make_point();

    rrt_star_spec.eta_min = params["/planner/eta_min"].as<double>();
    rrt_star_spec.eta_max = params["/planner/eta_max"].as<double>();

    // Inside the main function
    const std::vector<double> ss_lower_bounds{ params["/plant/state_space/lower_bound"].as<std::vector<double>>() };
    const std::vector<double> ss_upper_bounds{ params["/plant/state_space/upper_bound"].as<std::vector<double>>() };
    const std::vector<double> cs_lower_bounds{ params["/plant/control_space/lower_bound"].as<std::vector<double>>() };
    const std::vector<double> cs_upper_bounds{ params["/plant/control_space/upper_bound"].as<std::vector<double>>() };

    std::cout << "ss_lower_bounds: ";
    for (const double& value : ss_lower_bounds) 
    {
        std::cout << value << " ";
    }
    std::cout << std::endl;

    std::cout << "ss_upper_bounds: ";
    for (const double& value : ss_upper_bounds) 
    {
        std::cout << value << " ";
    }
    std::cout << std::endl;

    std::cout << "cs_lower_bounds: ";
    for (const double& value : cs_lower_bounds) 
    {
        std::cout << value << " ";
    }
    std::cout << std::endl;

    std::cout << "cs_upper_bounds: ";
    for (const double& value : cs_upper_bounds) 
    {
        std::cout << value << " ";
    }
    std::cout << std::endl;

    const std::vector<double> ps_values{ params["/plant/parameter_space/values"].as<std::vector<double>>() };

    ss->set_bounds(ss_lower_bounds, ss_upper_bounds);
    cs->set_bounds(cs_lower_bounds, cs_upper_bounds);

    ps->copy_from(ps_values);

    ss->copy(rrt_star_query.start_state, params["/plant/start_state"].as<std::vector<double>>());
    ss->copy(rrt_star_query.goal_state, params["/plant/goal_state"].as<std::vector<double>>());

    rrt_star_query.goal_region_radius = params["/planner/goal_region_radius"].as<double>();

    // Alternatively, change the goal_check function
    rrt_star_query.goal_check = [&](prx::space_point_t pt)  // no-lint
    {
        const double dist_to_goal{ rrt_star_spec.distance_function(pt, rrt_star_query.goal_state) };
        return dist_to_goal < rrt_star_query.goal_region_radius;
    };

    //const std::string out_dir{ params["/out/dir"].as<>() };
    const std::string file_prefix{ params["/out/file_prefix"].as<>() };
    // put_dir_updated shpuld be /common/home/im316/RL4Insertion_code/ML4KP-devel/out/peg_in_h_updated.txt/pih_0007_trajectories_updated.txt. Set it explicitly
    //std::string out_dir_updated= out_dir + file_prefix + "updated.txt";
    std::string out_dir = "/common/home/im316/RL4Insertion_code/ML4KP-devel/out/peg_in_hole/pih_0001_tree.txt";
    std::string out_dir_updated = "/common/home/im316/RL4Insertion_code/ML4KP-devel/out/peg_in_hole/pih_0001_tree_updated.txt";
    std::string out_dir_updated_traj = "/common/home/im316/RL4Insertion_code/ML4KP-devel/out/peg_in_hole/pih_0001_trajectories_updated.txt";
    std::cout << "out_dir: " << out_dir << std::endl;
    std::cout << "out_dir_updated: " << out_dir_updated << std::endl;

    rrt_star_query.get_visualization = params["visualize"].as<bool>();

    rrt_star.link_and_setup_spec(&rrt_star_spec);
    rrt_star.preprocess();
    rrt_star.link_and_setup_query(&rrt_star_query);

    // Grow tree or query tree based on command-line arguments
    bool grow_tree = params["grow_tree"].as<bool>();
    bool query_tree = params["query_tree"].as<bool>();
    bool tree_to_files = params["tree_to_files"].as<bool>();

    int successfulInsertions = 0;
    std::ofstream updatedFile("/common/home/im316/RL4Insertion_code/ML4KP-devel/suggested_states_updated.txt");
   
    std::cout << "Before loading tree from files" << std::endl;
    rrt_star.from_files(file_prefix, out_dir);
    std::cout << "After loading tree from files" << std::endl;

    auto suggestedStates = readSuggestedStates("/common/home/im316/RL4Insertion_code/ML4KP-devel/suggested_states.txt", rrt_star.get_state_space());
    
    // Load tree or grow tree based on command-line arguments
    if (grow_tree) 
    {
        prx::condition_check_t checker(params["/planner/checker_type"].as<>(), params["/planner/checker_value"].as<int>());
        
        for (const auto& eigenState : suggestedStates) 
        {
            Eigen::VectorXd eigenVector = convertToEigenVector(eigenState);
            bool added = processState(rrt_star, eigenVector, 20, 100);            
         
            // print "True" or "False" to the file
            updatedFile << (added ? "True" : "False") << std::endl;
            // print suggested state
            std::cout << "Suggested state: " << rrt_star.get_state_space()->print_point(eigenState) << std::endl; 
            if (added) 
            {
                successfulInsertions++;
            }    
        }
        //rrt_star.resolve_query(&checker);
    }
    
    if (query_tree) 
    {
        rrt_star.from_files(file_prefix, out_dir_updated);
        // Load the tree from a file
        rrt_star.connect_goal();
    }
    rrt_star.fulfill_query();
    
    if (tree_to_files) 
    {
        rrt_star.to_files(file_prefix, out_dir_updated);
    }

    double successPercentage = static_cast<double>(successfulInsertions) / static_cast<double>(suggestedStates.size()) * 100.0;
    std::cout << "Percentage of successful insertions: " << successPercentage << "%" << std::endl;

    // Visualization
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

}
