#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <memory>  // For shared_ptr

// Include necessary headers
#include "../src/prx/planning/planners/rrt_star.hpp"
#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"

// Assuming these types and functions are part of the prx namespace
using prx::space_point_t;
using prx::trajectory_t;
using prx::rrt_star_t;

typedef std::shared_ptr<Node> NodePtr;
typedef std::vector<NodePtr> CloseNodes;


// Define a struct to represent a node in the RRT* tree
struct TreeNode 
{
    int parent_idx;
    int edge_idx;
    int node_idx;
    double x, y, z, qw, qx, qy, qz;
    // Add any other fields you need
};

// Updated tryAddState function
bool tryAddState(const std::vector<double>& state) 
{
    // Create a space_point_t from the provided state
    //space_point_t new_point = _state_space->make_point();
    prx::space_point_t new_point = _state_space->make_point();

    for (std::size_t i = 0; i < state.size(); ++i) 
    {
        new_point->at(i) = state[i];
    }

    // Create a trajectory with just the new state (as if it's randomly sampled)
    trajectory_t traj(_state_space);
    traj.copy_onto_back(new_point);

    // Collision check the trajectory
    if (!_valid_check(traj)) 
    {
        // State cannot be added due to collision

        // Sample uniformly around the suggested state for candidate future states
        const int num_samples = 10;  // You can adjust the number of samples as needed
        for (int i = 0; i < num_samples; ++i) 
	{
            // Generate a random state around the suggested state
            space_point_t candidate_point = _state_space->make_point();
            // You can customize how you sample around the state, e.g., random perturbation
            // Modify candidate_point accordingly.

            // Create a trajectory with just the candidate state
            trajectory_t candidate_traj(_state_space);
            candidate_traj.copy_onto_back(candidate_point);

            // Collision check the candidate trajectory
            if (_valid_check(candidate_traj)) 
	    {
                // Candidate state can be added to the tree
                NodePtr x_new;
                add_to_tree(x_new, candidate_point);

                // Get nearby nodes (similar to the original RRT* algorithm)
                const CloseNodes X_near
		{ 
			get_near_nodes(x_new, _tree.num_vertices()) 
		};

                // Find the minimum-cost path to connect
                const double edge_cost
		{ 
			connect_along_minimum_cost(x_min, x_new, X_near, candidate_traj) 
		};

                // Add the edge to the tree
                add_edge(x_new, x_min, candidate_traj, edge_cost);

                // Rewire the tree
                rewire_tree(X_near, x_new);

                // Update the goal based on the new state
                update_goal(x_new->get_index());

                return true;  // Successfully added a candidate state
            }
        }
        return false;  // All candidate states failed collision check
    }

    // Now, add this state to the tree as if it's a randomly sampled state
    NodePtr x_new;
    add_to_tree(x_new, new_point);

    // Get nearby nodes (similar to the original RRT* algorithm)
    const CloseNodes X_near
    { 
	    get_near_nodes(x_new, _tree.num_vertices()) 
    };

    // Find the minimum-cost path to connect
    const double edge_cost
    { 
	    connect_along_minimum_cost(x_min, x_new, X_near, traj) 
    };

    // Add the edge to the tree
    add_edge(x_new, x_min, traj, edge_cost);

    // Rewire the tree
    rewire_tree(X_near, x_new);

    // Update the goal based on the new state
    update_goal(x_new->get_index());

    return true;
}


int main() 
{
    // Initialize your RRT* planner and other necessary objects here
    rrt_star_t planner;  // Replace with your custom RRT* planner class
    planner.initialize();  // Initialize the planner

    // Declare these variables before their first use
    int states_added = 0;
    int states_not_added = 0;
    double total_collision_distance = 0.0;

    std::ifstream rrt_tree_file("pih_0001_tree.txt");
    if (!rrt_tree_file.is_open()) 
    {
        std::cerr << "Failed to open the RRT* tree file" << std::endl;
        return 1;
    }

    std::ifstream suggested_states_file("suggested_states.txt");
    if (!suggested_states_file.is_open()) 
    {
        std::cerr << "Failed to open the suggested states file" << std::endl;
        return 1;
    }

    std::vector<TreeNode> rrt_tree;

    // Read and parse the RRT* tree from the file
    while (!rrt_tree_file.eof()) 
    {
        TreeNode node;
        rrt_tree_file >> node.parent_idx >> node.edge_idx >> node.node_idx
                      >> node.x >> node.y >> node.z
                      >> node.qw >> node.qx >> node.qy >> node.qz;
        // Add any other fields you need to read

        rrt_tree.push_back(node);
    }

    rrt_tree_file.close();

    // Read and parse the suggested states
    std::vector<TreeNode> suggested_states;
    while (!suggested_states_file.eof()) 
    {
        TreeNode state;
        suggested_states_file >> state.parent_idx >> state.edge_idx >> state.node_idx
                               >> state.x >> state.y >> state.z
                               >> state.qw >> state.qx >> state.qy >> state.qz;
        // Add any other fields you need to read

        // Call your updated tryAddState function here
        bool added = tryAddState({state.x, state.y, state.z, state.qw, state.qx, state.qy, state.qz});

	
        if (added) 
	{
	    states_added++;
            // State was successfully added to the tree
            // You can perform any additional actions here if needed
        } 
	else 
	{
	    states_not_added++;
	    // Calculate collision distance and add it to the total
            double collision_distance = std::sqrt(state.x * state.x + state.y * state.y + state.z * state.z);
            total_collision_distance += collision_distance;

            // State was not added due to collision
            // You can handle collision states here
        }

    }

    suggested_states_file.close();

    // Calculate statistics
    int states_added = 0;
    int states_not_added = 0;
    double total_collision_distance = 0.0;

    // Iterate through suggested states and try to add them to the RRT* tree
    for (const TreeNode& state : suggested_states) 
    {
        if (tryAddState(state, rrt_tree)) 
	{
            states_added++;
        } 
	else 
	{
            states_not_added++;
            // Calculate collision distance and add it to the total
            double collision_distance = std::sqrt(state.x * state.x + state.y * state.y + state.z * state.z);
            total_collision_distance += collision_distance;
        }
    }

    // Calculate the success rate as a percentage
    double success_rate = (static_cast<double>(states_added) / suggested_states.size()) * 100.0;

    // Report statistics
    std::cout << "States successfully added: " << states_added << std::endl;
    std::cout << "States not added due to collision: " << states_not_added << std::endl;
    std::cout << "Average collision distance: " << (states_not_added > 0 ? total_collision_distance / states_not_added : 0.0) << std::endl;
    std::cout << "Success rate (%): " << success_rate << std::endl;

    // Save the updated RRT* tree to a new file
    std::ofstream updated_tree_file("pih_0001_tree_Update1.txt");
    if (!updated_tree_file.is_open()) 
    {
        std::cerr << "Failed to open the updated tree file" << std::endl;
        return 1;
    }

    for (const TreeNode& node : rrt_tree) 
    {
        updated_tree_file << node.parent_idx << " " << node.edge_idx << " " << node.node_idx << " "
                          << node.x << " " << node.y << " " << node.z << " "
                          << node.qw << " " << node.qx << " " << node.qy << " " << node.qz << std::endl;
        // Add any other fields you need to save
    }

    updated_tree_file.close();

    // Save statistics to a file
    std::ofstream stats_file("rrt_extras_stats.txt");
    if (!stats_file.is_open()) {
        std::cerr << "Failed to open the statistics file" << std::endl;
        return 1;
    }

    stats_file << "States successfully added: " << states_added << std::endl;
    stats_file << "States not added due to collision: " << states_not_added << std::endl;
    stats_file << "Average collision distance: " << (states_not_added > 0 ? total_collision_distance / states_not_added : 0.0) << std::endl;
    stats_file << "Success rate (%): " << success_rate << std::endl;

    stats_file.close();

    // Visualization
    prx::three_js_group_t* vis_group = new prx::three_js_group_t({ plant }, { obstacle_list });

    //std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();
    std::string body_name = params["/plant/name"].as<std::string>() + "/" + params["/plant/vis_body"].as<std::string>();


    vis_group->set_floor_plane(
		    		std::vector<double>({ 0, 0, -3 }), 
		    		std::vector<double>({ 0.707, 0, 0, 0.707 }),
                               	std::vector<double>({ 500, 500 }), "0xbbbbbb");
    vis_group->add_vis_infos(prx::info_geometry_t::LINE, rrt_star.tree_visualization, body_name, ss);
    vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, rrt_star_query.solution_traj, body_name, ss);
    vis_group->add_vis_infos(prx::info_geometry_t::SPHERE, { Vec(rrt_star_query.goal_state).head(3) }, "0xffff00", rrt_star_query.goal_region_radius);
    vis_group->add_animation(rrt_star_query.solution_traj, ss, rrt_star_query.start_state);
    vis_group->output_html("peg_in_hole_rrt_star.html");

    delete vis_group;

    std::cout << "End of program" << std::endl;

    return 0;
}

