#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <iomanip>
#include <Eigen/Dense>

#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/rrt_star.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"

using Transform = Eigen::Transform<double, 3, Eigen::Isometry>;
using prx::split;
using prx::utilities::convert_to;


// Define a struct for the pose with additional indices
struct TreeNode {
    int parent_idx, edge_idx, node_idx;
    Eigen::Vector3d pos;
    Eigen::Quaterniond quat;
};

// Function to perform the ml4kp to isaac transformation
void transformPose(Eigen::Vector3d& pos, Eigen::Quaterniond& quat) {
    // Place your transformation logic here
    // For the purpose of demonstration, I'll use an identity transformation
    // Replace this with your actual transformation logic

    // Example: Identity transformation (no change to pos and quat)

    Transform ml4kpWorld_w{ Transform::Identity() };
    Transform pegBottom_w{ Transform::Identity() };
    Transform pegCenter_w{ Transform::Identity() };
    Transform pegBottom_ml4kp{ Transform::Identity() };
    Transform pegCenter_ml4kp{ Transform::Identity() };
    Transform pegCenter_pegBottom{ Transform::Identity() };

    pegCenter_pegBottom.translation() = Eigen::Vector3d(0, 0, 25);

    
    //ml4kpWorld_w.translation() = pos_socket;
    // ml4kpWorld_w.linear() = quat_socket.toRotationMatrix();
    ml4kpWorld_w.translation() = Eigen::Vector3d(130.0, 0.0, 1040.0);

    pegCenter_ml4kp.translation() = pos;
    pegCenter_ml4kp.linear() = quat.toRotationMatrix();

    pegBottom_ml4kp = pegCenter_ml4kp * pegCenter_pegBottom.inverse();
    pegBottom_w = ml4kpWorld_w * pegBottom_ml4kp;
    pegCenter_w = pegBottom_w * pegCenter_pegBottom;

    // Update the position and quaternion with the transformed values
    pos = pegBottom_w.translation();
    // Adjust z by subtracting 1 and then scale the position by dividing each component by 1000
    pos.z() -= 1.0;
    // Scale the position by dividing each component by 1000
    pos /= 1000.0;
    quat = Eigen::Quaterniond(pegBottom_w.rotation());
}

int main() 
{

    std::string inputFilename = "/common/users/im316/RRT_Star/rect_16_mm/pih_rectangular_16mm_RightTight_10mins_0002_tree.txt";
    std::string outputFilename = "/common/users/im316/RRT_Star/rect_16_mm/pih_rectangular_16mm_RightTight_10mins_0002_tree_isaac.txt";
    
    //std::string inputFilename = "/common/users/im316/RRT_Star/rect_16_mm/pih_rectangular_16mm_Knockin_Aravind_tree.txt";
    //std::string outputFilename = "/common/users/im316/RRT_Star/rect_16_mm/pih_rectangular_16mm_Knockin_Aravind_tree_isaac.txt";

    //std::string inputFilename = "/common/home/im316/RL4Insertion_code/ML4KP-devel/out/peg_in_hole/pih_Brief_Really_Biased_rectangular_16mm_0001_tree.txt";  // Replace with your input file path
    //std::string outputFilename = "/common/home/im316/RL4Insertion_code/ML4KP-devel/out/peg_in_hole/pih_Brief_Really_Biased_rectangular_16mm_0001_tree_isaac.txt"; // Replace with your output file path

    //std::string inputFilename = "/common/home/im316/RL4Insertion_code/CORL/algorithms/interpolated_poses_quat_ml4kp.txt";  // Replace with your input file path
    //std::string outputFilename = "/common/home/im316/RL4Insertion_code/CORL/algorithms/interpolated_poses_quat_isaac_2.txt"; // Replace with your output file path

    std::string headerLine;
    
    std::ifstream inputFile(inputFilename);
    std::ofstream outputFile(outputFilename);

    if (!inputFile.is_open()) {
        std::cerr << "Unable to open input file\n";
        return 1;
    }

    if (!outputFile.is_open()) {
        std::cerr << "Unable to open output file\n";
        return 1;
    }

    std::string line;
    std::vector<TreeNode> treeNodes;
    int lineCount = 0;

     // Read the header line from the input file
    if (!std::getline(inputFile, headerLine)) {
        std::cerr << "Unable to read the header line from the input file\n";
        return 1;
    }

    // Write the header line to the output file
    outputFile << headerLine << "\n";

    while (std::getline(inputFile, line)) {
        std::istringstream iss(line);
        TreeNode node;

        // Read indices and pose data
        if (iss >> node.parent_idx >> node.edge_idx >> node.node_idx 
            >> node.pos[0] >> node.pos[1] >> node.pos[2] 
            >> node.quat.w() >> node.quat.x() >> node.quat.y() >> node.quat.z()) {
            
	     // Print the first 5 lines read
            if (++lineCount <= 5) {
                std::cout << "Read: " << node.parent_idx << " " << node.edge_idx << " " << node.node_idx << " "
                          << node.pos[0] << " " << node.pos[1] << " " << node.pos[2] << " "
                          << node.quat.w() << " " << node.quat.x() << " " << node.quat.y() << " " << node.quat.z() << "\n";
            }
	
            transformPose(node.pos, node.quat);  // Transform the pose
            treeNodes.push_back(node);
            
        }
    }

    // Print the first 5 transformed lines before saving
    std::cout << "\nFirst 5 transformed lines to be saved:\n";
    for (int i = 0; i < 5 && i < treeNodes.size(); ++i) {
        const auto& node = treeNodes[i];
        std::cout << "Transformed: " << node.parent_idx << " " << node.edge_idx << " " << node.node_idx << " "
                  << node.pos[0] << " " << node.pos[1] << " " << node.pos[2] << " "
                  << node.quat.w() << " " << node.quat.x() << " " << node.quat.y() << " " << node.quat.z() << "\n";
    }


    // Write transformed data to output file
    for (const auto& node : treeNodes) {
        outputFile << node.parent_idx << " " << node.edge_idx << " " << node.node_idx << " "
                   << std::fixed << std::setprecision(5)
                   << node.pos[0] << " " << node.pos[1] << " " << node.pos[2] << " "
                   << node.quat.w() << " " << node.quat.x() << " " << node.quat.y() << " " << node.quat.z() << "\n";
    }

    outputFile.close();
    inputFile.close();

    std::cout << "\nData has been read from " << inputFilename << ", transformed, and written to " << outputFilename << "\n";

    return 0;
}

