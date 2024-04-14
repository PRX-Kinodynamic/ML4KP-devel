#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <iomanip>
#include <algorithm>
#include <Eigen/Dense>

using Transform = Eigen::Transform<double, 3, Eigen::Isometry>;

// Define a struct for the pose
struct TreeNode {
    Eigen::Vector3d pos;
    Eigen::Quaterniond quat;
};

// Function to perform the transformation
void transformPose(Eigen::Vector3d& pos, Eigen::Quaterniond& quat) {
    Transform ml4kpWorld_w{ Transform::Identity() };
    Transform pegBottom_w{ Transform::Identity() };
    Transform pegCenter_w{ Transform::Identity() };
    Transform pegBottom_ml4kp{ Transform::Identity() };
    Transform pegCenter_ml4kp{ Transform::Identity() };
    Transform pegCenter_pegBottom{ Transform::Identity() };

    pegCenter_pegBottom.translation() = Eigen::Vector3d(0, 0, 25);
    ml4kpWorld_w.translation() = Eigen::Vector3d(130.0, 0.0, 1040.0);

    pegCenter_ml4kp.translation() = pos;
    pegCenter_ml4kp.linear() = quat.toRotationMatrix();

    pegBottom_ml4kp = pegCenter_ml4kp * pegCenter_pegBottom.inverse();
    pegBottom_w = ml4kpWorld_w * pegBottom_ml4kp;
    pegCenter_w = pegBottom_w * pegCenter_pegBottom;

    pos = pegBottom_w.translation();
    pos /= 1000.0;
    quat = Eigen::Quaterniond(pegBottom_w.rotation());
}

int main() 
{
    std::string inputFilename = "/common/home/im316/RL4Insertion_code/CORL/algorithms/interpolated_poses_quat_ml4kp.txt";  // Replace with your input file path
    std::string outputFilename = "/common/home/im316/RL4Insertion_code/CORL/algorithms/interpolated_poses_quat_isaac_2.txt"; // Replace with your output file path

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

    while (std::getline(inputFile, line)) {
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream iss(line);
        TreeNode node;

        if (iss >> node.pos[0] >> node.pos[1] >> node.pos[2]
            >> node.quat.w() >> node.quat.x() >> node.quat.y() >> node.quat.z()) {
            transformPose(node.pos, node.quat);
            treeNodes.push_back(node);
        }
    }

    for (const auto& node : treeNodes) {
        outputFile << std::fixed << std::setprecision(5)
                   << node.pos[0] << " " << node.pos[1] << " " << node.pos[2] << " "
                   << node.quat.w() << " " << node.quat.x() << " " << node.quat.y() << " " << node.quat.z() << "\n";
    }

    outputFile.close();
    inputFile.close();

    std::cout << "Data has been read from " << inputFilename << ", transformed, and written to " << outputFilename << "\n";
    return 0;
}

