#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <chrono>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/rrt_star.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"

using namespace std::chrono_literals;
using Transform = Eigen::Transform<double, 3, Eigen::Isometry>;
using prx::split;
using prx::utilities::convert_to;

using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Matrix3d;

struct Pose
{
    Vector3d position;
    Quaterniond orientation;
};

void printPose(const std::string& message, const Vector3d& pos, const Quaterniond& quat) {
    std::cout << message
              << pos.x() << ", " << pos.y() << ", " << pos.z() << ", "
              << quat.w() << ", " << quat.x() << ", " << quat.y() << ", " << quat.z() << std::endl;
}

void transformPose(Vector3d& pos, Quaterniond& quat)
{
    // Transformation logic as provided
    Transform ml4kpWorld_w = Transform::Identity();
    Transform pegBottom_w = Transform::Identity();
    Transform pegCenter_w = Transform::Identity();
    Transform pegBottom_ml4kp = Transform::Identity();
    Transform pegCenter_ml4kp = Transform::Identity();
    Transform pegCenter_pegBottom = Transform::Identity();

    pos.z() = pos.z() + 1.0;
    pos = pos * 1000.0;
    printPose("Scaled position: ", pos, quat);
    //std::this_thread::sleep_for(1s);

    pegCenter_pegBottom.translation() = Vector3d(0, 0, 25);
    ml4kpWorld_w.translation() = Vector3d(130.0, 0.0, 1040.0);

    pegBottom_w.translation() = pos;
    pegBottom_w.linear() = quat.toRotationMatrix();
    //printPose("Peg bottom world pose: ", pegBottom_w.translation(), Quaterniond(pegBottom_w.rotation()));
    //std::this_thread::sleep_for(1s);

    pegBottom_ml4kp = ml4kpWorld_w.inverse() * pegBottom_w;
    //pegBottom_ml4kp = pegBottom_w * ml4kpWorld_w.inverse();
    printPose("Peg bottom ml4kp pose (mm): ", pegBottom_ml4kp.translation(), Quaterniond(pegBottom_ml4kp.rotation()));
    
    pegCenter_ml4kp = pegBottom_ml4kp * pegCenter_pegBottom;
    //pegCenter_ml4kp = pegCenter_pegBottom * pegBottom_ml4kp;


    pos = pegCenter_ml4kp.translation();
    quat = Quaterniond(pegCenter_ml4kp.rotation());
    //printPose("Transformed Pose: ", pos, quat);
    //std::this_thread::sleep_for(1s);
}

int main() {
    //std::ifstream inputFile("/common/home/im316/RL4Insertion_code/CORL/algorithms/interpolated_poses_quat.txt");
    //std::ofstream outputFile("/common/home/im316/RL4Insertion_code/CORL/algorithms/interpolated_poses_quat_ml4kp.txt");

    std::ifstream inputFile("/common/home/im316/RL4Insertion_code/CORL/algorithms/learned_RRT_Star_rectangular_16mm_0001_Deep_Quat_Old_0.txt");
    std::ofstream outputFile("/common/home/im316/RL4Insertion_code/CORL/algorithms/learned_RRT_Star_rectangular_16mm_0001_Deep_Quat_Old_0_ml4kp.txt");
    
    //std::ifstream inputFile("/common/home/im316/RL4Insertion_code/ML4KP-devel/out/peg_in_hole/replan_states_rectangular_16mm_2.txt");
    //std::ofstream outputFile("/common/home/im316/RL4Insertion_code/ML4KP-devel/out/peg_in_hole/replan_states_rectangular_16mm_2_transformed.txt");

    
    //std::ifstream inputFile("toy_isaac.txt");
    //std::ofstream outputFile("toy_ml4kp.txt");

    std::string line;
    int count = 0;

    if (!inputFile.is_open()) {
        std::cerr << "Error opening file." << std::endl;
        return 1;
    }

    while (std::getline(inputFile, line)) {
        std::istringstream iss(line);
        Pose pose;
        char comma;

        if (!(iss >> pose.position.x() >> comma >> pose.position.y() >> comma >> pose.position.z() >> comma
                  >> pose.orientation.w() >> comma >> pose.orientation.x() >> comma >> pose.orientation.y() >> comma >> pose.orientation.z())) 
	{
            std::cerr << "Error reading line." << std::endl;
            break;
        }

        // Print the first 5 original poses
        if (count < 5) {
            printPose("Original Pose " + std::to_string(count + 1) + ": ", pose.position, pose.orientation);
        }

        // Apply the transformation
        transformPose(pose.position, pose.orientation);

        // Write to the output file
        outputFile << pose.position.x() << ", " << pose.position.y() << ", " << pose.position.z() << ", "
                   << pose.orientation.w() << ", " << pose.orientation.x() << ", " << pose.orientation.y() << ", " << pose.orientation.z() << std::endl;

        ++count;
    }

    inputFile.close();
    outputFile.close();
    return 0;
}

