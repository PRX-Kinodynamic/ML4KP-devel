// #ifndef TORCH_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/mujoco/mj_simulator.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/mujoco/mj_utils.hpp"
#include "prx/utilities/heuristics/pose_utils.hpp"
#include "prx/utilities/heuristics/roadmap.hpp"

#include <boost/filesystem.hpp>
using namespace boost::filesystem;

using namespace prx;

int main(int argc, char* argv[])
{
    param_loader params;
    if (argc < 2)
    {
        params = param_loader("examples/JIST/heuristic_test.yaml");
    }
    else
    {
        params = param_loader(argv[1]);
    }
    init_random(params["random_seed"].as<int>());

    std::vector<std::pair<std::string, std::string>> ignored_pairs = params["ignored_pairs"].as<std::vector<std::pair<std::string, std::string>>>();

    std::shared_ptr<prx::mujoco_simulator_t> sim =
        std::make_shared<prx::mujoco_simulator_t>(params["scene_xml_path"].as<std::string>(), false);
    sim->init_simulator();

    sim->add_pair(ignored_pairs);

    auto context = sim->get_context("mujoco");
    auto ss = context.first->get_state_space();
    auto cs = context.first->get_control_space();

    std::shared_ptr<prx::mujoco_simulator_t> ee_sim =
        std::make_shared<prx::mujoco_simulator_t>(params["heuristic_xml_path"].as<std::string>());
    ee_sim->init_simulator();

    auto ee_context = ee_sim->get_context("mujoco");
    auto ee_ss = ee_context.first->get_state_space();
    auto ee_cs = ee_context.first->get_control_space();

    for (int i = 0; i < 100; i++)
    {
        sim->step_simulation();
        // ee_sim->step_simulation();
    }

    std::vector<std::string> joint_names = params["joint_names"].as<std::vector<std::string>>();
    auto arm_qpos_inds = get_qpos_indices(sim->m, mjOBJ_JOINT, joint_names);

    std::string hand = params["forward_name"].as<std::string>();
    std::vector<std::string> ee_names = params["end_effector"].as<std::vector<std::string>>();

    distance_function_t distance_function = [&](const space_point_t& a, const space_point_t& b) {
        double dist = 0;
        for (auto end_effector_body : ee_names){
        auto pose_a = forward_kinematics(sim->m, sim->d, arm_qpos_inds, end_effector_body, a);
        auto pose_b = forward_kinematics(sim->m, sim->d, arm_qpos_inds, end_effector_body, b);

        double euclidean = 0;
        for (int i = 0; i < 3; i++){
            euclidean += std::pow(pose_a[i] - pose_b[i], 2.0);
        }
        euclidean = std::sqrt(euclidean);

        dist += euclidean;
        }

        return dist;
    };

    auto qpos_inds = get_qpos_indices(sim->m, mjOBJ_JOINT, joint_names);

    std::string end_effector = params["forward_name"].as<std::string>();

    std::vector<double> q = params["goal_config"].as<std::vector<double>>();
    auto grasp_cart_quat = forward_kinematics(sim->m, sim->d, qpos_inds, end_effector, q);

    vector_t grasp_pos{grasp_cart_quat[0], grasp_cart_quat[1], grasp_cart_quat[2]};
    quaternion_t grasp_ori{grasp_cart_quat[3], grasp_cart_quat[4], grasp_cart_quat[5], grasp_cart_quat[6]};

    Eigen::Matrix4d grasp_matrix;
    grasp_matrix.setIdentity();

    grasp_matrix.block<3, 3>(0, 0) = grasp_ori.normalized().toRotationMatrix();
        
    grasp_matrix.block<3, 1>(0, 3) = grasp_pos;

    vector_t pregrasp_pos{0, 0, -.05};

    Eigen::Matrix4d approach_matrix;
    approach_matrix.setIdentity();

    approach_matrix.block<3, 1>(0, 3) = pregrasp_pos;

    std::cout << grasp_matrix << std::endl;

    std::cout << approach_matrix << std::endl;

    std::cout << grasp_matrix * approach_matrix << std::endl;

    vector_t approach_pos = (grasp_matrix * approach_matrix).block<3, 1>(0, 3);

    sim->set_record_video(true);
    sim->set_video_name(params["video_name"].as<std::string>());

    auto hand_inds = get_body_qpos_indices(ee_sim->m, hand);
    std::vector<double> pregrasp{grasp_cart_quat};

    for(int i = 0; i < approach_pos.size(); i++){
        pregrasp[i] = approach_pos[i];
    }

    /*
    for(int i = 0; i < 10000; i++){
        for(int i = 0; i < pregrasp.size(); i++){
            ee_sim->d->qpos[hand_inds[i]] = pregrasp[i];
        }
        sim->step_simulation();
        ee_sim->step_simulation(1);
    }
    */

    for(auto ee : ee_names){
        auto x = forward_kinematics(ee_sim->m, ee_sim->d, hand_inds, ee, pregrasp);
        for(auto a : x){
            std::cout << a << ", ";
        }
        std::cout << std::endl;
    }

}