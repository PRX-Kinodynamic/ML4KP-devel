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
        std::make_shared<prx::mujoco_simulator_t>(params["scene_xml_path"].as<std::string>());
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

    // mj_kinematics(sim->m, sim->d);
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

    auto end_effector_inds = get_body_qpos_indices(ee_sim->m, "hand");

    std::vector<double> q{1, 0, 1, 1, 0, 0, 0};

    auto pose = forward_kinematics(ee_sim->m, ee_sim->d, end_effector_inds, "left_finger", q);

    for (auto a : pose){
        std::cout << "hand " << a << std::endl;
    }

    std::cout << "----------------" << std::endl;

    for(int i = 0; i < ee_sim->m->nbody; i++){
        std::cout << mj_id2name(ee_sim->m, mjOBJ_BODY, i) << std::endl;
    }

    for (auto pair : ignored_pairs){
        std::cout << pair.first << ", " << pair.second << std::endl;
    }

    std::cout << "End of program!" << std::endl;
}