// #ifndef TORCH_NOT_BUILT
#include "prx/utilities/defs.hpp"

#include "prx/mujoco/mj_utils.hpp"
#include "prx/mujoco/mj_manipulation.hpp"
#include "prx/mujoco/mj_simulator.hpp"

#include "prx/planning/planners/dirt.hpp"

#include <boost/filesystem.hpp>
using namespace boost::filesystem;

using namespace prx;

int main(int argc, char* argv[])
{
    param_loader params;
    if (argc < 2)
    {
        params = param_loader("examples/JIST/franka_steer.yaml");
    }
    else
    {
        params = param_loader(argv[1]);
    }
    init_random(params["random_seed"].as<int>());

// Arm simulator
    std::shared_ptr<prx::mujoco_simulator_t> sim =
        std::make_shared<prx::mujoco_simulator_t>(params["scene_xml_path"].as<std::string>(), false);
    sim->init_simulator();

    auto context = sim->get_context("mujoco");
    auto ss = context.first->get_state_space();
    auto cs = context.first->get_control_space();

    auto start_state = ss->make_point();
    ss->copy_to(start_state);

    auto start_state_vec = Vec(start_state);

// End-effector simulator
    std::shared_ptr<prx::mujoco_simulator_t> ee_sim =
        std::make_shared<prx::mujoco_simulator_t>(params["ee_xml_path"].as<std::string>());
    ee_sim->init_simulator();

    auto ee_context = ee_sim->get_context("mujoco");
    auto ee_ss = ee_context.first->get_state_space();
    auto ee_cs = ee_context.first->get_control_space();
    for (int i = 0; i < 100; i++)
    {
        sim->step_simulation();
        ee_sim->step_simulation();
    }

// YAML parameters

    std::vector<std::string> joint_names = params["joint_names"].as<std::vector<std::string>>();
    auto arm_qpos_inds = get_indices(sim->m, mjOBJ_JOINT, joint_names);
    auto arm_ctrl_inds = get_indices(sim->m, mjOBJ_ACTUATOR, joint_names);

    std::string hand = params["forward_name"].as<std::string>();
    auto hand_qpos_inds = get_body_indices(ee_sim->m, hand);
    auto hand_ctrl_inds = get_indices(sim->m, mjOBJ_ACTUATOR, hand);


    std::vector<std::string> ee_names = params["end_effector"].as<std::vector<std::string>>();

// Steering 

    Eigen::Vector<double, 7> q_steer_start(params["steer_start"].as<std::vector<double>>().data());
    Eigen::Vector<double, 7> q_steer_goal(params["steer_goal"].as<std::vector<double>>().data());
    auto grasp = forward_kinematics(sim->m, sim->d, arm_qpos_inds, hand, q_steer_goal);

    Eigen::Vector<double, 7> q_plan_start = start_state_vec(arm_qpos_inds);

    // Desired pose of "hand" frame
    Eigen::Vector<double, 7> pregrasp = calculate_pregrasp(grasp);
    std::vector<Eigen::VectorXd> ee_transforms{};

    /*
    for(auto ee_name : ee_names){
        std::cout << forward_kinematics(ee_sim->m, ee_sim->d, hand_qpos_inds, ee_name, pregrasp).transpose() << std::endl;
        ee_sim->step_simulation();
        sleep(5);
        if (strcmp(ee_name.c_str(), hand.c_str()) == 0){
            Eigen::Vector<double, 7> temp_pose{0, 0, 0, 1, 0, 0, 0};
            ee_transforms.push_back(temp_pose);
            continue;
        }
        
        int temp_body_id = mj_name2id(ee_sim->m, mjOBJ_BODY, ee_name.c_str());
        Eigen::VectorXd temp_pose(7);
        temp_pose({0, 1, 2}) = vector_t{ee_sim->m->body_pos[3*temp_body_id], ee_sim->m->body_pos[3*temp_body_id+1], ee_sim->m->body_pos[3*temp_body_id+2]};
        temp_pose({3, 4, 5, 6}) = Eigen::Vector<double, 4>{ee_sim->m->body_quat[4*temp_body_id], ee_sim->m->body_quat[4*temp_body_id+1], 
            ee_sim->m->body_quat[4*temp_body_id+2], ee_sim->m->body_quat[4*temp_body_id+3]};

        ee_transforms.push_back(temp_pose);
        
        int temp_ee_id = mj_name2id(ee_sim->m, mjOBJ_BODY, ee_name.c_str());
    }
    */

    std::cout << "pregrasp" << std::endl;
    for (auto pos : pregrasp){
        std::cout << pos << " ";
    }
    std::cout << std::endl;

    std::cout << "qpos_indices" << std::endl;
    for (auto x_ind : get_body_qpos_indices(sim->m, hand)){
        std::cout << x_ind << " ";
    }
    std::cout << std::endl;

    std::cout << "left_finger indices" << std::endl;
    for (auto ind : get_body_qpos_indices(ee_sim->m, "left_finger")){
        std::cout << ind << " ";
    }
    std::cout << std::endl;

    std::cout << "right_finger indices" << std::endl;
    for (auto ind : get_body_qpos_indices(ee_sim->m, "right_finger")){
        std::cout << ind << " ";
    }
    std::cout << std::endl;

    std::cout << "x_indices" << std::endl;
    for (auto x_ind : get_body_indices(sim->m, hand)){
        std::cout << x_ind << " ";
    }
    std::cout << std::endl;

    std::cout << "left_finger indices" << std::endl;
    for (auto ind : get_body_indices(ee_sim->m, "left_finger")){
        std::cout << ind << " ";
    }
    std::cout << std::endl;

    std::cout << "right_finger indices" << std::endl;
    for (auto ind : get_body_indices(ee_sim->m, "right_finger")){
        std::cout << ind << " ";
    }
    std::cout << std::endl;

    int ind;
    double pos;
    for(int i = 0; i < 1; i++){
        std::cout << "iteration " << i << std::endl;
        
        ee_sim->d->qpos[0] = .497037;
        ee_sim->d->qpos[1] = 0;
        ee_sim->d->qpos[2] = .171616;
        ee_sim->d->qpos[3] = .0179892;
        ee_sim->d->qpos[4] = .0330859;
        ee_sim->d->qpos[5] = .99929;
        ee_sim->d->qpos[6] = 0;

        ee_sim->step_simulation(-1);

        std::cout << ee_sim->d->xpos[27] << " " << ee_sim->d->xpos[28] << " " << ee_sim->d->xpos[29] << std::endl;
        std::cout << ee_sim->d->xpos[6] << " " << ee_sim->d->xpos[7] << " " << ee_sim->d->xpos[8] << std::endl;
        std::cout << ee_sim->d->xpos[9] << " " << ee_sim->d->xpos[10] << " " << ee_sim->d->xpos[11] << std::endl;

        /*
        ee_sim->d->xpos[27] = .497037;
        ee_sim->d->xpos[28] = 0;
        ee_sim->d->xpos[29] = .171616;
        ee_sim->d->xquat[36] = .0179892;
        ee_sim->d->xquat[37] = .0330859;
        ee_sim->d->xquat[38] = .99929;
        ee_sim->d->xquat[39] = 0;
        */

       
        // std::cout << ee_sim->d->qpos[0] << " " << ee_sim->d->qpos[1] << " " << ee_sim->d->qpos[2] << " " << std::endl;
        // std::cout << pregrasp.transpose() << std::endl;
        
    }
    
    std::cout << "End of program!" << std::endl;
}
