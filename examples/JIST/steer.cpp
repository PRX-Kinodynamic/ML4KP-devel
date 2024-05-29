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
        std::make_shared<prx::mujoco_simulator_t>(params["scene_xml_path"].as<std::string>());
    sim->init_simulator();

    auto context = sim->get_context("mujoco");
    auto ss = context.first->get_state_space();
    auto cs = context.first->get_control_space();

    auto start_state = ss->make_point();
    ss->copy_to(start_state);

    auto start_state_vec = Vec(start_state);

// End-effector simulator
    std::shared_ptr<prx::mujoco_simulator_t> ee_sim =
        std::make_shared<prx::mujoco_simulator_t>(params["ee_xml_path"].as<std::string>(), false);
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

    std::vector<std::string> ee_names = params["end_effector"].as<std::vector<std::string>>();

// Steering 

    Eigen::Vector<double, 7> q_steer_start(params["steer_start"].as<std::vector<double>>().data());
    Eigen::Vector<double, 7> q_steer_goal(params["steer_goal"].as<std::vector<double>>().data());
    auto grasp = forward_kinematics(sim->m, sim->d, arm_qpos_inds, hand, q_steer_goal);

    Eigen::Vector<double, 7> q_plan_start = start_state_vec(arm_qpos_inds);

    // Desired pose of "hand" frame
    Eigen::Vector<double, 7> pregrasp = calculate_pregrasp(grasp);
    std::vector<pose_t> ee_transforms{};

    for(auto ee_name : ee_names){
        std::cout << forward_kinematics(ee_sim->m, ee_sim->d, hand_qpos_inds, ee_name, pregrasp).transpose() << std::endl;
        if (strcmp(ee_name.c_str(), hand.c_str()) == 0){
            pose_t temp_pose{0, 0, 0, 1, 0, 0, 0};
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
    
    auto test = Eigen::Vector<double, 7>{0, 0, 0, 1, 0, 0, 0};
    auto temp = compose_transformations(pregrasp, test);
    
    auto x_steer = forward_kinematics(sim->m, sim->d, arm_qpos_inds, hand, q_steer_goal);
    int body = mj_name2id(sim->m, mjOBJ_BODY, "hand");

// Planning 

    dirt_t dirt(params["planner_name"].as<>());
    dirt_specification_t dirt_spec(context.first, context.second);

    dirt_spec.distance_function = [&](const space_point_t& a, const space_point_t& b){
        double dist = 0;

        auto pose_a = forward_kinematics(sim->m, sim->d, arm_qpos_inds, hand, a);
        auto pose_b = forward_kinematics(sim->m, sim->d, arm_qpos_inds, hand, b);
        
        return task_distance(pose_a, pose_b, ee_transforms);
    };

    /*
    dirt_spec.distance_function = [&](const space_point_t& a, const space_point_t& b) {
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

        return dist/ee_names.size();
    };*/

    float min_control_scaling = 0.5;
    float max_control_scaling = 1.0;
    dirt_spec.min_control_steps = min_control_scaling * (1.0 / simulation_step);
    dirt_spec.max_control_steps = max_control_scaling * (1.0 / simulation_step);

    // dirt query
    dirt_query_t dirt_query(context.first->get_state_space(), context.first->get_control_space());
    dirt_query.get_visualization = params["visualize"].as<bool>();

    dirt_query.goal_state = context.first->get_state_space()->make_point();
    dirt_query.start_state = context.first->get_state_space()->make_point();
    ss->copy_to(dirt_query.start_state);

    // goal region radius
    dirt_query.goal_region_radius = params["goal_region_radius"].as<double>();

    // goal
    /*
    std::vector<double> goal_vec = params["goal_config"].as<std::vector<double>>();
    for (int i = 0; i < goal_vec.size(); i++)
    {
        dirt_query.goal_state->at(i) = goal_vec[i];
    }
    */

    // check whether sampled state is in goal region
    dirt_query.goal_check = [&](const space_point_t& point) {
        auto pose = forward_kinematics(sim->m, sim->d, arm_qpos_inds, hand, point);

        // NOTE: may be able to improve this implementation
        if (task_distance(pose, pregrasp, ee_transforms) >= dirt_query.goal_region_radius)
            return false;
        
        // call steering function here
        return dirt_spec.distance_function(point, dirt_query.goal_state) < dirt_query.goal_region_radius;
    };
    
    condition_check_t checker(params["checker_type"].as<>(), params["checker_value"].as<int>());
    dirt.link_and_setup_spec(&dirt_spec);
    dirt.preprocess();
    dirt.link_and_setup_query(&dirt_query);
    dirt.resolve_query(&checker);
    dirt.fulfill_query();

// OUTPUT
    
    if(params["output_plan"].as<bool>()){
        dirt_query.solution_plan.to_file(params["output_path"].as<std::string>());
    }

    trajectory_t traj{ss};
    // jacobian_steering(sim->m, sim->d, traj, x_goal, body, arm_qpos_inds);
    // steer_test(sim, traj, x_steer, body, arm_qpos_inds);

    // std::cout << jac << std::endl;

    std::cout << "End of program!" << std::endl;
}
