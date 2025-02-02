#include "prx/utilities/defs.hpp"
#include "prx/mujoco/mj_simulator.hpp"

using namespace prx;

int main()
{
    // Load parameters
    param_loader params("examples/tasks/push_plan.yaml");
    init_random(params["random_seed"].as<int>());

    // Initialize simulator
    bool visualize = params["visualize"].as<bool>();
    std::string xml_path = params["xml_path"].as<std::string>();
    std::shared_ptr<mujoco_simulator_t> sim = 
        std::make_shared<mujoco_simulator_t>(xml_path, visualize);
    sim->init_simulator();
    // warm up the simulator
    sim->step_simulation();

    auto context = sim->get_context("mujoco");
    auto sg = context.first;
    auto ss = sg->get_state_space();
    auto cs = sg->get_control_space();

    // template function to check if the cylinder is in the goal region
    auto is_in_goal_region = [](double* pos, double* goal_pos)
    {
        return (pos[0] - goal_pos[0]) * (pos[0] - goal_pos[0]) + (pos[1] - goal_pos[1]) * (pos[1] - goal_pos[1]) < 0.01;
    };

    // random goal position
    double goal_x = (rand() / (double)RAND_MAX) * 2 - 1;
    double goal_y = (rand() / (double)RAND_MAX) * 2 - 1;

    space_point_t goal_state = ss->make_point();
    goal_state->at(0) = goal_x;
    goal_state->at(1) = goal_y;

    auto robot_id = mj_name2id(sim->m, mjOBJ_GEOM, "robot");
    auto cylinder_id = mj_name2id(sim->m, mjOBJ_GEOM, "cylinder");

    // Update robot state
    double* init_robot_pos = &sim->d->geom_xpos[3*robot_id];

    auto robot_state = ss->make_point();
    // Get cylinder position
    double* cylinder_pos = &sim->d->geom_xpos[3*cylinder_id];
    
    // Calculate angle between cylinder and goal
    double dx = 1.0 - cylinder_pos[0];
    double dy = 1.0 - cylinder_pos[1]; // goal_y - cylinder_pos[1];
    double angle = atan2(dy, dx);
    std::cout << "Angle: " << angle << std::endl;
    // Calculate robot position on cylinder circumference
    // Assuming cylinder radius is 0.5 (adjust as needed)
    double cylinder_radius = 0.2;  
    double robot_radius = 0.05;    // Adjust based on your robot size
    double total_radius = cylinder_radius + robot_radius;
    
    // Position robot behind cylinder (opposite to goal direction)
    double robot_x = cylinder_pos[0] - total_radius * cos(angle);
    double robot_y = cylinder_pos[1] - total_radius * sin(angle);
    
    // Set robot position
    sim->d->qpos[0] = robot_x - init_robot_pos[0];
    sim->d->qpos[1] = robot_y - init_robot_pos[1];
    sim->d->qvel[0] = 0.0;
    sim->d->qvel[1] = 0.0;
    
    for(int i = 0; i < 100; i++)
    {
        sim->step_simulation();
    }
    
    space_point_t start_state = ss->make_point();
    ss->copy_to(start_state);

    std::cout << "Start state: " << start_state->at(0) << ", " << start_state->at(1) << ", " << start_state->at(2) << std::endl;

    // int steps = 100;
    // double multiplier = 1 / simulation_step;
    double duration = 1.0; // steps / multiplier;

    trajectory_t* traj = new trajectory_t(ss);
    plan_t* plan = new plan_t(cs);
    for(int i = 0; i < 100; i++)
    {
        traj->clear();
        plan->clear();
        plan->append_onto_back(duration);
        space_point_t control = cs->make_point();

        // push in the direction of the goal
        control->at(0) = cos(angle);
        control->at(1) = sin(angle);
        plan->back().control = control;
        sg->propagate(start_state, *plan, *traj);

        ss->copy_from(traj->back());
        ss->copy_to(start_state);
    }
    return 0;
}