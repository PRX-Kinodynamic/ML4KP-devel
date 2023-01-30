#include "prx/mujoco/mj_simulator.hpp"
#include "prx/planning/planners/dirt.hpp"

#include <fstream>

using namespace prx;

int main(int argc, char* argv[])
{
    init_random(210896);

    // std::shared_ptr<mujoco_simulator_t> sim = std::make_shared<mujoco_simulator_t>("diffdrive_collision.xml");
    std::shared_ptr<mujoco_simulator_t> sim = std::make_shared<mujoco_simulator_t>("mushr.xml");
    sim->init_simulator();

    auto context = sim -> get_context("mujoco");
    auto ss = context.first -> get_state_space();
    auto cs = context.first -> get_control_space();

    for (double i = 0; i < 1.0/simulation_step; i += 1)
    {
        sim -> step_simulation(propagate_step::FIRST_STEP);
    }

    dirt_t dirt("dirt");
    dirt_specification_t dirt_spec(context.first, context.second);
    dirt_spec.blossom_number = 1;

    dirt_spec.distance_function = [&dirt_spec](const space_point_t& point1, const space_point_t& point2)
    {
        return dirt_spec.state_space -> euclidean_2d(point1, point2, 0, 2);
    };

    dirt_spec.min_control_steps = 0.5 * (1.0/simulation_step);
    dirt_spec.max_control_steps = 2.0 * (1.0/simulation_step);

    dirt_query_t dirt_query(ss,cs);
    dirt_query.start_state = ss -> make_point();
    dirt_query.goal_state = ss -> make_point();
    ss -> copy_to_point(dirt_query.start_state);
    dirt_query.start_state ->at(0) = -8.0;
    dirt_query.start_state ->at(1) = 0.0;
    ss -> copy_to_point(dirt_query.goal_state);
    dirt_query.goal_state -> at(0) = 8.0;
    dirt_query.goal_state -> at(1) = 0.0;
    std::cout << ss -> print_point(dirt_query.start_state, 4) << std::endl;
    std::cout << ss -> print_point(dirt_query.goal_state, 4) << std::endl;

    dirt_query.goal_check = [&](const space_point_t& point)
    {
        return dirt_spec.distance_function(point, dirt_query.goal_state) < 0.5;
    };

    dirt_query.get_visualization = true;

    dirt.link_and_setup_spec(&dirt_spec);
    dirt.preprocess();
    dirt.link_and_setup_query(&dirt_query);

    condition_check_t checker("time", 60.0);
    dirt.resolve_query(&checker);
    dirt.fulfill_query(); 

    std::ofstream fout;
    unsigned counter = 0;
    for (auto& traj : dirt_query.tree_visualization)
    {
        fout.open(output_path + "tree" + std::to_string(counter) + ".txt");
        fout << traj.print(2);
        fout.close();
        counter++;
    }

    fout.open(output_path + "solution.txt");
    fout << dirt_query.solution_traj.print(4);
    fout.close();

    std::cout << dirt_query.solution_plan.print(4) << std::endl;
}