#include "prx/mujoco/mj_simulator.hpp"
#include "prx/planning/planners/rrt.hpp"

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

    rrt_t rrt("rrt");
    rrt_specification_t rrt_spec(context.first, context.second);

    rrt_spec.distance_function = [&rrt_spec](const space_point_t& point1, const space_point_t& point2)
    {
        return rrt_spec.state_space -> euclidean_2d(point1, point2, 0, 2);
    };

    rrt_spec.min_control_steps = 0.5 * (1.0/simulation_step);
    rrt_spec.max_control_steps = 2.0 * (1.0/simulation_step);

    rrt_query_t rrt_query(ss,cs);
    rrt_query.start_state = ss -> make_point();
    rrt_query.goal_state = ss -> make_point();
    ss -> copy_to_point(rrt_query.start_state);
    ss -> copy_to_point(rrt_query.goal_state);
    rrt_query.goal_state -> at(0) += 5.0;
    rrt_query.goal_state -> at(1) += 5.0;
    std::cout << ss -> print_point(rrt_query.start_state, 4) << std::endl;
    std::cout << ss -> print_point(rrt_query.goal_state, 4) << std::endl;

    rrt_query.goal_check = [&](const space_point_t& point)
    {
        return rrt_spec.distance_function(point, rrt_query.goal_state) < 0.5;
    };

    rrt_query.get_visualization = true;

    rrt.link_and_setup_spec(&rrt_spec);
    rrt.preprocess();
    rrt.link_and_setup_query(&rrt_query);

    condition_check_t checker("time", 5.0);
    rrt.resolve_query(&checker);
    rrt.fulfill_query(); 

    std::ofstream fout;
    unsigned counter = 0;
    for (auto& traj : rrt_query.tree_visualization)
    {
        fout.open(output_path + "tree" + std::to_string(counter) + ".txt");
        fout << traj.print(2);
        fout.close();
        counter++;
    }

    fout.open(output_path + "solution.txt");
    fout << rrt_query.solution_traj.print(4);
    fout.close();

    std::cout << rrt_query.solution_plan.print(4) << std::endl;
}