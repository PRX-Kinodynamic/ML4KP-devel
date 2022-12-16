#include "prx/mujoco/mj_simulator.hpp"
#include "prx/planning/planners/dirt.hpp"

#include <fstream>

using namespace prx;

int main(int argc, char* argv[])
{
    init_random(210896);

    std::shared_ptr<mujoco_simulator_t> sim = std::make_shared<mujoco_simulator_t>("cartpole.xml");
    sim->init_simulator();

    auto context = sim -> get_context("mujoco");
    auto ss = context.first -> get_state_space();
    auto cs = context.first -> get_control_space();

    dirt_t dirt("dirt");
    dirt_specification_t dirt_spec(context.first, context.second);

    dirt_spec.valid_state = [](const space_point_t& point)
    {
        return true;
    };

    dirt_spec.valid_check = [&dirt_spec](trajectory_t& traj)
    {
        return true;
    };

    dirt_spec.distance_function = [&dirt_spec](const space_point_t& point1, const space_point_t& point2)
    {
        return dirt_spec.state_space -> euclidean_2d(point1, point2, 0, 4);
    };

    dirt_spec.h = [&dirt_spec](const space_point_t& point1, const space_point_t& point2)
    {
        return dirt_spec.state_space -> euclidean_2d(point1, point2, 2, 4);
    };

    dirt_spec.min_control_steps = 25;
    dirt_spec.max_control_steps = 100;
    dirt_spec.blossom_number = 5;

    dirt_query_t dirt_query(ss,cs);
    dirt_query.start_state = ss -> make_point();
    dirt_query.goal_state = ss -> make_point();
    dirt_query.start_state -> at(2) = PRX_PI;

    dirt_query.goal_check = [&](const space_point_t& point)
    {
        return (point->at(2) * point->at(2) + point->at(3) * point->at(3)) < 0.11;
    };

    dirt.link_and_setup_spec(&dirt_spec);
    dirt.preprocess();
    dirt.link_and_setup_query(&dirt_query);

    condition_check_t checker("time", 10.0);
    dirt.resolve_query(&checker);
    dirt.fulfill_query(); 

    std::cout << dirt_query.solution_plan.print(4) << std::endl;
}