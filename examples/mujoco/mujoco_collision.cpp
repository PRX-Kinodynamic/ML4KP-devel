#include "prx/mujoco/mj_simulator.hpp"
#include "prx/utilities/general/timer.hpp"

using namespace prx;

int main(int argc, char** argv)
{
    std::shared_ptr<mujoco_simulator_t> sim = std::make_shared<mujoco_simulator_t>("diffdrive_collision.xml");
    sim->init_simulator();

    auto context = sim -> get_context("mujoco");
    auto ss = context.first -> get_state_space();
    auto cs = context.first -> get_control_space();

    space_point_t start = ss -> make_point();
    space_point_t end  = ss -> make_point();
    for (double i = 0; i < 1.0/simulation_step; i += 1)
    {
        sim -> step_simulation(propagate_step::FIRST_STEP);
    }
    ss -> copy_to_point(start);
    std::cout << "Collision? " << sim -> in_collision() << std::endl;

    init_random(111093);
    plan_t plan(cs);
    plan.append_onto_back(3.0);
    plan.back().control -> at(0) = 1.0;
    plan.back().control -> at(1) = 1.0;

    context.first -> propagate(start, plan, end);
    std::cout << ss -> print_point(end,4) << std::endl;
    std::cout << "Collision? " <<  sim -> in_collision() << std::endl;
}