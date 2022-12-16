#include "prx/mujoco/mj_simulator.hpp"
#include "prx/simulation/playback/plan.hpp"

using namespace prx;

int main(int argc, char** argv)
{
    std::shared_ptr<mujoco_simulator_t> sim = std::make_shared<mujoco_simulator_t>("cartpole.xml");
    sim->init_simulator();

    auto context = sim -> get_context("mujoco");
    auto ss = context.first -> get_state_space();
    auto cs = context.first -> get_control_space();

    init_random(111093);
    space_point_t start = ss -> make_point();
    space_point_t end  = ss -> make_point();
    ss -> sample(start);

    plan_t plan(cs);
    for (int i = 0; i < 10; i++)
    {
        plan.append_onto_back(1.0);
        cs -> sample(plan.back().control);
    }

    for (int i = 0; i < 30; i++)
    {
        context.first -> propagate(start, plan, end);
        std::cout << ss -> print_point(end) << std::endl;
    }
}