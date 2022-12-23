#include "prx/mujoco/mj_simulator.hpp"
#include "prx/simulation/playback/plan.hpp"

using namespace prx;

int main(int argc, char** argv)
{
    std::vector<double> plan_to_playback = 
    {
       -1.9251 , 1.72,
        -0.8954 , 0.5,
        2.6199 , 1.26,
        -1.3660 , 0.66,
        2.1589 , 0.58,
        -2.2905 , 0.58,
        2.0933 , 0.74,
        -2.8685 , 0.52,
        1.6676 , 0.52
    };

    std::shared_ptr<mujoco_simulator_t> sim = std::make_shared<mujoco_simulator_t>("cartpole.xml");
    sim->init_simulator();

    auto context = sim -> get_context("mujoco");
    auto ss = context.first -> get_state_space();
    auto cs = context.first -> get_control_space();

    init_random(111093);
    space_point_t start = ss -> make_point();
    space_point_t end = ss -> make_point();
    start -> at(2) = PRX_PI;

    plan_t plan(cs);
    for (int i = 0; i < plan_to_playback.size(); i+=2)
    {
        plan.append_onto_back(plan_to_playback[i+1]);
        plan.back().control -> at(0) = plan_to_playback[i];
    }

    while(true)
    {
        context.first -> propagate(start, plan, end);
        usleep(int(1e6));
    }
}